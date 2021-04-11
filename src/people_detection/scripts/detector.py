#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from object_detection import object_detector

def drawPred(frame, objects_detected):

    objects_list = list(objects_detected.keys())

    for object_, info in objects_detected.items():
        box = info[0]
        confidence = info[1]
        label = '%s: %.2f' % (object_, confidence)
        p1 = (int(box[0]), int(box[1]))
        p2 = (int(box[0] + box[2]), int(box[1] + box[3]))
        cv.rectangle(frame, p1, p2, (0, 255, 0))
        left = int(box[0])
        top = int(box[1])
        labelSize, baseLine = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv.rectangle(frame, (left, top - labelSize[1]), (left + labelSize[0], top + baseLine), (255, 255, 255), cv.FILLED)
        cv.putText(frame, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))  
       

def postprocess(frame, out, threshold, classes, framework):

    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]
    objects_detected = dict()

    if framework == 'Caffe':
        # Network produces output blob with a shape 1x1xNx7 where N is a number of
        # detections and an every detection is a vector of values
        # [batchId, classId, confidence, left, top, right, bottom]
        for detection in out[0, 0]:
            confidence = detection[2]
            if confidence > threshold:
                left = int(detection[3] * frameWidth)
                top = int(detection[4] * frameHeight)
                right = int(detection[5] * frameWidth)
                bottom = int(detection[6] * frameHeight)
                #classId = int(detection[1]) - 1  # Skip background label
                
                classId = int(detection[1])
                i = 0
                label = classes[classId]
                label_with_num = str(label) + '_' + str(i)
                while(True):
                    if label_with_num not in objects_detected.keys():
                        break
                    label_with_num = str(label) + '_' + str(i)
                    i = i+1
                objects_detected[label_with_num] = [(int(left),int(top),int(right - left), int(bottom-top)),confidence] 
                #print(label_with_num + ' at co-ordinates '+ str(objects_detected[label_with_num]))

    else:
        # Network produces output blob with a shape NxC where N is a number of
        # detected objects and C is a number of classes + 4 where the first 4
        # numbers are [center_x, center_y, width, height]
        for detection in out:
            confidences = detection[5:]
            classId = np.argmax(confidences)
            confidence = confidences[classId]
            confidence=confidences[0]#remove all but person detections
            if confidence > threshold:
                center_x = int(detection[0] * frameWidth)
                center_y = int(detection[1] * frameHeight)
                width = int(detection[2] * frameWidth)
                height = int(detection[3] * frameHeight)
                left = center_x - (width / 2)
                top = center_y - (height / 2)
                
                i = 0
                label = classes[classId]
                label_with_num = str(label) + '_' + str(i)
                while(True):
                    if label_with_num not in objects_detected.keys():
                        break
                    label_with_num = str(label) + '_' + str(i)
                    i = i+1
                objects_detected[label_with_num] = [(int(left),int(top),int(width),int(height)),confidence]
                #print(label_with_num + ' at co-ordinates '+ str(objects_detected[label_with_num]))
    return objects_detected

def intermediate_detections(frame, predictor, threshold, classes):
    predictions = predictor.predict(frame)
    objects_detected = postprocess(frame, predictions, threshold, classes, predictor.framework)
        
    objects_list = list(objects_detected.keys())
    #print('Tracking the following objects', objects_list)
    #print(objects_detected.values())

    trackers_dict = dict()    
    #multi_tracker = cv.MultiTracker_create()

    if len(objects_list) > 0:
        
        trackers_dict = {key : cv.TrackerKCF_create() for key in objects_list}
        for item in objects_list:
            trackers_dict[item].init(frame, objects_detected[item][0])
            
    return frame, objects_detected, objects_list, trackers_dict

def process(cv_image):

    objects_detected = dict()
    frame = cv_image

    predictor = object_detector('model_data/yolov2.weights', 'model_data/yolov2.cfg')
    #stream = cv.VideoCapture('tmp.png')
    #window_name = "Tracking in progress"
    #cv.namedWindow(window_name, cv.WINDOW_NORMAL)
    #cv.setWindowProperty(window_name, cv.WND_PROP_AUTOSIZE, cv.WINDOW_AUTOSIZE)        
    #cv.moveWindow(window_name,10,10)

    
    with open('model_data/coco_classes.txt', 'rt') as f:
        classes = f.read().rstrip('\n').split('\n')
    
    frame, objects_detected, objects_list, trackers_dict = intermediate_detections(frame, predictor, 0.35, classes)
    
    if len(objects_detected) > 0:
        drawPred(frame, objects_detected)
        objects_list = list(objects_detected.keys())
        for object_, info in objects_detected.items():
    	    box = info[0]
    	    #label = '%s: %.2f' % (object_, confidence)
    	    point1 = (int(box[0]), int(box[1]))
    	    point2 = (int(box[0] + box[2]), int(box[1] + box[3]))
    	    mask1 = np.zeros(frame.shape[:2], dtype="uint8")
    	    cv.rectangle(mask1, (point1), (point2), 255, -1)
    	    #cv.rectangle(mask1, point1, point2,(255,255,255),thickness=-1)
    	    masked_data = cv.bitwise_and(frame, frame, mask=mask1)
    else:
        masked_data = np.zeros(frame.shape[:2], dtype="uint8")
    	
    #cv.imshow("Image window", frame)
    #cv.imshow("Masked window", masked_data)
    #cv.waitKey(1)    

    return masked_data


class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/Multisense/left/image_rect_color",Image,self.callback)
    self.image_pub = rospy.Publisher("/Multisense/left/masked_image",Image)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    output=True
    masked_frame = process(cv_image)
    #push masked_frame to publisher can then alter stero cloud to only show masked pixels
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(masked_frame, "bgr8"))
    except CvBridgeError as e:
      print(e)
 

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
