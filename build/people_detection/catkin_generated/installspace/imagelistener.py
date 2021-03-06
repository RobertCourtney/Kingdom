#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub_left = rospy.Subscriber("/Multisense/left/image_rect_color",Image,self.callbackL)
    #self.image_sub_right = rospy.Subscriber("/Multisense/right/image_rect",Image,self.callbackR)
    #self.image_sub_left = rospy.Subscriber("/Multisense/image_points2_color",Image,self.callbackD)

  def callbackL(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Left Image window", cv_image)
    cv2.waitKey(3)
    
  def callbackR(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Right Image window", cv_image)
    cv2.waitKey(3)
    
  def callbackD(self,data):
    print("tesr")



def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
