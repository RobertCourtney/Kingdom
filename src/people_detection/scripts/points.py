#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.point_sub = rospy.Subscriber("/points",PointCloud2,self.callback)

  def callback(self,data):
    nparr = pcl2_2_np.msg_to_arr(data)
    print('test')
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
