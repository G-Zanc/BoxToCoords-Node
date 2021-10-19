#!/usr/bin/env python
from re import X
import rospy
import cv2
from cv_bridge import CvBridge
import sys
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
import numpy as np
import message_filters

Known_distance = 76.2
Known_width = 14.3
 
# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
 
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
 
   focal_length = (width_in_rf_image * measured_distance) / real_width
   return focal_length
 
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
 
   distance = (real_face_width * Focal_Length)/face_width_in_frame
 
   return distance

def callback(camera_info, coordinates):
   camera_info_K = np.array(camera_info.K).reshape([3, 3])
   fx = camera_info_K[0][0]
   fy = camera_info_K[1][1]
   cx = camera_info_K[0][2]
   cy = camera_info_K[1][2]


   if(coordinates.right != -1):
      px, py, Distance = Distance_finder(Focal_Length_Finder(Known_distance, Known_width, (coordinates.right - coordinates.left)), 
                                 Known_width, (coordinates.right - coordinates.left))
      print(px)
      print(py)
      print(Distance)

      x = (px - cx) * Distance / fx
      y = (py - cy) * Distance / fy

      print(x)
      print(y)


if __name__ == '__main__':
   print("Node Started\n")
   
   rospy.init_node('my_node', anonymous=True)
   camera = rospy.get_param("/boxToCoords/camera", default="camera")
   Known_distance = rospy.get_param("/boxToCoords/Initial_Distance", default=15)
   Known_width = rospy.get_param("/boxToCoords/Known_Width", default = 14)
   
   info_sub = message_filters.Subscriber(camera + '/camera_info', CameraInfo)
   bboxCoords = message_filters.Subscriber('/bounding_box')
   ts = message_filters.ApproximateTimeSynchronizer([info_sub, bboxCoords], 10, 0.02)
   ts.registerCallback(callback)
   rospy.spin()
