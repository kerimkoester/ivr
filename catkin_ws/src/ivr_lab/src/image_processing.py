#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send messages to a topic named image_topic
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.callback)

  # Recieve data, process it, and publish
  def callback(self,data):
    # Recieve the image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      
      
    # loading template for links as binary image (used in lab 2)
  #  self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
  #  self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
  #  self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))   
    
    # Perform image processing task (your code goes here)
    # The image is loaded as cv_imag

    # Uncomment if you want to save the image
    # cv2.imwrite('image_copy.png', cv_image)

    #cv2.imshow('window', cv_image)
    #cv2.waitKey(3)
    #cv2.imshow('window', cv_image)
    #cv2.waitKey(3)
    #blue circle
    def measure_angle(img):
      ret,thresh_B = cv2.threshold(img[:,:,0],50,255,cv2.THRESH_BINARY)
      ret,thresh_G = cv2.threshold(img[:,:,1],50,255,cv2.THRESH_BINARY_INV)
      ret,thresh_R = cv2.threshold(img[:,:,2],50,255,cv2.THRESH_BINARY_INV)
      
      combined_B = thresh_R*thresh_G*thresh_B
      
      #yellow circle
      ret,thresh_B = cv2.threshold(img[:,:,0],50,255,cv2.THRESH_BINARY_INV)
      ret,thresh_G = cv2.threshold(img[:,:,1],50,255,cv2.THRESH_BINARY)
      ret,thresh_R = cv2.threshold(img[:,:,2],50,255,cv2.THRESH_BINARY)
      
      combined_Y = thresh_R*thresh_G*thresh_B
      
      #blue circle
      ret,thresh_B = cv2.threshold(img[:,:,0],50,255,cv2.THRESH_BINARY_INV)
      ret,thresh_G = cv2.threshold(img[:,:,1],50,255,cv2.THRESH_BINARY_INV)
      ret,thresh_R = cv2.threshold(img[:,:,2],50,255,cv2.THRESH_BINARY)
      
      combined_R = thresh_R*thresh_G*thresh_B

      #green circle
      ret,thresh_B = cv2.threshold(img[:,:,0],50,255,cv2.THRESH_BINARY_INV)
      ret,thresh_G = cv2.threshold(img[:,:,1],50,255,cv2.THRESH_BINARY)
      ret,thresh_R = cv2.threshold(img[:,:,2],50,255,cv2.THRESH_BINARY_INV)
      
      combined_G = thresh_R*thresh_G*thresh_B
      
      
      #moments
      centerposition_YBGR = np.zeros([4,2])
      #yellow
      _,contours,hierarchy = cv2.findContours(combined_Y,1,2)
      cnt = contours[0]
      M = cv2.moments(cnt)
      centerposition_YBGR[0,0],centerposition_YBGR[0,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])

      _,contours,hierarchy = cv2.findContours(combined_B,1,2)
      cnt = contours[0]
      M = cv2.moments(cnt)
      centerposition_YBGR[1,0],centerposition_YBGR[1,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])

      _,contours,hierarchy = cv2.findContours(combined_G,1,2)
      cnt = contours[0]
      M = cv2.moments(cnt)
      centerposition_YBGR[2,0],centerposition_YBGR[2,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])
  
      _,contours,hierarchy = cv2.findContours(combined_R,1,2)
      cnt = contours[0]
      M = cv2.moments(cnt)
      centerposition_YBGR[3,0],centerposition_YBGR[3,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])
      
      distance = centerposition_YBGR[1:,:]-centerposition_YBGR[:-1,:]
      avg_distance = np.sum(np.sqrt(np.sum(distance**2,axis=1)))/3
      
      convert = 3/avg_distance
      centerposition_YBGR -= 400
      centerposition_YBGR *= convert
  
      arm0 = np.array([0,-3])
      arm1 = centerposition_YBGR[1]-centerposition_YBGR[0]
      arm2 = centerposition_YBGR[2]-centerposition_YBGR[1]
      arm3 = centerposition_YBGR[3]-centerposition_YBGR[2]
      angle1 = np.arccos(np.dot(arm0,arm1)/np.linalg.norm(arm0)/np.linalg.norm(arm1))
      angle2 = np.arccos(np.dot(arm1,arm2)/np.linalg.norm(arm1)/np.linalg.norm(arm2))
      angle3 = np.arccos(np.dot(arm2,arm3)/np.linalg.norm(arm2)/np.linalg.norm(arm3))
      angle = np.array([angle1,angle2,angle3])
      return angle


    # change te value of self.joint.data to your estimated value from thew images once you have finalized the code
    angles = measure_angle(cv_image)
    self.joints = Float64MultiArray()
    self.joints.data = angles

    # Publish the results - the images are published under a topic named "image_topic" and calculated joints angles are published under a topic named "joints_pos"
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


