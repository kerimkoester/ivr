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
    self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
    self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
    self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))   
    
    # Perform image processing task (your code goes here)
    
    def isolate_links_and_crop(img):
        #Isolate links
        ret,thresh_B = cv2.threshold(img[:,:,0],50,255,cv2.THRESH_BINARY_INV)
        ret,thresh_G = cv2.threshold(img[:,:,1],50,255,cv2.THRESH_BINARY_INV)
        ret,thresh_R = cv2.threshold(img[:,:,2],50,255,cv2.THRESH_BINARY_INV)

        thresh_tot = thresh_B*thresh_G*thresh_R


        #___________________________________FIND CIRCLES_______________________________________________
        #blue circle
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
        #xx,contours,hierarchy = cv2.findContours(combined_Y,1,2)
        contours,hierarchy = cv2.findContours(combined_Y,1,2)
        cnt = contours[0]
        M = cv2.moments(cnt)
        centerposition_YBGR[0,0],centerposition_YBGR[0,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])

        contours,hierarchy = cv2.findContours(combined_B,1,2)
        cnt = contours[0]
        M = cv2.moments(cnt)
        centerposition_YBGR[1,0],centerposition_YBGR[1,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])

        contours,hierarchy = cv2.findContours(combined_G,1,2)
        cnt = contours[0]
        M = cv2.moments(cnt)
        centerposition_YBGR[2,0],centerposition_YBGR[2,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])

        contours,hierarchy = cv2.findContours(combined_R,1,2)
        cnt = contours[0]
        M = cv2.moments(cnt)
        centerposition_YBGR[3,0],centerposition_YBGR[3,1] = int(M['m10']/M['m00']),int(M['m01']/M['m00'])

        #____________________________center of links ______________________

        link1 = (centerposition_YBGR[0]+centerposition_YBGR[1])/2
        link2 = (centerposition_YBGR[1]+centerposition_YBGR[2])/2
        link3 = (centerposition_YBGR[2]+centerposition_YBGR[3])/2

        link_center = np.array([link1,link2,link3])


        link1_temp = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
        link2_temp = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
        link3_temp = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))

        link1_cropped = thresh_tot[int(link1[1]-50):int(link1[1]+50),int(link1[0]-30):int(link1[0]+30)]
        link2_cropped = thresh_tot[int(link2[1]-50):int(link2[1]+50),int(link2[0]-30):int(link2[0]+30)]
        link3_cropped = thresh_tot[int(link3[1]-50):int(link3[1]+50),int(link3[0]-30):int(link3[0]+30)]
        return thresh_tot,link1_cropped,link2_cropped,link3_cropped, centerposition_YBGR


    link_tot,link1_cropped,link2_cropped,link3_cropped, YBGR = isolate_links_and_crop(cv_image)
    cv2.imwrite("image_copy.png",cv_image)

    def get_quadrant(lower_blob,upper_blob):
        vec = upper_blob-lower_blob
        print(vec)
        if vec[0]<=0:
            if vec[1]<=0:
                return 0
            else:
                return 90
        else:
            if vec[1]>=0:
                return 180
            else:
                return 270
                
    
    link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
    link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
    link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))  
    rows,cols = link1.shape

    def rotate_im(im,angle):
        M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
        return cv2.warpAffine(im,M,(cols,rows))

    def find_angle(template,image,lower_blob,upper_blob,N_steps=100):
        image_inv = cv2.bitwise_not(image)
        image_dist = cv2.distanceTransform(image_inv,cv2.DIST_L2,cv2.DIST_MASK_5)
        min_angle = get_quadrant(lower_blob,upper_blob)-20
        angles = np.linspace(min_angle,min_angle+110,N_steps)
        min_dist = np.inf
        for angle in angles:
            temp = rotate_im(template,angle)
            dist = (temp*image_dist).sum()
            if dist < min_dist:
                min_angle = angle
                min_dist = dist
        return min_angle*2*np.pi/360,min_dist
    
    angle1,_ = find_angle(link1,link1_cropped,YBGR[0],YBGR[1])
    angle2,_ = find_angle(link2,link2_cropped,YBGR[1],YBGR[2])
    angle3,_ = find_angle(link3,link3_cropped,YBGR[2],YBGR[3])
    
    angle2 = angle2-angle1
    if angle2<0:
        angle2 += 2*np.pi
    angle3 = angle3-angle2-angle1
    if angle3<0:
        angle3 += 2*np.pi
    print(angle1,angle2,angle3)


    # change te value of self.joint.data to your estimated value from thew images once you have finalized the code
    self.joints = Float64MultiArray()
    self.joints.data = np.array([0, 0, 0])

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


