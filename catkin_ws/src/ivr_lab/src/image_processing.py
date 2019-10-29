#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
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
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
    # initialize a publisher to send desired trajectory
    self.trajectory_pub = rospy.Publisher("trajectory",Float64MultiArray, queue_size=10)
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.callback)
    # record the begining time
    self.time_trajectory = rospy.get_time()
    self.first_time = rospy.get_time()
    self.first_position = self.trajectory()
  
  # Define a circular trajectory (for lab 3)
  def trajectory(self):
    # get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    x_d = float(6* np.cos(cur_time * np.pi/100))
    y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    return np.array([x_d, y_d])
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

        contours,hierarchy = cv2.findContours(combined_R,3,2)
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

    def find_angle(template,image,lower_blob,upper_blob,N_steps=200):
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
    if angle1 > np.pi:
        angle1-=2*np.pi
    angle2,_ = find_angle(link2,link2_cropped,YBGR[1],YBGR[2])
    angle3,_ = find_angle(link3,link3_cropped,YBGR[2],YBGR[3])
    
    angle2 = angle2-angle1
    if angle2 > np.pi:
        angle2-=2*np.pi
    angle3 = angle3-angle2-angle1
    if angle3 > np.pi:
        angle3-=2*np.pi

    # change te value of self.joint.data to your estimated value from thew images once you have finalized the code
    self.joints = Float64MultiArray()
    self.joints.data = np.array([angle1,angle2,angle3])
    
    def trans_mat(theta,a=3):
        return np.array([[np.cos(theta),-np.sin(theta),np.cos(theta)*a],
                                [np.sin(theta),np.cos(theta),np.sin(theta)*a],
                                [0,0,1]])

    def trans_mat_tot(theta):
        return np.dot(trans_mat(theta[0]+np.pi/2),np.dot(trans_mat(theta[1]),trans_mat(theta[2])))
    
    def get_position(mat):
        return np.array([mat[0][2],mat[1][2]])
        
    def calc_endeff(ybgr):
        ybgr[0][1] *=-1
        ybgr[1][1] *=-1
        ybgr[2][1] *=-1
        ybgr[3][1] *=-1
        c = 3/np.linalg.norm(ybgr[1]-ybgr[0])
        ybgr *= c
        return ybgr[3]-ybgr[0]

    end_effector_lab3 = get_position(trans_mat_tot(self.joints.data))
    end_effector_lab1 = calc_endeff(YBGR)

    # publish the estimated position of robot end-effector (for lab 3)
    #x_e_image = np.reshape(np.array([end_effector_lab1, end_effector_lab3]),4)
    x_e_image = end_effector_lab1
    self.end_effector=Float64MultiArray()
    self.end_effector.data= x_e_image

    def xx(t1,t2,t3):
        return -3*(np.sin(t1)+np.sin(t1+t2)+np.sin(t1+t2+t3))
    def yy(t1,t2,t3):
        return 3*(np.cos(t1)+np.cos(t1+t2)+np.cos(t1+t2+t3))
    
    def jacobian(t):
        c1 = np.cos(t[0])
        c12 = np.cos(t[0]+t[1])
        c123 = np.cos(t[0]+t[1]+t[2])
        s1 = np.sin(t[0])
        s12 = np.sin(t[0]+t[1])
        s123 = np.sin(t[0]+t[1]+t[2])
        return 3*np.array([[-c1-c12-c123,-c12-c123,-c123],
                                [s1+s12+s123,s12+s123,s123]])
    
    J = jacobian(self.joints.data)
    J_inv = np.linalg.pinv(J)
    
    # Publishing the desired trajectory on a topic named trajectory(for lab 3)
    x_d = self.trajectory()    # getting the desired trajectory
    self.trajectory_desired= Float64MultiArray()
    self.trajectory_desired.data=x_d
    dx = x_d - self.first_position
    self.first_position = x_d
    
    current_time = rospy.get_time()
    dt = current_time - self.first_time
    self.first_time = current_time
    
    vel = (dx/dt)
    q_dot = np.dot(J_inv,vel)
    
    q_d = self.joints.data+dt*q_dot
    print([xx(*q_d),yy(*q_d)])
    print(x_d)
    
    # send control commands to joints (for lab 3)
    # q_d = [np.pi,0,0]
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
        

    # Publish the results
    try: 
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
      #(for lab 3)
      self.trajectory_pub.publish(self.trajectory_desired)
      self.end_effector_pub.publish(self.end_effector)
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
    except CvBridgeError as e:
      print(e)
      
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


