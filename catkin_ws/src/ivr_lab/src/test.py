import numpy as np
import cv2

img = cv2.imread('image_copy.png',1)

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
centerposition_YBGR *= convert

arm0 = np.array([0,-3])
arm1 = centerposition_YBGR[1]-centerposition_YBGR[0]
arm2 = centerposition_YBGR[2]-centerposition_YBGR[1]
arm3 = centerposition_YBGR[3]-centerposition_YBGR[2]
angle1 = np.arccos(np.dot(arm0,arm1)/np.linalg.norm(arm0)/np.linalg.norm(arm1))
angle2 = np.arccos(np.dot(arm1,arm2)/np.linalg.norm(arm1)/np.linalg.norm(arm2))
angle3 = np.arccos(np.dot(arm2,arm3)/np.linalg.norm(arm2)/np.linalg.norm(arm3))

print(angle1,angle2,angle3)







