import cv2
import numpy as np

img = cv2.imread('image_copy.png',1)

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
cv2.imwrite('link1_cropped.png', link1_cropped)
cv2.imwrite('link2_cropped.png', link2_cropped)
cv2.imwrite('link3_cropped.png', link3_cropped)




















