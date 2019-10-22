import cv2
import matplotlib.pyplot as plt
import numpy as np

img = cv2.imread('image_copy.png',1)

def isolate_links_and_crop(img):
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


link_tot,link1_cropped,link2_cropped,link3_cropped, YBGR = isolate_links_and_crop(img)

def get_quadrant(lower_blob,upper_blob):
    vec = upper_blob-lower_blob
    if vec[0]<=0:
        if vec[1]>=0:
            return 1
        else:
            return 2
    else:
        if vec[1]<=0:
            return 3
        else:
            return 4
            
    
    

link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))  
rows,cols = link1.shape

def rotate_im(im,angle):
    M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
    return cv2.warpAffine(im,M,(cols,rows))

def find_angle(template,image,N_steps=3600):
    image_inv = cv2.bitwise_not(image)
    image_dist = cv2.distanceTransform(image_inv,cv2.DIST_L2,cv2.DIST_MASK_5)
    angles = np.linspace(0,360,N_steps)
    min_angle = 0
    min_dist = np.inf
    for angle in angles:
        temp = rotate_im(template,angle)
        dist = (temp*image_dist).sum()
        if dist < min_dist:
            min_angle = angle
            min_dist = dist
    return min_angle*2*np.pi/360,min_dist
for angle in range(360):
    cv2.imshow('window',rotate_im(link1,angle))
    cv2.waitKey(50)

























