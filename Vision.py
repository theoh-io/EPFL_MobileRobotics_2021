import cv2
import numpy as np
from PIL import Image 
import math
import matplotlib
from matplotlib.pyplot import imshow
from matplotlib import pyplot as plt
import time


#tuning parameters
#corners
iterations_erode=4
area_size=70 #used for corner detections
#obstacles
area_obst_min=500
area_obst_max=30000
thresh_low=90
thresh_up=255
#color ranges
green_lower=np.array([45,40,30])
green_upper=np.array([90,255,255])
red_lower=np.array([170,50,50])
red_upper=np.array([240,255,255])
yellow_lower=np.array([10,30,30])
yellow_upper=np.array([40,255,255])


def order_points(pts):
    #sorting points first by the 2nd the coordinate then 1st coordinate
    pts=sorted(pts, key=lambda x: (int(x[1]), int(x[0]))) #topleft,topright,bottomleft,bottomright
    #top left x > top right x => erreur et intervertit
    if pts[0][0] > pts[1][0]:
        pts[0], pts[1] = pts[1], pts[0]
    if pts[2][0] > pts[3][0]:
        pts[2], pts[3] = pts[3], pts[2]
    return pts


def four_point_transform(image, ordered_pts):
	# obtain a consistent order of the points and unpack them
	# individually
    (tl, tr, bl, br) = ordered_pts
    
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    
	# now that we have the dimensions of the new image, construct
	# the set of destination points to obtain a "birds eye view",
	# (i.e. top-down view) of the image, again specifying points
	# in the top-left, top-right, bottom-right, and bottom-left
	# order
    #conversion needed corner points => np.array
    ordered_pts=np.array(ordered_pts , dtype = "float32")
    dst = np.array([[0, 0],[maxWidth - 1, 0],[0, maxHeight - 1],[maxWidth - 1, maxHeight - 1]],dtype = "float32")
	# compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(ordered_pts, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
	
    return warped


def detectThymio(image):
    pts=[]
    p1=[]   #p1 is the big circle 
    p2=[]   #p2 is the little circle
    nb_iterations=1
    img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    #Create the color mask and then apply erode and dilate to reduce noise
    mask = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
    mask = cv2.erode(mask, None, iterations = nb_iterations)
    mask = cv2.dilate(mask, None, iterations = nb_iterations)
    #Apply the mask to find the contours of the two yellow circle
    elements,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #Find the center of both circles using cv2.minEnclosingCircle
    if (len(elements) == 2):
        #sorting the detected contours by descending area size
        elements.sort(key=cv2.contourArea, reverse=True)
        #finding the big circle
        c0=elements[0]
        ((x,y),rayon) = cv2.minEnclosingCircle(c0)
        #finding the little circle
        c1=elements[1]
        ((x2,y2),rayon2) = cv2.minEnclosingCircle(c1)
        p1=[x,y]
        p2=[x2,y2]
        pts=[p1,p2]
    else:
        pts=[[None,None],[None,None]]
        
    return pts


def detectGoal(image):
    coord = []
    coord = [0,0]
    nb_iterations=1
    
    img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img_hsv, red_lower, red_upper)
    mask = cv2.erode(mask, None, iterations = nb_iterations)
    mask = cv2.dilate(mask, None, iterations = nb_iterations)
    elements,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(elements) > 0:
        for i in range(0,len(elements)):
            c = elements[i] 
            ((x,y),rayon) = cv2.minEnclosingCircle(c)
            coord = [x,y]
    return coord

def angle_between(pts):
    #pts[0] is the big_circle and pts[1] is the little circle
    dist = [pts[1][0]-pts[0][0],pts[1][1]-pts[0][1]]
    #inverse distance in y because y axis is inversed in openCv 
    dist[1] = -dist[1]
    ang = np.arctan2(dist[1],dist[0])
    return ang

def directionThymio(image):
    coordThymio = detectThymio(image)
    #handling the case where thymio can't be detected and pass a None to Kalman
    if bool(not coordThymio[0][0]):
        print("thymio indetectable")
        return None
    else:
        direction = angle_between(coordThymio)
        return direction


#high level functions (celles qu'on appelle dans main)

#used only the first time
def find_corners(image):
    #img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_blur = cv2.GaussianBlur(image, (7, 7), 0)
    HSV = cv2.cvtColor(img_blur, cv2.COLOR_RGB2HSV)
    mask=cv2.inRange(HSV,green_lower,green_upper)
    mask = cv2.erode(mask,None, iterations=iterations_erode)
    mask = cv2.dilate(mask,None, iterations=iterations_erode)
    #trouve les contours des coins
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    #suppressing false corners
    initial_length=len(contours)
    for i in range(len(contours)):
        backwards_i=initial_length-i-1
        area = cv2.contourArea(contours[backwards_i])
        # Shortlisting the regions based on there area.
        if area < area_size:
            del contours[backwards_i]
    #finding corners center
    corner_points = []
    for i in range(len(contours)):
        if (cv2.contourArea(contours[i]) > area_size):
            mom = cv2.moments(contours[i])
            corner_points.append((int(mom['m10'] / mom['m00']), int(mom['m01'] / mom['m00']))) #centre des carrés
    if len(corner_points) != 4:
        print("failure in identifying corners")
        print("length corner ",len(corner_points))
    corner_points=order_points(corner_points)
    return corner_points


def img_calibration(image, corner_coord):
    #input img doit etre en rgb
    #img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_blur = cv2.GaussianBlur(image, (7, 7), 0)
    HSV = cv2.cvtColor(img_blur, cv2.COLOR_RGB2HSV)
    mask=cv2.inRange(HSV,green_lower,green_upper)
    mask = cv2.erode(mask,None, iterations=iterations_erode)
    mask = cv2.dilate(mask,None, iterations=iterations_erode)
    #trouve les contours des coins
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    #suppressing false corners
    initial_length=len(contours)
    for i in range(len(contours)):
        backwards_i=initial_length-i-1
        area = cv2.contourArea(contours[backwards_i])
        # Shortlisting the regions based on there area.
        if area < area_size:
            del contours[backwards_i]
    #finding corners center
    corner_points = []
    for i in range(len(contours)):
        if (cv2.contourArea(contours[i]) > area_size):
            mom = cv2.moments(contours[i])
            corner_points.append((int(mom['m10'] / mom['m00']), int(mom['m01'] / mom['m00']))) #centre des carrés
    if len(corner_points) != 4:
        print("failure in identifying corners")
        corner_points=corner_coord
    corner_points=order_points(corner_points)
    warpedimg=four_point_transform(image,corner_points)
    
    return warpedimg


def obstacle_detection(image):
    # Reading image
    #img2 = cv2.imread('obs.png', cv2.IMREAD_COLOR)
    
    # Reading same image in another variable and 
    # converting to gray scale.
    image = cv2.cvtColor( image , cv2.COLOR_RGB2GRAY)
    
    # Converting image to a binary image 
    # (black and white only image).
    _,threshold = cv2.threshold(image, thresh_low, thresh_up, cv2.THRESH_BINARY)
    
    # Detecting shapes in image by selecting region 
    # with same colors or intensity.
    contours,_=cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    list_polygon=[]
    polygon=[]
    # Searching through every region selected to 
    # find the required polygon.
    for cnt in contours :
        area = cv2.contourArea(cnt)
    
        # Shortlisting the regions based on there area.
        if area > area_obst_min: 
            if area < area_obst_max:
                approx = cv2.approxPolyDP(cnt, 
                                        0.009 * cv2.arcLength(cnt, True), True)
            
                # Checking if the no. of sides of the selected region is 7.
                if(len(approx) == 4): 
                    cv2.drawContours(image, [approx], 0, (0, 0, 255), 5)
                    for i in range (len(approx)):
                        point=(approx[i][0][0],approx[i][0][1])
                        polygon.append(point)
                    list_polygon.append(polygon)
                    polygon=[]      
    return list_polygon


def printGlobalPath(path, image):
    lineThickness = 3
    for i in range(0,len(path)-1):
        cv2.line(image, (round(path[i][0]), round(path[i][1])),(round(path[i+1][0]), round(path[i+1][1])), (0,255,0), lineThickness)

def initialization(pic):
    #obstacle detection
    polygons=obstacle_detection(pic)
    #Start and goal detection
    pos_start=detectThymio(pic)[0]
    angle_start=directionThymio(pic)
    #start=vis.detectCircle(pic,'start')
    goal=detectGoal(pic)
    init=[pos_start,goal,polygons,angle_start]
    return init

def printThymio(pic, posThym, coordThym):
    coordThym.append((round(posThym[0]),round(posThym[1]))) #pos_thym = [x,y]
    for i in range(0,len(coordThym)):
        cv2.circle(pic,(int(coordThym[i][0]),int(coordThym[i][1])), int(3), (0,0,255), 2)

def printGlobalPath(path, pic):
    lineThickness = 3
    for i in range(0,len(path)-1):
        cv2.line(pic, (round(path[i][0]), round(path[i][1])),(round(path[i+1][0]), round(path[i+1][1])), (0,255,0), lineThickness)
        
def takePicture(cap): 
    #Wait for the camera to focus by extracting 5 frames
    for i in range(5):
        # 2. Extract a frame from reading camera
        check, frame = cap.read()
        time.sleep(1)
    return frame