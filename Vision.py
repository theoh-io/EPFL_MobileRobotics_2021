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
area_size=50 #used for corner detections
#obstacles
area_obst_min=500
area_obst_max=30000
thresh_low=90
thresh_up=255



#Defining color ranges
pink_lower=[]
pink_upper=[]
lower_red = [0,50,50]
upper_red = [10,255,255]
lower_cyan =[170,50,50]
upper_cyan =[180,255,255]
green_lower=np.array([45,40,30])
green_upper=np.array([90,255,255])



#low level functions (appelées a l'intérieur d'autres fonctions)
def plot_contours(img,contours,area_size, nb_sides):
    for cnt in contours :
        area = cv2.contourArea(cnt)
   
    # Shortlisting the regions based on there area.
    if area > area_size: 
        approx = cv2.approxPolyDP(cnt, 
                                  0.009 * cv2.arcLength(cnt, True), True)
        #print(approx)
   
        # Checking if the no. of sides of the selected region.
        if(len(approx) == nb_sides): 
            cv2.drawContours(img, [approx], -1, (255, 0, 0), 10)
    return img

def order_points(pts):
    """
    # initialzie a list of coordinates that will be ordered
	# such that the first entry in the list is the top-left,
	# the second entry is the top-right, the third is the
	# bottom-right, and the fourth is the bottom-left
	rect = np.zeros((4, 2), dtype = "float32")
	# the top-left point will have the smallest sum, whereas
	# the bottom-right point will have the largest sum
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
	# now, compute the difference between the points, the
	# top-right point will have the smallest difference,
	# whereas the bottom-left will have the largest difference
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]
	# return the ordered coordinates
	return rect
    """
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
	# return the warped image
    return warped



def color_mask(imgRGB, color):
    img_hsv = cv2.cvtColor(imgRGB, cv2.COLOR_RGB2HSV)

    if color == 'red':
        mask = cv2.inRange(img_hsv, lower_red, upper_red)
    if color == 'green':
        mask = cv2.inRange(img_hsv, lower_green, upper_green)
    if color == 'cyan':
        mask = cv2.inRange(img_hsv, lower_cyan, upper_cyan)
    if color == 'pink':
        mask = cv2.inRange(img_hsv, lower_cyan, upper_cyan)
        
    output_hsv = img_hsv.copy()
    output_hsv[np.where(mask==0)] = 0
    
    return output_hsv

def detectThymio(imgRGB):
    pts=[]
    #p1 is the big circle and p2 the little
    p1=[]
    p2=[]
    lower=np.array([10,30,30])
    upper=np.array([40,255,255])
    nb_iterations=1
    img_hsv = cv2.cvtColor(imgRGB, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img_hsv, lower, upper)
    img_hsv = cv2.blur(img_hsv,(7,7))
    mask = cv2.erode(mask, None, iterations = nb_iterations)
    mask = cv2.dilate(mask, None, iterations = nb_iterations)
    elements,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print(len(elements))
    if len(elements) > 0:
        #sorting the detected contours by descending area size
        elements.sort(key=cv2.contourArea, reverse=True)
        #finding big circle
        c=elements[0]
        ((x,y),rayon) = cv2.minEnclosingCircle(c)
        #finding little circle
        c2=elements[1]
        ((x2,y2),rayon2) = cv2.minEnclosingCircle(c2)
        p1=[x,y]
        p2=[x2,y2]
        pts=[p1,p2]
    return pts

def detectCircle(imgRGB,target):
    coord = []
    color_infos = (0,255,255)
    
    if target == 'goal':
        coord = [0,0]
        lower=np.array([170,50,50])
        upper=np.array([240,255,255])
        nb_iterations=1
    if target == 'start':
        coord = [0,0]
        lower=np.array([85,50,50])
        upper=np.array([105,255,255])
        nb_iterations=3
    
    img_hsv = cv2.cvtColor(imgRGB, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img_hsv, lower, upper)
    img_hsv = cv2.blur(img_hsv,(7,7))
    mask = cv2.erode(mask, None, iterations = nb_iterations)
    mask = cv2.dilate(mask, None, iterations = nb_iterations)
    #image2 = cv2.bitwise_and(imgRGB, imgRGB, mask=mask)
    elements,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(elements) > 0:
        for i in range(0,len(elements)):
            c = elements[i] 
            ((x,y),rayon) = cv2.minEnclosingCircle(c)
            coord = [x,y]
    return coord

def angle_between(pts):
    #pts[0]=big_circle and pts[1]= little circle
    dist = [pts[1][0]-pts[0][0],pts[1][1]-pts[0][1]]
    #inverse distance en y a cause de l'axe y inversé d'openCV
    dist[1] = -dist[1]
    ang = np.arctan2(dist[1],dist[0])
    return np.rad2deg(ang)

def directionThymio(imgRGB):
    coordThymio = detectThymio(imgRGB)
    direction = angle_between(coordThymio)
    return direction


#high level functions (celles qu'on appelle dans main)

#used only the first time
def find_corners(img):
    #img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_blur = cv2.GaussianBlur(img, (7, 7), 0)
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

def img_calibration(img, corner_coord):
    #input img doit etre en rgb
    #img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_blur = cv2.GaussianBlur(img, (7, 7), 0)
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
    warpedimg=four_point_transform(img,corner_points)
    
    return warpedimg



def obstacle_detection(img):
    # Reading image
    #img2 = cv2.imread('obs.png', cv2.IMREAD_COLOR)
    
    # Reading same image in another variable and 
    # converting to gray scale.
    img = cv2.cvtColor( img , cv2.COLOR_RGB2GRAY)
    
    # Converting image to a binary image 
    # (black and white only image).
    _,threshold = cv2.threshold(img, thresh_low, thresh_up, 
                                cv2.THRESH_BINARY)
    
    # Detecting shapes in image by selecting region 
    # with same colors or intensity.
    contours,_=cv2.findContours(threshold, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
    
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
                    cv2.drawContours(img, [approx], 0, (0, 0, 255), 5)
                    for i in range (len(approx)):
                        point=(approx[i][0][0],approx[i][0][1])
                        polygon.append(point)
                    list_polygon.append(polygon)
                    polygon=[]      
    return list_polygon

def printGlobalPath(path, img):
    lineThickness = 3
    for i in range(0,len(path)-1):
        cv2.line(img, (round(path[i][0]), round(path[i][1])),(round(path[i+1][0]), round(path[i+1][1])), (0,255,0), lineThickness)

def takePicture(nb): 
    # 1. Create an object. Zero for external camera
    #video = cv2.VideoCapture(0)
    # 3. Create a frame object
    cap = cv2.VideoCapture(nb, cv2.CAP_DSHOW)
    for i in range(5):
        check, frame = cap.read()
        time.sleep(1)
        print("frame {}".format(i))
    # 4. Show the frame!
    #cv2.imshow("Capturing picture",frame)

    #frame=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) => src empty in gaussian Blur
    # 2. Shutdown camera
    cap.release()
    return frame



def playVideo(): 
    # 1. Create an object. Zero for external camera
    video = cv2.VideoCapture(0)
    a = 0
    video_imgs = []
    while True:
        a = a + 1
        # 3. Create a frame object
        check, frame = video.read()
        
        # all frames
        video_imgs.append(frame) ## si on veut toutes les images de la video 
            
        # 4. Show the frame!
        cv2.imshow("Capturing Video",frame)

        # 7. for playing
        key = cv2.waitKey(1)
        if key == ord('q') :
            break
            
    # 2. Shutdown camera
    video.release()

    cv2.destroyAllWindows
    
    return video_imgs