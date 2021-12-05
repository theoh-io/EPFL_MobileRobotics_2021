import cv2 as cv
import numpy as np
from PIL import Image 
import math
import matplotlib
from matplotlib.pyplot import imshow
from matplotlib import pyplot as plt

#Defining color ranges
pink_lower=[]
pink_upper=[]
lower_red = [0,50,50]
upper_red = [10,255,255]
green_lower=[85,50,50]
green_upper=[135,255,255]
lower_cyan =[170,50,50]
upper_cyan =[180,255,255]


#low level functions (appelées a l'intérieur d'autres fonctions)
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
    #vérification du sorting et intervertit si il y a eu une erreur
    #top left x > top right x => erreur et intervertit
    if pts[0][0] > pts[1][0]:
        pts[0], pts[1] = pts[1], pts[0]
    if pts[2][0] > pts[3][0]:
        pts[2], pts[3] = pts[3], pts[2]
    return pts


def four_point_transform(image, ordered_pts):
	# obtain a consistent order of the points and unpack them
	# individually
	(tl, tr, br, bl) = ordered_pts
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
	dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")
	# compute the perspective transform matrix and then apply it
	M = cv2.getPerspectiveTransform(ordered_pts, dst)
	warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
	# return the warped image
	return warped



#high level functions (celles qu'on appelle dans main)
def img_calibration(img):
    #input img doit etre en rgb
    HSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    #applique flou gaussien pour que les contours soit moins durs
    HSV_blur = cv2.GaussianBlur(HSV, (7, 7), 0)
    #a voir si on utilise une autre couleur
    green_mask=cv2.inRange(HSV_blur,green_lower,green_upper)
    #green_mask[70:350,:]=0 #remove the yellow which is similar to green corners
    #green_mask[:,100:500]=0 #remove the yellow which is similar to green corners
    
    # Identifier les coins et leurs coordonnées
    #trouve les contours des coins
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    corner_points = []
    for i in range(len(contours)):
        if (cv2.contourArea(contours[i]) > 200):
            mom = cv2.moments(contours[i])
            corner_points.append((int(mom['m10'] / mom['m00']), int(mom['m01'] / mom['m00'])))
    if len(corner_points) != 4:
        print("failure in identifying corners")
        print(corner_points)
    corner_points=order_points(corner_points)
    warpedimg=four_point_transform(img,corner_points)
    return warpedimg

def obstacle_detection():
    # Reading image
    img2 = cv2.imread('obs.png', cv2.IMREAD_COLOR)
    
    # Reading same image in another variable and 
    # converting to gray scale.
    img = cv2.imread('obs.png', cv2.IMREAD_GRAYSCALE)
    
    # Converting image to a binary image 
    # (black and white only image).
    _,threshold = cv2.threshold(img, 110, 255, 
                                cv2.THRESH_BINARY)
    
    # Detecting shapes in image by selecting region 
    # with same colors or intensity.
    contours,_=cv2.findContours(threshold, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
    
    # Searching through every region selected to 
    # find the required polygon.
    for cnt in contours :
        area = cv2.contourArea(cnt)
    
        # Shortlisting the regions based on there area.
        if area > 400: 
            polygons = cv2.approxPolyDP(cnt, 
                                    0.009 * cv2.arcLength(cnt, True), True)
        
    return polygons