import cv2 as cv
import numpy as np
from PIL import Image 
import math
import matplotlib
from matplotlib.pyplot import imshow
from matplotlib import pyplot as plt


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