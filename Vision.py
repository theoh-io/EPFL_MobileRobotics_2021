import cv2 as cv
import numpy as np
from PIL import Image 
import math
import matplotlib
from matplotlib.pyplot import imshow
from matplotlib import pyplot as plt

filename = 'obs.png'
img = cv.imread(filename) #HSV!!!, LUV
plt.imshow(img)

red = np.uint8([[[255,0,0 ]]])
hsv_red = cv.cvtColor(red,cv.COLOR_BGR2HSV)
print(hsv_red)

img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
# lower mask (0-10)
lower_red = np.array([0,50,50])
upper_red = np.array([10,255,255])
mask0 = cv.inRange(img_hsv, lower_red, upper_red)

# upper mask (170-180)
lower_red = np.array([170,50,50])
upper_red = np.array([180,255,255])
mask1 = cv.inRange(img_hsv, lower_red, upper_red)

# join my masks
mask = mask0+mask1

# set my output img to zero everywhere except my mask
output_img = img.copy()
output_img[np.where(mask==0)] = 0

# or your HSV image, which I *believe* is what you want
output_hsv = img_hsv.copy()
output_hsv[np.where(mask==0)] = 0

plt.imshow(output_hsv)
cv.imwrite('image_rouge.png', output_hsv)
cv.waitKey(0)
cv.destroyAllWindows()

maskg = cv.inRange(img_hsv, (36, 25, 25), (70, 255,255))

output_hsvg = img_hsv.copy()
output_hsvg[np.where(maskg==0)] = 0

cv.imwrite("green.png", output_hsvg)

plt.imshow(output_hsvg)