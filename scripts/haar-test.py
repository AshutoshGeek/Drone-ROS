#!/usr/bin/env python3
import cv2
from matplotlib import pyplot as plt


logo_cascade = cv2.CascadeClassifier('data/cascade.xml')

img = cv2.imread('data/test_3.png')  # Source image
cv2.imshow("image",img)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)

for (x, y, w, h) in logo:
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)

centerx =  x + w/2
centery =  y + h/2
center_coordinates = (int(centerx), int(centery))
radius = 2
# Blue color in BGR
color = (255, 0, 0)
  
# Line thickness of 2 px
thickness = 2
  
# Using cv2.circle() method
# Draw a circle with blue line borders of thickness of 2 px
image = cv2.circle(img, center_coordinates, radius, color, thickness)
cv2.imshow("window_name", image)	
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.show()

