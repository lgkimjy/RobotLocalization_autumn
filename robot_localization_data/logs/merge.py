#!/usr/bin/env python3
import cv2
import numpy as np
import sys

trajectory_img = cv2.imread("./scenario2/gt.png", -1)
feild_img = cv2.imread("./scenario2/result.png", -1)
h, w, depth = trajectory_img.shape
result = np.zeros((h, w, 3), np.uint8)

for i in range(h):
    for j in range(w):
        color1 = feild_img[i, j]
        color2 = trajectory_img[i, j]
        alpha = color2[3] / 255.0
        new_color = [ (1 - alpha) * color1[0] + alpha * color2[0],
                      (1 - alpha) * color1[1] + alpha * color2[1],
                      (1 - alpha) * color1[2] + alpha * color2[2] ]
        result[i, j] = new_color


cv2.imwrite('./scenario2/result_with_gt.JPG', result)