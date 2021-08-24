import cv2
import numpy as np

size_map = [1040, 740]
thickness = 5

iamge_color_name = './white.jpg'

if __name__ == "__main__":
    color_image = np.zeros((size_map[1],size_map[0],3), np.uint8)
    color_image[:,:,:] = (255,255,255)
    cimage = color_image
    
    cv2.imwrite(iamge_color_name, cimage)
    print('save ok')
