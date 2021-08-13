import cv2
import numpy as np

size_map = [1600, 1100]
thickness = 5

landmarks = []
sideline_1 = ["line"] + [[100, 1000], [1500, 1000]]
sideline_2 = ["line"] + [[100, 100], [1500, 100]]
endline_1 = ["line"] + [[100, 100], [100, 1000]]
endline_2 = ["line"] + [[1500, 100], [1500, 1000]]
halfway_line = ["line"] + [[800, 100], [800, 1000]]
area_1_line_a = ["line"] + [[200, 300], [200, 800]]
area_2_line_a = ["line"] + [[100, 800], [200, 800]]
area_3_line_a = ["line"] + [[100, 300], [200, 300]]
area_1_line_b = ["line"] + [[1400, 300], [1400, 800]]
area_2_line_b = ["line"] + [[1400, 800], [1500, 800]]
area_3_line_b = ["line"] + [[1400, 300], [1500, 300]]
  
center_circle = ["circle"] + [[800, 550], [150]]
  
center_point = ["point"] + [[800, 550]]
penalty_a = ["point"] + [[310, 550]]
penalty_b = ["point"] + [[1290, 550]]

landmarks = [sideline_1] + [sideline_2] + [endline_1] + [endline_2] + [halfway_line] + [area_1_line_a] + [area_2_line_a] + [area_3_line_a] + [area_1_line_b] + [area_2_line_b] + [area_3_line_b] + [center_circle] + [center_point] + [penalty_a] + [penalty_b]

iamge_color_name = './soccer_field.jpg'
image_binary_name = './soccer_field_binary.jpg'

if __name__ == "__main__":
    blank_image = np.zeros((size_map[1],size_map[0],1), np.uint8)
    color_image = np.zeros((size_map[1],size_map[0],3), np.uint8)
    color_image[:,:,:] = (91,139,50)
    bimage = blank_image
    cimage = color_image
    for landmark in landmarks:
        if landmark[0] == 'line':
            cv2.line(bimage, tuple(landmark[1]), tuple(landmark[2]), (255), thickness)
            cv2.line(cimage, tuple(landmark[1]), tuple(landmark[2]), (255,255,255), thickness)
        elif landmark[0] == 'point':
            cv2.circle(bimage, tuple(landmark[1]), 5, (255), -1)
            cv2.circle(cimage, tuple(landmark[1]), thickness, (255,255,255), -1)
        elif landmark[0] == 'circle':
            cv2.circle(bimage, tuple(landmark[1]), landmark[2][0], (255), thickness)
            cv2.circle(cimage, tuple(landmark[1]), landmark[2][0], (255,255,255), thickness)
    
    cv2.imwrite(image_binary_name, bimage)
    cv2.imwrite(iamge_color_name, cimage)
    print('save ok')
