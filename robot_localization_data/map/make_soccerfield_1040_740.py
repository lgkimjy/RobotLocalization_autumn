import cv2
import numpy as np

size_map = [1040, 740]
thickness = 5

landmarks = []
sideline_1 = ["line"] + [[70, 670], [970, 670]]
sideline_2 = ["line"] + [[70, 70], [970, 70]]
endline_1 = ["line"] + [[70, 70], [70, 670]]
endline_2 = ["line"] + [[970, 70], [970, 670]]
halfway_line = ["line"] + [[520, 70], [520, 670]]
area_1_line_a = ["line"] + [[170, 120], [170, 620]]
area_2_line_a = ["line"] + [[70, 620], [170, 620]]
area_3_line_a = ["line"] + [[70, 120], [170, 120]]
area_1_line_b = ["line"] + [[870, 120], [870, 620]]
area_2_line_b = ["line"] + [[870, 620], [970, 620]]
area_3_line_b = ["line"] + [[870, 120], [970, 120]]
  
center_circle = ["circle"] + [[520, 370], [75]]
  
center_point = ["point"] + [[520, 370]]
penalty_a = ["point"] + [[220, 370]]
penalty_b = ["point"] + [[820, 370]]

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
