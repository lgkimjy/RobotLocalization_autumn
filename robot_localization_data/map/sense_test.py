import cv2
import numpy as np
from math import *
import sys

ep = sys.float_info.epsilon

horizontal_view = pi / 2.0
vertical_view = pi /3.0
interval_angle = pi/18.0

sense_max_distance = 2000.0

robot_height = 120
head_vertical = -pi/6.0
head_horizontal = pi/3.0

def rad(theta):
    return pi * theta / 180.0

def deg(theta):
    return 180.0 * theta / pi

def getPoint(start_point, end_point):
    x1, y1 = start_point
    x2, y2 = end_point
    points = []
    if abs(x2-x1) >= abs(y2-y1):
        for i in range(int(abs(x1-x2))):
            if y1 == y2:
                points.append([int(min(x1,x2)+i), int(y1)])
            else:
                x = min(x1,x2)+i
                y = round( ( (y2-y1)/(x2-x1) ) * (x-x1) + y1 )
                points.append([int(x),int(y)])
    else:
        for i in range(int(abs(y1-y2))):
            if x1 == x2:
                points.append([int(x1), int(min(y1,y2)+i)])
            else:
                y = min(y1,y2) + i
                x = round( ( (x2-x1)/(y2-y1) ) * (y-y1) + x1 )
                points.append([int(x),int(y)])
    return points

def getDistance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    d = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
    return d

def changeImagePoint(point, image_width, image_height):
    image_cordinate_point = [point[0], image_height - point[1]]
    return image_cordinate_point

def changeRealPoint(point, image_width, image_height):
    real_cordinate_point = [point[0], image_height - point[1]]
    return real_cordinate_point

def validPointCheck(point, image_width, image_height):
    if 0<= point[0] <= image_width-1 and 0<= point[1] <= image_height-1:
        return True
    else:
        return False

def getKeyPoint(robot_pos, image_width, image_height):
    global horizontal_view, vertical_view, interval_angle, sense_max_distance, head_vertical, head_horizontal
    Rx, Ry, Rw = robot_pos
   
    temp_l_theta = -head_vertical+vertical_view/2.0
    temp_L_theta = -head_vertical-vertical_view/2.0

    if temp_l_theta == 0.0:
        temp_l_theta = ep
    if temp_L_theta == 0.0:
        temp_L_theta = ep

    temp_l = robot_height/tan(temp_l_theta)
    temp_L = robot_height/tan(temp_L_theta)

    if temp_l <= 0:
        temp_l = sense_max_distance
    if temp_L <=0:
        temp_L = sense_max_distance

    l = min(temp_l, sense_max_distance)
    L = min(temp_L, sense_max_distance)

    interval_angles = []
    for i in range(int(horizontal_view/interval_angle)+1):
        interval_angles.append(Rw+head_horizontal-horizontal_view/2.0+interval_angle*i)
    
    key_points = []
    sense_lines = []

    for sense_theta in interval_angles:
        key_point_sense_line = []

        start_point = [l*cos(sense_theta)+Rx, l*sin(sense_theta)+Ry]
        end_point   = [L*cos(sense_theta)+Rx, L*sin(sense_theta)+Ry]

        start_point = changeImagePoint(start_point, image_width, image_height)
        end_point = changeImagePoint(end_point, image_width, image_height)

        sense_lines.append([start_point, end_point])
        points = getPoint(start_point, end_point)
        
        white_points = []
        for point in points:
            if validPointCheck(point, image_width, image_height):
                color = [image.item(point[1],point[0],0), image.item(point[1],point[0],1), image.item(point[1],point[0],2)]
                if color == [255,255,255]:
                    white_points.append(point)
  
        if len(white_points) != 0:
            group_white_points = []
            index_split = []
            for i in range(len(white_points)-1):
                if(getDistance(white_points[i], white_points[i+1]) >= 5):
                    index_split.append(i)

            if not 0 in index_split:
                index_split = [0] + index_split
            if not len(white_points)-1 in index_split:
                index_split = index_split + [len(white_points)-1]

            for i in range(len(index_split)-1):
                if index_split[i] == 0:
                    group_white_points.append(white_points[index_split[i]:index_split[i+1]+1])
                else:
                    group_white_points.append(white_points[index_split[i]+1:index_split[i+1]+1])
   
            clustering_white_points = []
            for group_white_point in group_white_points:
                sum_point = [0,0]
                for each_white_point in group_white_point:
                    sum_point[0] += each_white_point[0]
                    sum_point[1] += each_white_point[1]
                avg_point = [int(sum_point[0]/len(group_white_point)), int(sum_point[1]/len(group_white_point))]
                clustering_white_points.append(avg_point)

            for clustering_white_point in clustering_white_points:
                key_point_sense_line.append(clustering_white_point)
        key_points.append(key_point_sense_line)
            
    return key_points, sense_lines
   

if __name__ == "__main__":
    image = cv2.imread("./soccer_field_binary.jpg")
    image_height, image_width, _ = image.shape
    robot_pos = [image_width/2.0, image_height/2.0, pi/2.0]
    
    key_points, sense_lines = getKeyPoint(robot_pos, image_width, image_height)
 
    for sense_line in sense_lines:
        cv2.line(image, (int(sense_line[0][0]), int(sense_line[0][1])), (int(sense_line[1][0]), int(sense_line[1][1])), (0,0,255), 1)
   
    for key_points_eachline in key_points:
        for key_point in key_points_eachline:
            cv2.circle(image, tuple(key_point), 5, (255,0,0), -1)

    while True:
        cv2.imshow("image",image)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cv2.destroyAllWindows()

