#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import UInt8
from alice_msgs.msg import FoundObjectArray

state = None
team_index = None

penalty = False
other_penalty = False

time_stamp = time.time()
other_time_stamp = time.time()
penalty_sec = 0
other_penalty_sec = 0

destination = None

dropball_flags = 0

left_start = [3.1, 0.87, math.pi/2.0]
right_start = [12.9, 0.87, math.pi/2.0]

left_offense_start  = [3.1, 0.87, math.pi/2.0]
left_offense_start_opp  = [3.1, 10.13, -math.pi/2.0]
right_offense_start = [12.9, 0.87, math.pi/2.0]
right_offense_start_opp = [12.9, 10.13, -math.pi/2.0]

destination = [8.0, 5.5, 0]
# destination = [0.0, 0.0, 0]

goal_width = 0
goal_x_offset = 0

state_flag = False
count = 0

def state_callback(data):
    global state
    state = data.data

def teamindex_callback(data):
    global team_index
    team_index = data.data

def penalty_callback(data):
    global penalty, time_stamp, penalty_sec
    penalty = data.data
    if penalty == 1 and penalty_sec == 0:
        time_stamp = time.time()
        penalty_sec += 1
    elif penalty == 0:
        penalty_sec = 0

def other_penalty_callback(data):
    global other_penalty, other_time_stamp, other_penalty_sec
    other_penalty = data.data
    if other_penalty == 1 and other_penalty_sec == 0:
        time_stother_time_stampamp = time.time()
        other_penalty_sec += 1
    elif penalty == 0:
        other_penalty_sec = 0

def vision_callback(data):
    global goal_width, goal_x_offset
    goal_width = 0
    goal_x_offset = 0
    # vision = []
    for i in range(data.length):
        if data.data[i].name == "goal":
            # vision.append([data.data[i].roi.width, data.data[i].roi.x_offset])
            goal_width = data.data[i].roi.width
            goal_x_offset = data.data[i].roi.x_offset

def curr_pos_callback(data):
    global curr_x, curr_y, curr_theta
    curr_x = data.x
    curr_y = data.y
    curr_theta = data.theta

def dropball_Callback(data):
    global dropball_flags
    dropball_flags = data.data

if __name__ == "__main__":

    rospy.init_node('state_diterminer_player',anonymous=False)

    rospy.Subscriber('/game_controller/state',UInt8,state_callback)
    rospy.Subscriber('/game_controller/team_index',UInt8,teamindex_callback)
    rospy.Subscriber('/alice/player/penalty',UInt8,other_penalty_callback)
    rospy.Subscriber('/alice/keeper/penalty',UInt8,penalty_callback)
    rospy.Subscriber('/alice/vision/detected_objects', FoundObjectArray, vision_callback)
    rospy.Subscriber('/alice/robot_pos', Pose2D, curr_pos_callback)
    rospy.Subscriber('/alice/dropball', UInt8, dropball_Callback)

    state_determine_pub = rospy.Publisher('/alice/reposition',Pose2D,queue_size=10)

    ready_destination = None
    state_set_flag = False
    flag_reposition = False

    while not rospy.is_shutdown():
        state_determine_msg = Pose2D()
        if state == 0: #init
            state_flag = True
            if team_index == 0: #left
                state_determine_msg.x = left_start[0]
                state_determine_msg.y = left_start[1]
                state_determine_msg.theta = left_start[2]
                flag_reposition = True
            elif team_index == 1: #right
                state_determine_msg.x = right_start[0]
                state_determine_msg.y = right_start[1]
                state_determine_msg.theta = right_start[2]
                flag_reposition = True
            else: #exception
                print("state 0, team index error!")

        elif state == 1: #ready
            if state_flag == False:
                if count == 0:
                    if team_index == 0: #left
                        state_determine_msg.x = left_start[0]
                        state_determine_msg.y = left_start[1]
                        state_determine_msg.theta = left_start[2]
                        flag_reposition = True
                    elif team_index == 1: #right
                        state_determine_msg.x = right_start[0]
                        state_determine_msg.y = right_start[1]
                        state_determine_msg.theta = right_start[2]
                        flag_reposition = True
                    count += 1
                    flag_reposition = True

            if curr_y > 4.5 or curr_y< -4.5:
                state_set_flag = True
            elif curr_x > 7 or curr_x < -7:
                state_set_flag = True
            else:
                state_set_flag = False
                
            if penalty != 0:
                state_set_flag = True


        elif state == 2: #set
            if state_set_flag:
                if team_index == 0:
                    if goal_x_offset < 320 : 
                        state_determine_msg.x = left_offense_start[0]
                        state_determine_msg.y = left_offense_start[1]
                        state_determine_msg.theta = left_offense_start[2]
                        flag_reposition = True
                    elif goal_x_offset >= 320 :
                        state_determine_msg.x = left_offense_start_opp[0]
                        state_determine_msg.y = left_offense_start_opp[1]
                        state_determine_msg.theta = left_offense_start_opp[2]
                        flag_reposition = True

                    if goal_width < 110 or goal_width == None:
                        state_determine_msg.x -= 1.0  # if already one player got penalty, next penalized robot should move 1m beside to the reposition position
                        flag_reposition = True
                
                elif team_index == 1:
                    if goal_x_offset > 320 : 
                        state_determine_msg.x = right_offense_start[0]
                        state_determine_msg.y = right_offense_start[1]
                        state_determine_msg.theta = right_offense_start[2]
                        flag_reposition = True
                    elif goal_x_offset <= 320 :
                        state_determine_msg.x = right_offense_start_opp[0]
                        state_determine_msg.y = right_offense_start_opp[1]
                        state_determine_msg.theta = right_offense_start_opp[2]
                        flag_reposition = True

                    if goal_width < 110 or goal_width == None:
                        state_determine_msg.x += 1.0  # if already one player got penalty, next penalized robot should move 1m beside to the reposition position
                        flag_reposition = True
                else:
                    print("play penalty team_index error!")


        elif state == 3: #play
            if penalty != 0:
                if team_index == 0:
                    if goal_x_offset < 320 : 
                        state_determine_msg.x = left_offense_start[0]
                        state_determine_msg.y = left_offense_start[1]
                        state_determine_msg.theta = left_offense_start[2]
                        flag_reposition = True
                    elif goal_x_offset >= 320 :
                        state_determine_msg.x = left_offense_start_opp[0]
                        state_determine_msg.y = left_offense_start_opp[1]
                        state_determine_msg.theta = left_offense_start_opp[2]
                        flag_reposition = True

                    if goal_width < 110 or goal_width == None:
                        state_determine_msg.x -= 1.0  # if already one player got penalty, next penalized robot should move 1m beside to the reposition position
                        flag_reposition = True
                
                elif team_index == 1:
                    if goal_x_offset > 320 : 
                        state_determine_msg.x = right_offense_start[0]
                        state_determine_msg.y = right_offense_start[1]
                        state_determine_msg.theta = right_offense_start[2]
                        flag_reposition = True
                    elif goal_x_offset <= 320 :
                        state_determine_msg.x = right_offense_start_opp[0]
                        state_determine_msg.y = right_offense_start_opp[1]
                        state_determine_msg.theta = right_offense_start_opp[2]
                        flag_reposition = True

                    if goal_width < 110 or goal_width == None:
                        state_determine_msg.x += 1.0  # if already one player got penalty, next penalized robot should move 1m beside to the reposition position
                        flag_reposition = True
                else:
                    print("play penalty team_index error!")

            else:
                # print("no penalty")
                pass

            if dropball_flags != 0:
                if team_index == 0 and curr_x > 0:
                    if goal_x_offset < 320 : 
                        state_determine_msg.x = left_offense_start[0]
                        state_determine_msg.y = left_offense_start[1]
                        state_determine_msg.theta = left_offense_start[2]
                        flag_reposition = True
                    elif goal_x_offset >= 320 :
                        state_determine_msg.x = left_offense_start_opp[0]
                        state_determine_msg.y = left_offense_start_opp[1]
                        state_determine_msg.theta = left_offense_start_opp[2]
                        flag_reposition = True

                    if goal_width < 110 or goal_width == None:
                        state_determine_msg.x -= 1.0  # if already one player got penalty, next penalized robot should move 1m beside to the reposition position
                        flag_reposition = True
                
                elif team_index == 1 and curr_x < 0:
                    if goal_x_offset > 320 : 
                        state_determine_msg.x = right_offense_start[0]
                        state_determine_msg.y = right_offense_start[1]
                        state_determine_msg.theta = right_offense_start[2]
                        flag_reposition = True
                    elif goal_x_offset <= 320 :
                        state_determine_msg.x = right_offense_start_opp[0]
                        state_determine_msg.y = right_offense_start_opp[1]
                        state_determine_msg.theta = right_offense_start_opp[2]
                        flag_reposition = True

                    if goal_width < 110 or goal_width == None:
                        state_determine_msg.x += 1.0  # if already one player got penalty, next penalized robot should move 1m beside to the reposition position
                        flag_reposition = True

            else:
                # print("no dropball")
                pass

        else:
            flag_reposition = True

        if flag_reposition:
            # rospy.loginfo("[Localization Kinematics] : Repostion!")
            state_determine_pub.publish(state_determine_msg)
            flag_reposition = False
        rospy.sleep(0.01)
