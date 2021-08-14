#!/usr/bin/env python2
import rospy
import math
from alice_ft_sensor_msgs.msg import ForceTorque
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose2D
from std_msgs.msg import UInt8

state = None
team_index = None
penalty = None
destination = None
left_foot_torque = None
right_foot_torque = None

left_start = [4.5, 0.87, math.pi/2.0]
right_start = [11.5, 0.87, math.pi/2.0]

left_offense_start  = [6.5, 5.5, 0]
right_offense_start = [9.5, 5.5, math.pi]
    
destination = [8.0, 5.5, 0]
# destination = [0, 0, 0]

def kickoffteam_callback(data):
    global kickoffteam
    kickoffteam = data.data

def state_callback(data):
    global state
    state = data.data

def teamindex_callback(data):
    global team_index
    team_index = data.data

def penalty_callback(data):
    global penalty
    penalty = data.data

def destination_callback(data):
    global destination
    destination = [data.x + 8.0, data.y + 5.5, data.z]

def torque_callback(data):
    global left_foot_torque, right_foot_torque
    left_foot_torque = data.force_z_raw_l
    right_foot_torque = data.force_z_raw_r

if __name__ == "__main__":
    rospy.init_node('state_diterminer',anonymous=False)
    rospy.Subscriber('/game_controller/state',UInt8,state_callback)
    rospy.Subscriber('/game_controller/team_index',UInt8,teamindex_callback)
    rospy.Subscriber('/alice/penalty',UInt8,penalty_callback)
    rospy.Subscriber('/alice/global_destination',Vector3,destination_callback)
    rospy.Subscriber('/alice/force_torque_data',ForceTorque,torque_callback)
    rospy.Subscriber('/alice/kickoffteam', UInt8, kickoffteam_callback)

    state_determine_pub = rospy.Publisher('/alice/reposition',Pose2D,queue_size=10)

    ready_destination = None
    state_set_flag = False
    flag_reposition = False

    while not rospy.is_shutdown():
        state_determine_msg = Pose2D()
        if state == 0: #init
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
            # ready_destination = destination[:]
            if team_index == 0: #left
                ready_destination = left_offense_start
            elif team_index == 1: #right
                ready_destination = right_offense_start

        elif state == 2: #set
            print("ft_seneor : ",left_foot_torque, right_foot_torque)
            # if left_foot_torque >= -20 and right_foot_torque >= -20:
            # state_set_flag = True
            if kickoffteam == 1:
                state_set_flag = 1
            if state_set_flag:    # which mean people will replace robots postion by manually to the field cetenr
                state_determine_msg.x = ready_destination[0]
                state_determine_msg.y = ready_destination[1]
                state_determine_msg.theta = ready_destination[2]
                flag_reposition = True

        elif state == 3: #play
            state_set_flag = False
            if penalty != 0:
                if team_index == 0:
                    state_determine_msg.x = left_start[0]
                    state_determine_msg.y = left_start[1]
                    state_determine_msg.theta = left_start[2]
                    flag_reposition = True
                elif team_index == 1:
                    state_determine_msg.x = right_start[0]
                    state_determine_msg.y = right_start[1]
                    state_determine_msg.theta = right_start[2]
                    flag_reposition = True
                else:
                    print("play penalty team_index error!")
            else:
                print("no penalty")
                pass

        else:
            flag_reposition = True

        if flag_reposition:
            state_determine_pub.publish(state_determine_msg)
            flag_reposition = False
        # rospy.sleep(0.01)
        rospy.sleep(0.0001)
