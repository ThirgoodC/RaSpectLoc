# Imports
import roslib
import numpy as np
from sensor_msgs.msg import PointCloud2
import rospy
import tf
import geometry_msgs.msg

def FileCheck(fn):
    try:
      open(fn, "r")
      return 1
    except IOError:
      print "Error: File does not appear to exist."
      return 0

def get_msg(filename):
    file_open=False
    while not file_open:
        try:
            f = open(filename, 'r')
            file_open = True
        except IOError:
            file_open = False

    msg = f.read()

    f.close()
    return msg

def parse_msg(msg,curr_stamp):
    T = msg.split(",")

    S = T[0:1]
    P = T[1:len(T)-1]
    D = T[len(T)-1:len(T)]

    send, stamp = check_stamp(S, curr_stamp)
    goal = parse_pose(P)
    type = check_type(D)

    if send:
        print("Send: {}, Type: {}, Goal: {}".format(send,type,goal))

    return send, type, goal, stamp

def check_stamp(S,curr_stamp):
    nums = S[0].split("=")
    stamp = float(nums[1])

    send_goal = False
    if curr_stamp<stamp:
        send_goal = True
        curr_stamp=stamp
    return send_goal, curr_stamp

def parse_pose(P):
    goal = []
    for v in P:
        axis = v.split(" ")
        for x in axis:
            nums = x.split("=")
            goal.append(float(nums[1]))

    return goal

def check_type(D):

    #1 - Passive User Position and Gaze (previously Passive Gaze (1Hz))
    #2 - Active Target from Gaze (On Press)
    #3 - Robot Goal (On Press)
    #4 - Butthead Goal (On Press)
    #5 - Stimpy Goal (On Press)

    nums = D[0].split("=")
    type = int(nums[1])

    return type

def get_goal_msg(goal):
    goal_msg = geometry_msgs.msg.PoseStamped()
    goal_q = tf.transformations.quaternion_from_euler(0, 0, goal[4] * 3.14 / 180)
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.pose.position.x = goal[0]
    goal_msg.pose.position.y = goal[1]
    goal_msg.pose.position.z = goal[2]
    goal_msg.pose.orientation.x = goal_q[0]
    goal_msg.pose.orientation.y = goal_q[1]
    goal_msg.pose.orientation.z = goal_q[2]
    goal_msg.pose.orientation.w = goal_q[3]
    return goal_msg

if __name__ == '__main__':
    rospy.init_node('goal_from_udp')

    goal_pub1 = rospy.Publisher("/gaze/passive/move_base_simple/goal", geometry_msgs.msg.PoseStamped,queue_size=1)
    goal_pub2 = rospy.Publisher("/gaze/active/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=1)
    goal_pub3 = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=1)
    goal_pub4 = rospy.Publisher("/butthead/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=1)
    goal_pub5 = rospy.Publisher("/stimpy/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=1)

    filename = "/mnt/Ottoman/TurtleBot/Butthead/pos.txt"

    curr_stamp = 0

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        data = get_msg(filename)
        send, type, goal, stamp = parse_msg(data, curr_stamp)
        #send_goal = check_stamp(data,curr_stamp)
        #goal = parse_pose(msg)

        if send is True:
            curr_stamp = stamp

            if type is 1:
                print ("Sent 1: {}".format(goal))
                goal_msg = get_goal_msg(goal)
                goal_msg.header.frame_id = "map"
                #Parse Vector
                goal_pub1.publish(goal_msg)
            if type is 2:
                print ("Sent 2: {}".format(goal))
                goal_msg = get_goal_msg(goal)
                goal_msg.header.frame_id = "map"
                #Parse Gaze Vector
                goal_pub2.publish(goal_msg)
            if type is 3:
                print ("Sent 3: {}".format(goal))
                goal_msg = get_goal_msg(goal)
                goal_msg.header.frame_id = "map"
                goal_pub3.publish(goal_msg)
            if type is 4:
                print ("Sent 4: {}".format(goal))
                goal_msg = get_goal_msg(goal)
                goal_msg.header.frame_id = "/butthead/map"
                goal_pub4.publish(goal_msg)
            if type is 5:
                print ("Sent 5: {}".format(goal))
                goal_msg = get_goal_msg(goal)
                goal_msg.header.frame_id = "/stimpy/map"
                goal_pub5.publish(goal_msg)

        rate.sleep()
