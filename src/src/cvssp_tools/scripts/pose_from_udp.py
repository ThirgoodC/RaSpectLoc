# Imports
import roslib
import numpy as np
from sensor_msgs.msg import PointCloud2
import rospy
import tf
import geometry_msgs.msg

#For UDP
import socket
import StringIO

# Create a UDP socket
sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#server_address1 = ('131.227.165.206', 9997)
server_address1 = ('131.227.165.96', 9997)
sock1.bind(server_address1)



if __name__ == '__main__':
    rospy.init_node('goal_from_udp')

    goal_pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped,queue_size=1)
    goal_msg = geometry_msgs.msg.PoseStamped()
    goal_msg.header.frame_id = "map"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print("Waiting...")
        data, server = sock1.recvfrom(4096)

        print(data)

        T = data.split(",")
        print data

        goal = []
        for v in T:
            axis = v.split(" ")
            for x in axis:
                nums = x.split("=")
                goal.append(float(nums[1]))

        print goal

        goal_q = tf.transformations.quaternion_from_euler(0, 0, goal[4]*3.14/180)

        #Pose Parsing
        #goal = np.fromstring(data, dtype=float, sep=' ')

        goal_msg.header.stamp =  rospy.Time.now()
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.z = goal[2]
        goal_msg.pose.orientation.x = goal_q[0]
        goal_msg.pose.orientation.y = goal_q[1]
        goal_msg.pose.orientation.z = goal_q[2]
        goal_msg.pose.orientation.w = goal_q[3]

        #goal_pub.publish(goal_msg)

        rate.sleep()
