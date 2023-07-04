#!/usr/bin/env python
import rospy
import tf

import numpy as np
import socket
import StringIO

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#server_address2 = ('131.227.165.206', 9996)
#server_address1 = ('131.227.165.96', 9997) #Pose Listener

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    robot = rospy.get_namespace()
    print robot
    if robot == "/butthead/":
        port = 9998

    if robot == "/stimpy/":
        port = 9996

    map = robot + "map"
    base = robot + "base_footprint"
    print("Listening from: {} to: {}".format(map, base))

    server_address1 = ('131.227.94.121', port)

    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(map, base, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    print("Tf Error")
            continue

        #Parse Position
        trans_s = StringIO.StringIO()
        np.savetxt(trans_s, trans, fmt="%.2f")
        trans_msg = trans_s.getvalue().replace('\n', ' ')

        # Parse Orientation
        rot_s = StringIO.StringIO()
        np.savetxt(rot_s, rot, fmt="%.2f")
        rot_msg = rot_s.getvalue().replace('\n', ' ')

        #Create Pose Message
        pose_msg = trans_msg + rot_msg
        print pose_msg

        sent1 = sock1.sendto(pose_msg.encode('utf-8'), server_address1)

        #Create Pose Message offset (debug)
        #trans[0] = trans[0] + 1.0
        #trans_s = StringIO.StringIO()
        #np.savetxt(trans_s, trans, fmt="%.2f")
        #trans_msg = trans_s.getvalue().replace('\n', ' ')
        #pose_msg = trans_msg + rot_msg
        #print pose_msg
        #sent1 = sock1.sendto(pose_msg.encode('utf-8'), server_address2)
        rate.sleep()
