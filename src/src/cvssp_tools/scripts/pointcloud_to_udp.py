#!/usr/bin/env python
# Imports
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import sys
import copy
import rospy
import tf
import geometry_msgs.msg
import time
import sensor_msgs.point_cloud2 as pc2

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


#For UDP
import socket
import os
from os import listdir
import io

import StringIO

# Create a UDP socket
sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address1 = ('131.227.165.206', 9999)
server_address2 = ('131.227.165.206', 9995) #debug
#server_address1 = ('131.227.165.96', 9997)


class Points:
    def __init__(self):
        rospy.init_node('point_to_udp', anonymous=True)

        robot = rospy.get_namespace()
        if robot == "/butthead/":
            self.server_address = ('131.227.165.206', 9999)

        if robot == "/stimpy/":
            self.server_address = ('131.227.165.206', 9995)

        self.map = robot[1:len(robot)] + "map"

        print("Listening on {}".format(robot+"joints"))

        self.points_sub = rospy.Subscriber(robot+"joints", PointCloud2, self.callback_points)
        #self.t = tf.TransformerROS(True, rospy.Duration(10.0))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_cloud_msg(self, cloud):
        cloud_msg = np.array_str(cloud)
        s = StringIO.StringIO()
        np.savetxt(s, cloud, fmt="%.2f")
        cloud_msg = s.getvalue()
        cloud_msg = cloud_msg.replace('\n',' ')
        cloud_msg = str(np.shape(cloud)[0]) + " { " + cloud_msg + "}"
        print ("Points:" ,np.shape(cloud))
        print cloud_msg
        return cloud_msg

    def callback_points(self, data):
        """
        Function to handle the arrival of pointcloud data
        """
        try:
            transform = self.tf_buffer.lookup_transform(self.map, data.header.frame_id, rospy.Time())
            cloud = do_transform_cloud(data, transform)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("TF Error")
            cloud = data

        cloud = list(pc2.read_points(cloud, skip_nans=False, field_names=("x", "y", "z")))
        cloud = np.asarray(cloud)
        cloud_msg = self.get_cloud_msg(cloud)
        sent1 = sock1.sendto(cloud_msg.encode('utf-8'), self.server_address)
        
        #Offset for debug
        #cloud[:,0] = cloud[:,0] + 1.0
        #cloud_msg = self.get_cloud_msg(cloud)
        #sent2 = sock1.sendto(cloud_msg.encode('utf-8'), server_address2)

point_cloud = Points()
rospy.spin()
