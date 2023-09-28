#!/usr/bin/env python3

import rospy
import math

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan 
from logical_camera.msg import MaterialAndDepth

class log_cam_listener:
    def __init__(self):

        self.image_sub = rospy.Subscriber(
                "/gazebo/logical_camera", ModelStates, self.LC_callback, queue_size=1
        )
        self.ray_sub = rospy.Subscriber(
                "/rot_laser_scan", LaserScan, self.Ray_callback, queue_size=1
        )

        self.filter_pub = rospy.Publisher(
            "filter_log_cam_topic", MaterialAndDepth, queue_size=1
            )

    def Ray_callback(self, data):
        self.laserScan = data
        self.range = data.ranges[0]
        

    def LC_callback(self, data):
        # msg = ModelState()
        msg = MaterialAndDepth()
        smallestDst = 100.0
        smallestIt = 0
        floorIt = 0
        dists = [0] * len(data.name)
        min_val = 100.0
        min_index = 0

        #Get the models in the list and find the relative distance away
        for i in range(len(data.name)):
            if data.name[i] != "ground_plane":
                edist = math.sqrt(math.pow((data.pose[i].position.x), 2) + math.pow((data.pose[i].position.y), 2))# + math.pow((data.pose[i].position.z), 2))
                dists[i] = edist
            else:
                dists[i] = 100.0


       #If more than one value in the list
        if len(dists) > 1:
            # min_val = min(dists)
            # min_index = dists.index(min_val)
      
            print(self.range)

            print(min(dists, key=lambda x:abs(x-self.range)))

            self.closest_val = min(dists, key=lambda x:abs(x-self.range))
            min_index = dists.index(self.closest_val)

            print(dists)
        
        #Distance from camera - Should this be published if range away
        if data.name[min_index] != "ground_plane":
            if self.closest_val < 20.0 and (self.closest_val + 0.1 >= self.range or self.closest_val - 0.1 <= self.range):
                msg.material = data.name[min_index]
                print(data.name[min_index])
                msg.depth = self.range

        else:
            msg.material = "/"
            msg.depth = 0.0
            
        self.filter_pub.publish(msg)
            
                        
if __name__ == "__main__":
    
    rospy.init_node("logical_camera_listener", anonymous=True)
    log_cam_obj = log_cam_listener()
    
    rospy.spin()
