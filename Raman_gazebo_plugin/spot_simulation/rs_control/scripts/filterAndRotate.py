#! /usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan 

from logical_camera.msg import SemanticLCScan

import math

# rospy.init_node ("rotate_laser")

# pub = rospy.Publisher ("/spot1/base_to_laser_joint_position_controller/command", Float64, queue_size=1)


position = 0.0

delta = 1.0
increment = 1.0

radian = math.pi / 180 

# r = rospy.Rate(0.5)

# while not rospy.is_shutdown():
#     pub.publish (position)
#     position = position * (delta * radian)
#     r.sleep()

class cam_rotate:
    def __init__(self):

        self.start_pos = 0.0
        self.position = 0.0

        self.rotations = 0
        self.LCIt = 0
        self.scanIt = 0
        self.rotationIt = 0

        self.init_rotate = False
        self.Published = False
        self.setZero = False

        # self.currMsg = SemanticLCScan()
        self.ranges = [0] * 360
        self.labels = [0] * 360
        self.angles = [0] * 360

        self.print_final = True

        self.LC_sub = rospy.Subscriber(
                "/gazebo/logical_camera", ModelStates, self.LC_callback, queue_size=1
        )
        self.ray_sub = rospy.Subscriber(
                "/rot_laser_scan", LaserScan, self.Ray_callback, queue_size=1
        )
 
        self.init_rotate = rospy.Subscriber(
            "init_rotate", 
            Bool, 
            self.init_rotate_callback, 
            queue_size=1
        )

        self.rotate_pub = rospy.Publisher(
            "/spot1/base_to_laser_joint_position_controller/command", 
            Float64, 
            queue_size=1)   
        
        self.semantic_pub = rospy.Publisher(
            "/amcl_semantic_scan", 
            SemanticLCScan, 
            queue_size=1
        )   

        # self.rotate_pub.publish(position)  

    def Ray_callback(self, data):
        if self.rotations < 360 and self.init_rotate == True:
            if self.scanIt == 0:
                self.currMsg.header = data.header
                self.currMsg.angle_min = data.angle_min
                self.currMsg.angle_max = data.angle_increment
                self.currMsg.time_increment = 0.001
                self.currMsg.scan_time = 500/360
                self.currMsg.range_min = data.range_min
                self.currMsg.range_max = data.range_max

            self.currentLaserScan = data
            # self.ranges[self.scanIt] = data.ranges[0]
            self.range = data.ranges[0]
            self.scanIt = self.scanIt + 1

    def LC_callback(self, data):

        if (self.rotationIt < 360) and (self.init_rotate == True):

            dists = [0] * len(data.name)
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
                # print(self.range)
                print(min(dists, key=lambda x:abs(x-self.range)))
                print(dists)
                print(self.range)

                self.closest_val = min(dists, key=lambda x:abs(x-self.range))
                min_index = dists.index(self.closest_val)   
        
            
            #Distance from camera to be stored and 
            if data.name[min_index] != "ground_plane":
                if self.closest_val < 20.0 and (self.closest_val + 0.1 >= self.range or self.closest_val - 0.1 <= self.range):
                    self.labels[self.LCIt] = data.name[min_index]
                    print(self.labels[self.LCIt])
                    self.ranges[self.LCIt] = self.range

            else:
                self.labels[self.LCIt] = "/"
                self.ranges[self.LCIt] = 0.0
            
            self.LCIt = self.LCIt + 1

            self.rotate_scan()
        
        else:
            self.publishMessage()
            self.init_rotate = False

    def rotate_scan(self):
        rospy.sleep(0.001)
        self.position = self.position + (delta * radian)
        self.rotate_pub.publish(self.position)
        self.setZero = True
        self.angles[self.rotationIt] = self.position #???? is this correct
        self.rotations = self.rotations + increment
        self.rotationIt = self.rotationIt + 1

    def init_rotate_callback(self, data):
        self.start = rospy.get_rostime()
        # self.init_rotate = data.data
        # print(self.init_rotate)

        if data.data == True:
            print("Starting sequence")
            # self.Published = False
            self.print_final = True
            self.rotations = 0
            self.position = 0.0
            self.scanIt = 0
            self.LCIt = 0
            self.rotationIt = 0
            self.currMsg = SemanticLCScan()
            # self.rotate_scan()
            self.rotate_pub.publish(self.position)
            rospy.sleep(1.0)
            self.init_rotate = True


    def LCD_callback(self, data):
        if self.rotations < 360 and self.init_rotate == True:
            print(data)
            
            while (self.LCIt != self.rotationIt):
                print("Waiting for LC msg")

    def publishMessage(self):
        if (self.Published == False) and (self.init_rotate == True):
            self.Fin = rospy.get_rostime()
            self.rotate_pub.publish(0.0)
            print(f"diff time = {self.Fin.to_sec() - self.start.to_sec()}")
            self.print_final = False  
            self.setZero = False

            self.currMsg.ranges = self.ranges
            self.currMsg.angles = self.angles
            self.currMsg.labels = self.labels

            print(len(self.angles))
            print(len(self.ranges))
            print(len(self.labels))

            print(self.currMsg)

            self.Published == True

            self.semantic_pub.publish(self.currMsg)         


if __name__ == "__main__":
    
    rospy.init_node("rot_node", anonymous=False)

    rotate_cam_obj = cam_rotate()
    
    rospy.spin()