#! /usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan 
from logical_camera.msg import MaterialAndDepth

from gazebo_msgs.msg import ModelState
from logical_camera.msg import MaterialAndDepth
from logical_camera.msg import SemanticLCScan

import math

# rospy.init_node ("rotate_laser")

# pub = rospy.Publisher ("/spot1/base_to_laser_joint_position_controller/command", Float64, queue_size=1)


position = 0.0

delta = 1.0

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

        self.init_rotate = False

        self.items = []

        self.print_final = True

        self.image_sub = rospy.Subscriber(
            "filter_log_cam_topic", 
            MaterialAndDepth, 
            self.LCD_callback, 
            queue_size=1
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

        # self.rotate_pub.publish(position)  

    def rotate_scan(self):
        self.position = self.position + (delta * radian)
        self.rotate_pub.publish (self.position)
        self.rotations = self.rotations + 1
        
        # self.rotate_pub.publish(position)

    def init_rotate_callback(self, data):
        self.start = rospy.get_rostime()
        self.init_rotate = data.data
        print(self.init_rotate)

        if data.data == True:
            print("Starting sequence")
            self.print_final = True
            self.rotations = 0
            self.position = 0.0
            self.rotate_pub.publish(self.position)


    def LCD_callback(self, data):
        if self.rotations <= 360 and self.init_rotate == True:
            print(data)
            
            self.items.append(tuple([data.material, data.depth]))
            
            self.rotate_scan()

            # rospy.sleep(0.0001)
        
        else:
            if self.print_final == True:
                self.Fin = rospy.get_rostime()
                self.rotate_pub.publish(0.0)
                print(f"diff time = {self.Fin.to_sec() - self.start.to_sec()}")
                self.print_final = False  
                print(self.items)
            
                        
if __name__ == "__main__":
    
    rospy.init_node("rot_node", anonymous=False)

    rotate_cam_obj = cam_rotate()
    
    rospy.spin()