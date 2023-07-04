#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan 
from cvssp_tools.msg import SemanticScan 

class remapper:
    def __init__(self):

        self.subScan = rospy.Subscriber(
            "/raman/semantic/scan", 
            SemanticScan, 
            self.ScanCallback, 
            queue_size=1
        )

        self.pubScan = rospy.Publisher(
            "/scan2", 
            SemanticScan,
            # LaserScan, 
            queue_size=1
        )   

        # self.rotate_pub.publish(position)  

    def ScanCallback(self, data):
        # msg = SemanticScan()
        # # msg = LaserScan()
        # msg.header = data.header
        # msg.angle_min = data.angle_min
        # msg.angle_max = data.angle_max
        # msg.angle_increment = data.angle_increment
        # msg.time_increment = data.time_increment
        # msg.scan_time = data.scan_time
        # msg.range_min = data.range_min
        # msg.range_max = data.range_max
        # msg.ranges = data.ranges
        # msg.labels = [2] * len(data.ranges)
        # msg.angles = [0] * len(data.ranges)
        # msg.header.frame_id = "camera_depth_frame"
        # print("msg republished")

        print(data)

        # j = 0
        # for i in range(len(data.ranges)):
        #     if j > 6:
        #         j = 0

        #     msg.labels[i] = j

        #     j = j +1

        # self.pubScan.publish(msg)


if __name__ == "__main__":
    
    rospy.init_node("remapper", anonymous=True)

    remap = remapper()
    
    rospy.spin()


            # msg.header = data.header
        # msg.angle_min = data.angle_min
        # msg.angle_max = data.angle_max
        # msg.angle_increment = data.angle_increment
        # msg.time_increment = data.time_increment
        # msg.scan_time = data.scan_time
        # msg.range_min = data.range_min
        # msg.range_max = data.range_max
        # msg.ranges = data.ranges