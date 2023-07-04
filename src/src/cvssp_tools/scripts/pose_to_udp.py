# Imports
import rospy
import numpy as np
import geometry_msgs.msg

#For UDP
import socket
import StringIO


# Create a UDP socket
sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address1 = ('131.227.165.206', 9998)

class Pose:
    def __init__(self):
        rospy.init_node('pose_to_udp', anonymous=True)
        self.pose_sub = rospy.Subscriber("pose", geometry_msgs.msg.Pose, self.callback_pose)
    
    def callback_pose(self, data):
        """
        Function to handle the arrival of pose data
        """
        pose=np.asarray([data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        s = StringIO.StringIO()
        np.savetxt(s, pose, fmt="%.2f")

        pose_msg = s.getvalue()
        pose_msg = pose_msg.replace('\n', ' ')
        print(pose_msg)
        #cloud = np.asarray(cloud)
        #cloud_msg = np.array_str(cloud) 
        #print np.shape(cloud)
        #print cloud_msg
        #sent1 = sock1.sendto(cloud_msg.encode('utf-8'), server_address1)
        
p = Pose()
rospy.spin()
