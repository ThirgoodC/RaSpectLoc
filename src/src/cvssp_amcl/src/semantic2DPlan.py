#!/usr/bin/env python3

import argparse
import rospy
import time
from pathlib2 import Path
from tqdm import tqdm
import rosbag
from geometry_msgs.msg import PoseStamped

# ==================================================================================================
def main(args):
    rospy.init_node("Pose_Publisher", anonymous=True)
	# """Main script function."""
    num_lines = sum(1 for line in open(args.pose_file,'r'))

    publisher = rospy.Publisher("PoseGT",PoseStamped,queue_size=100)

    # print(num_lines)
    with open(args.pose_file, 'r') as f:
        while not rospy.is_shutdown():
            for i, line in enumerate(tqdm(f, total=num_lines)):
                values = line.strip().split(" ")
                msg = PoseStamped()
                msg.header.seq = i
                msg.header.stamp = rospy.Time.now() #Figure it out
                msg.header.frame_id = "map"
                msg.pose.position.x = float(values[1])
                msg.pose.position.y = float(values[2])
                msg.pose.position.z = float(values[3])
                msg.pose.orientation.x = float(values[4])
                msg.pose.orientation.y = float(values[5])
                msg.pose.orientation.z = float(values[6])
                msg.pose.orientation.w = float(values[7])

                publisher.publish(msg)
                print(msg)
                time.sleep(0.5)
    # rospy.spin()         
            
    #         j = 5
    # with open(args.pose_file,'r') as f:
    #     for line in tqdm(f, total=num_lines):
    




# ==================================================================================================
# def format_size(size):
# 	"""Format bytes with more human-readable units."""
# 	suffixes = ["B", "KB", "MB", "GB", "TB"]
# 	si = 0
# 	size = float(size)
# 	while size > 1024:
# 		size /= 1024
# 		si += 1

# 	return f"{size:.2f} {suffixes[si]}"


# ==================================================================================================
def parse_args():
	"""Parse arguments."""
	parser = argparse.ArgumentParser()
	parser.add_argument("pose_file", type=Path, help="Path to pose text file")

	return parser.parse_args()


# ==================================================================================================
if __name__ == "__main__":
	main(parse_args())
