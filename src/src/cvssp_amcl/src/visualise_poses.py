
import pandas as pd
import tqdm
import argparse
import helper_functions as hf
import numpy as np

def poses_to_path_msg(poses):
    import rospy
    import tf2_ros
    import geometry_msgs.msg as gm
    import nav_msgs.msg as nm

    import tf.transformations

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("camera_rgb_optical_frame", 'camera_rgb_frame', rospy.Time())
            print(trans)
            print(type(trans))
            T = hf.transform_to_matrix(trans)
            print(T)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error getting TF, ignoring!")


    msg = nm.Path()
    msg.header.frame_id = "map"
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    for i, pose in enumerate(tqdm.tqdm(poses)):

        P = hf.pose_to_matrix(pose)
        P = np.matmul(P,T)
        pose = hf.matrix_to_pose(P)
        

        p = gm.PoseStamped()
        p.header.seq = i
        p.header.stamp = msg.header.stamp
        p.header.frame_id = "map"
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.w = pose[6]   
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]

        msg.poses.append(p)
    return msg

def publish_poses_as_path(poses, vis_topic="trajectory2", init=True, s=1000000):
    import rospy
    import nav_msgs.msg as nm

    if init:
        rospy.init_node('pose_visualiser', anonymous=True)
    pub = rospy.Publisher(vis_topic, nm.Path, queue_size=0)
    msg = poses_to_path_msg(poses)
    rate = rospy.Rate(1)  # 10hz
    i = 0
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        if i > s:
            rospy.signal_shutdown("Autoshutdown")
        i = i + 1

def main():

    #Parse Parameters
    parser = argparse.ArgumentParser(description='Interpolate Poses in GT File to stamps in rosbag. Requires Melodic.')
    parser.add_argument('--pose_file', default="/mnt/Storage/Data/raspect_data/TurtlebotTourCVSSP_0_poses.txt", help="CSV that contains poses with timestamps.")
    parser.add_argument('--rate', default=1, help="CSV that contains poses with timestamps.")
    args = parser.parse_args()

    data = pd.read_csv(args.pose_file, header=None, skiprows=3, delimiter=" ")
    data = data.sort_values(0)
    poses = data.values[:,1:]
    print(poses.shape)
    print("Min: {}".format(poses.min(0)))
    print("Max: {}".format(poses.max(0)))

    publish_poses_as_path(poses)


if __name__ == "__main__":
    main()
