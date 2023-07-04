import numpy as np

#import quaternion as qlib
from scipy.interpolate import interp1d
import tf.transformations as tft

import rospy
import geometry_msgs.msg as gmsgs

import tqdm

def inv(m):
    return np.linalg.inv(m)

def quat_to_eul(q):
    return tft.euler_from_quaternion(q)

def eul_to_quat(r,p,y):
    return tft.quaternion_from_euler(r,p,y)

def transform_to_pose(T):
    pose = np.zeros(7)
    pose[0] = T.transform.translation.x
    pose[1] = T.transform.translation.y
    pose[2] = T.transform.translation.z
    pose[3] = T.transform.rotation.x
    pose[4] = T.transform.rotation.y
    pose[5] = T.transform.rotation.z
    pose[6] = T.transform.rotation.w
    return pose

def pose_to_msg(pose, frame_id=""):
    msg = gmsgs.PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time(0)
    msg.header.seq = 0
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]

    return msg

def msg_to_pose(msg):
    pose = np.zeros(7)
    pose[0] = msg.pose.position.x
    pose[1] = msg.pose.position.y
    pose[2] = msg.pose.position.z
    pose[3] = msg.pose.orientation.x
    pose[4] = msg.pose.orientation.y
    pose[5] = msg.pose.orientation.z
    pose[6] = msg.pose.orientation.w
    return pose

def transform_to_matrix(T):
    pose  = transform_to_pose(T)
    return pose_to_matrix(pose)


def pose_to_matrix(p):
    t = np.array(p[0:3])
    q = np.array(p[3:])
    T = tft.identity_matrix()
    T[0:3, 3] = t
    T[0:3, 0:3] = tft.quaternion_matrix(q)[0:3, 0:3]
    return T


def matrix_to_pose(T):
    p = np.zeros(7)
    p[0:3] = T[0:3, 3]
    p[3:] = tft.quaternion_from_matrix(T)
    return p


def interp_axis(in_stamps, in_values, out_stamps):
    axis_interp = interp1d(in_stamps, in_values, kind='cubic', bounds_error=False, fill_value=(in_values[0], in_values[-1])) #"extrapolate")
    #axis_interp = interp1d(in_stamps, in_values, kind='cubic', bounds_error=True)
    return axis_interp(out_stamps)


# def interp_quat(in_stamps, in_values, out_stamps):
#     # Interpolate Q
#     q_in = np.array([qlib.quaternion(q[3], q[0], q[1], q[2]) for q in in_values])
#     q_new = qlib.squad(q_in, in_stamps, out_stamps)
#     for idx, s_out in enumerate(out_stamps):
#         if s_out < in_stamps[0]:
#             q_new[idx] = q_in[0]
#         elif s_out > in_stamps[-1]:
#             q_new[idx] = q_in[-1]

#     return np.array([[q.x, q.y, q.z, q.w] for q in q_new])

def pose_to_dict(in_stamp, in_pose):

    out_dict = {'stamp': in_stamp,
                'x': in_pose[:, 0], 'y': in_pose[:, 1], 'z': in_pose[:, 2],
                'qx': in_pose[:, 3], 'qy': in_pose[:, 4], 'qz': in_pose[:, 5], 'qw': in_pose[:, 6]}

    return out_dict

def dict_to_pose(in_dict):
    poses = np.array([in_dict['stamp'],in_dict['x'], in_dict['y'], in_dict['z'], in_dict['qx'], in_dict['qy'], in_dict['qz'], in_dict['qw']]).transpose()

    return poses

# def interpolate_pose(in_stamps, in_pose, out_stamp, T = None):
#     poses_raw_position = in_pose[:, 0:3]
#     poses_raw_rotation = in_pose[:, 3:7]

#     # Interpolate X
#     x_new = interp_axis(in_stamps, poses_raw_position[:, 0], out_stamp)

#     # Interpolate Y
#     y_new = interp_axis(in_stamps, poses_raw_position[:, 1], out_stamp)

#     # Interpolate Z
#     z_new = interp_axis(in_stamps, poses_raw_position[:, 2], out_stamp)

#     # Interpolate Q
#     q_new = interp_quat(in_stamps, poses_raw_rotation, out_stamp)

#     out_dict = {'stamp': out_stamp,
#                 'x': x_new, 'y': y_new, 'z': z_new,
#                 'qx': q_new[:, 0], 'qy': q_new[:, 1], 'qz': q_new[:, 2], 'qw': q_new[:, 3]}


#     return out_dict

def make_relative(in_dict):
    # Transform
    poses = np.array([in_dict['x'], in_dict['y'], in_dict['z'], in_dict['qx'], in_dict['qy'], in_dict['qz'], in_dict['qw']]).transpose()
    T = pose_to_matrix(poses[0])
    for i, pose in enumerate(poses):
        X = pose_to_matrix(pose)
        X = np.matmul(T, X)
        poses[i, :] = matrix_to_pose(X)

    out_dict = {'stamp': in_dict['stamp'],
                'x': poses[:,0], 'y': poses[:,1], 'z': poses[:,2],
                'qx': poses[:,3], 'qy': poses[:,4], 'qz': poses[:,5], 'qw': poses[:,6]}
    return out_dict


def transform_pose(in_dict, T = None):
    # Transform
    if T is not None:
        poses = np.array([in_dict['x'], in_dict['y'], in_dict['z'], in_dict['qx'], in_dict['qy'], in_dict['qz'], in_dict['qw']]).transpose()
        for i, pose in enumerate(poses):
            X = pose_to_matrix(pose)

            #XT
            Xp = np.matmul(X,T)
            #Xp = np.matmul(np.linalg.pinv(X), T)
            #Xp = np.matmul(X, np.linalg.pinv(T))
            #Xp = np.matmul(np.linalg.pinv(X), np.linalg.pinv(T))

            #TX
            #Xp = np.matmul(T, X)
            #Xp = np.matmul(np.linalg.pinv(T), X)
            #Xp = np.matmul(T, np.linalg.pinv(X))
            #Xp = np.matmul(np.linalg.pinv(T), np.linalg.pinv(X))

            #TXT
            #Xp = np.matmul(np.matmul(T, X), np.linalg.pinv(T))
            #Xp = np.matmul(np.matmul(np.linalg.pinv(T), X), T)

            #TX'T
            #Xp = np.matmul(np.matmul(T, np.linalg.pinv(X)), np.linalg.pinv(T))
            #Xp = np.matmul(np.matmul(np.linalg.pinv(T), np.linalg.pinv(X)), T)
            #Xp = np.linalg.pinv(np.matmul(np.matmul(T, np.linalg.pinv(X)), np.linalg.pinv(T)))
            #Xp = np.linalg.pinv(np.matmul(np.matmul(np.linalg.pinv(T), np.linalg.pinv(X)), T))

            #XTX
            #Xp = np.matmul(np.matmul(np.linalg.pinv(X), T), X)
            #Xp = np.matmul(np.matmul(X, T), np.linalg.pinv(X))

            #XT'X
            #Xp = np.matmul(np.matmul(np.linalg.pinv(X), np.linalg.pinv(T)), X)
            #Xp = np.matmul(np.matmul(X, np.linalg.pinv(T)), np.linalg.pinv(X))
            #Xp = np.matmul(np.matmul(np.linalg.pinv(X), np.linalg.pinv(T)), X)
            #Xp = np.matmul(np.matmul(X, np.linalg.pinv(T)), np.linalg.pinv(X))

            poses[i, :] = matrix_to_pose(Xp)

        out_dict = {'stamp': in_dict['stamp'],
                    'x': poses[:,0], 'y': poses[:,1], 'z': poses[:,2],
                    'qx': poses[:,3], 'qy': poses[:,4], 'qz': poses[:,5], 'qw': poses[:,6]}
        return out_dict

    else:
        return in_dict

def transform_pose_tf(in_dict, source_frame_id, target_frame_id, tfb=None):
    # Transform
    if tfb is not None:
        poses = np.array([in_dict['x'], in_dict['y'], in_dict['z'], in_dict['qx'], in_dict['qy'], in_dict['qz'], in_dict['qw']]).transpose()
        for i, pose in enumerate(poses):
            msg = pose_to_msg(pose, source_frame_id)
            msg = tfb.transform(msg,target_frame_id)
            poses[i, :]  = msg_to_pose(msg)

        out_dict = {'stamp': in_dict['stamp'],
                    'x': poses[:,0], 'y': poses[:,1], 'z': poses[:,2],
                    'qx': poses[:,3], 'qy': poses[:,4], 'qz': poses[:,5], 'qw': poses[:,6]}
        return out_dict

    else:
        return in_dict

def get_sensor_stamps(bagfiles, sensor_topic):
    import rosbag
    import tqdm
    stamps = []
    frame_id = None
    for i, bagfile in enumerate(bagfiles):
        bag = rosbag.Bag(bagfile)
        print("Extracting {} Stamps from bag {}/{}".format(bag.get_message_count(sensor_topic), i, len(bagfiles)))
        with tqdm.tqdm(total=bag.get_message_count()) as pbar:
            for topic, msg, t in bag.read_messages():
                pbar.update(1)
                if topic == sensor_topic:
                    stamps.append(msg.header.stamp.to_nsec())
                    if frame_id is None:
                        frame_id = msg.header.frame_id
                        print("Got Frame ID: {}".format(frame_id))
    return np.array([np.array(stamps), frame_id])

def downsample_poses(poses, linear_dist, angular_dist):
    il = 0
    idxs = []
    for i in tqdm.tqdm(range(1, poses.shape[0])):
        Tp = pose_to_matrix(poses[il])
        T = pose_to_matrix(poses[i])
        Td = np.matmul(np.linalg.inv(Tp), T)
        dx = np.linalg.norm(Td[0:3, 3])
        da = np.rad2deg(np.arccos((np.trace(Td[0:3, 0:3]) - 1) / 2))
        if dx > linear_dist or da > angular_dist:
            idxs.append(i)
            il = i
    return idxs