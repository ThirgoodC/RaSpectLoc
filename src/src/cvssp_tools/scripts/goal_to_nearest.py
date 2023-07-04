#!/usr/bin/env python
import math

import rospy
import tf
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan

bots = ["butthead","stimpy"]

class GoalListener:
    def __init__(self):
        rospy.init_node('goal_to_nearest', anonymous=True)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_goal)
        # self.t = tf.TransformerROS(True, rospy.Duration(10.0))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_pubs = [];
        for bot in bots:
            self.goal_pubs.append( rospy.Publisher(bot+"/move_base_simple/goal", PoseStamped, queue_size=10))

        #self.butthead_pose_est_sub = rospy.Subscriber("/butthead/initialpose", PoseWithCovarianceStamped, self.callback_pose_butthead)
        #self.stimpy_pose_est_sub = rospy.Subscriber("/stimpy/initialpose", PoseWithCovarianceStamped, self.callback_pose_stimpy)

    def distanceBetweenPose(self, pose1, pose2):
        """Compute the euclidian distance between 2 poses"""
        return math.sqrt(pow(pose2.position.x-pose1.position.x, 2) +
                         pow(pose2.position.y-pose1.position.y, 2))

    def computePathLength(self, plan):
        """Compute the length path with the poses of the plan"""
        poses = plan.poses
        pathLength = 0
        #Iteration among along the poses in order to compute the length
        for index in range(1, len(poses)):
            pathLength += self.distanceBetweenPose(poses[index-1].pose, poses[index].pose)
        return pathLength

    def get_pose_path_length(self,bot,goal,pose):
        start =PoseStamped()

        goal.header.frame_id = bot + "/map"
        start.header = goal.header
        start.pose.position = pose.transform.translation
        start.pose.orientation = pose.transform.rotation

        service = bot+'/move_base/make_plan'
        print("Waiting for service {}".format(service))
        try:
            rospy.wait_for_service(service, 2.0)
            get_plan = rospy.ServiceProxy(service, GetPlan)
            resp1 = get_plan(start, goal,0.1)
            dist = self.computePathLength(resp1.plan)
            print(dist)
            return dist, True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return 10000, False

    def get_robot_pose(self, bot_name):
        transform = []
        flag = []
        map_id = bot_name + "/map"
        frame_id = bot_name + "/base_footprint"
        try:
            transform = self.tf_buffer.lookup_transform(map_id,frame_id, rospy.Time(0))
            flag = True
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error with transform: {}".format(frame_id))
            flag = False

        return transform, flag

    #def callback_pose_stimpy(self):

    def get_pose_distance(self,goal,pose):
        g_x = goal.pose.position.x
        g_y = goal.pose.position.y
        g_z = goal.pose.position.z

        p_x = pose.transform.translation.x
        p_y = pose.transform.translation.y
        p_z = pose.transform.translation.z

        g = np.array([g_x, g_y, g_z])
        p = np.array([p_x, p_y, p_z])
        dist = np.linalg.norm(g-p)
        return dist

    def callback_goal(self, msg):
        """
        Function to handle the arrival of pointcloud data
        """
        print("msg: {}".format(msg))
        bots_pose = []
        for bot in bots:
            pose,flag = self.get_robot_pose(bot)
            if flag is False:
                return
            bots_pose.append(pose)

        valid = False
        dist = np.zeros(len(bots))
        for i in range(len(bots_pose)):
            #dist[i]=(self.get_pose_distance(msg, bots_pose[i]))
            dist[i],flag=self.get_pose_path_length(bots[i], msg, bots_pose[i])
            if flag is True:
                valid = True

        if valid is False:
            print("No valid path found, reverting to euclidean...")
            for i in range(len(bots_pose)):
                dist[i]=(self.get_pose_distance(msg, bots_pose[i]))

        index_min = np.argmin(dist)

        msg.header.frame_id = bots[index_min] + "/map"
        self.goal_pubs[index_min].publish(msg)
        print(index_min)

goalAllocator = GoalListener()
rospy.spin()