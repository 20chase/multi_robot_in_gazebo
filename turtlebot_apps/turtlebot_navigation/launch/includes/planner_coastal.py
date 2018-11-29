import rospy
import time
import tf
import os

import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from cartographer_ros_msgs.msg import SubmapList
from cartographer_ros_msgs.srv import FinishTrajectory
from point_reader import PointReader


class CoastalPlanner(object):
    def __init__(self):
        rospy.init_node("slamer")

        self.loop = rospy.Rate(100)

        self.listener = tf.TransformListener()

        self.goal = np.zeros(2)
        self.last_recover_point = np.zeros(2)
        self.start_planning_flag = False
        self.finish_planning_flag = False

        reader = PointReader()
        self.recover_points = reader.read("recover_points_supermarket.txt")

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        rospy.Subscriber("/submap_list", SubmapList, self._submap_cb)
        rospy.Subscriber("/clicked_point", PointStamped, self._goal_cb)
        rospy.Subscriber("/evaluator/start", Bool, self._start_cb)
        rospy.Subscriber("/evaluator/finish", Bool, self._finish_cb)

        rospy.wait_for_service("finish_trajectory")
        self._finish_traj = rospy.ServiceProxy("finish_trajectory", FinishTrajectory)

    def eval(self):
        while not rospy.is_shutdown():
            if self.start_planning_flag:
                self.start_planning_flag = False
                self.run()

    def run(self):
        self.finish_planning_flag = False
        self._init_slam_link()
        while not rospy.is_shutdown():
            if self.finish_planning_flag:
                break
            self._go_recover_point(10)
            self.loop.sleep()

    def _go_recover_point(self, radius):
        robot_pos = np.zeros(3)
        (trans, rot) = self.listener.lookupTransform("/map", "/slam_link", rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        robot_pos[0] = trans[0]
        robot_pos[1] = trans[1]
        robot_pos[2] = euler[2]
        candidate_points = []
        values_map = []
        values_goal = []
        for point in self.recover_points:
            dx = robot_pos[0] - point[0]
            dy = robot_pos[1] - point[1]
            distance = np.hypot(dx, dy)
            goal_distance = -np.hypot(self.goal[0] - point[0], self.goal[1] - point[1])
            if distance < radius:
                candidate_points.append(np.asarray([point[0], point[1]]))
                values_map.append((20. - point[2]) / 20.)
                values_goal.append(goal_distance)
        candidate_points = np.asarray(candidate_points)
        values_map = 0.2 * self._normalize_values(np.asarray(values_map))
        values_goal = 1. * self._normalize_values(np.asarray(values_goal))
        rospy.loginfo("values_map: " + str(values_map))
        rospy.loginfo("values_goal: " + str(values_goal))
        values = values_map + values_goal
        sorts = np.argsort(values)
        rospy.loginfo("values: " + str(values))
        recover_point = candidate_points[sorts[-1]]
        self._set_goal(recover_point)
            
    def _normalize_values(self, values):
        return (values - values.mean()) / (values.std() + 1e-6)

    def _set_goal(self, point):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.orientation.w = 1.
        self.goal_pub.publish(msg)

    def _init_slam_link(self):
        self._stop_slam_pose()
        trans, rot = self.listener.lookupTransform("/map", "/true_pose", rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        self._start_slam_pose(trans, euler)

    def _start_slam_pose(self, trans, rot):
        cmd = "rosrun cartographer_ros cartographer_start_trajectory \
        -configuration_directory \
        '/home/xinjing/catkin_ws_ext/install_isolated/share/cartographer_ros/configuration_files' \
        -configuration_basename turtlebot_2d_localization.lua \
        -initial_pose \
        '{to_trajectory_id=0,relative_pose=\
        {translation={%f,%f,%f},rotation={%f,%f,%f}},timestamp = 0}'" \
        % (trans[0], trans[1], trans[2], rot[0], rot[1], rot[2])
        os.system(cmd)

    def _stop_slam_pose(self):
        self._finish_traj(self.new_traj_id)

    def _submap_cb(self, msg):
        self.new_traj_id = msg.submap[-1].trajectory_id

    def _goal_cb(self, msg):
        self.goal[0] = msg.point.x
        self.goal[1] = msg.point.y
        self.update_goal_flag = True

    def _start_cb(self, msg):
        self.start_planning_flag = msg.data

    def _finish_cb(self, msg):
        self.finish_planning_flag = msg.data

if __name__ == "__main__":
    planner = CoastalPlanner()
    planner.run()

        
