#!/usr/bin/env python

"""
neupan_core is the main class for the neupan_ros package. It is used to run the NeuPAN algorithm in the ROS framework, which subscribes to the laser scan and localization information, and publishes the velocity command to the robot.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

from neupan import neupan
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2
from math import sin, cos, atan2
import numpy as np
from neupan.util import get_transform
import tf
import sensor_msgs.point_cloud2 as pc2


class neupan_core:
    def __init__(self) -> None:

        rospy.init_node("neupan_node", anonymous=True)

        # ros parameters
        self.planner_config_file = rospy.get_param("~config_file", None)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "laser_link")
        self.marker_size = float(rospy.get_param("~marker_size", "0.05"))
        self.marker_z = float(rospy.get_param("~marker_z", "1.0"))

        scan_angle_range_para = rospy.get_param("~scan_angle_range", "-3.14 3.14")
        self.scan_angle_range = np.fromstring(
            scan_angle_range_para, dtype=np.float32, sep=" "
        )

        self.scan_downsample = int(rospy.get_param("~scan_downsample", "1"))

        scan_range_para = rospy.get_param("~scan_range", "0.0, 5.0")
        self.scan_range = np.fromstring(scan_range_para, dtype=np.float32, sep=" ")

        self.dune_checkpoint = rospy.get_param("~dune_checkpoint", None)
        self.refresh_initial_path = rospy.get_param("~refresh_initial_path", False)
        self.flip_angle = rospy.get_param("~flip_angle", False)
        self.include_initial_path_direction = rospy.get_param("~include_initial_path_direction", False)
        
        if self.planner_config_file is None:
            raise ValueError(
                "No planner config file provided! Please set the parameter ~config_file"
            )

        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(
            self.planner_config_file, pan=pan
        )
        # print()

        # data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False

        # publisher
        self.vel_pub = rospy.Publisher("/neupan_cmd_vel", Twist, queue_size=10)
        self.plan_pub = rospy.Publisher("/neupan_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher(
            "/neupan_ref_state", Path, queue_size=10
        )  # current reference state
        self.ref_path_pub = rospy.Publisher(
            "/neupan_initial_path", Path, queue_size=10
        )  # initial path

        ## for rviz visualization
        self.point_markers_pub_dune = rospy.Publisher(
            "/dune_point_markers", MarkerArray, queue_size=10
        )
        self.robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
        self.point_markers_pub_nrmp = rospy.Publisher(
            "/nrmp_point_markers", MarkerArray, queue_size=10
        )

        self.listener = tf.TransformListener()

        # subscriber
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # three types of initial path:
        # 1. from given path
        # 2. from waypoints
        # 3. from goal position
        rospy.Subscriber("/initial_path", Path, self.path_callback)
        rospy.Subscriber("/neupan_waypoints", Path, self.waypoints_callback)
        rospy.Subscriber("/neupan_goal", PoseStamped, self.goal_callback)
        
    def run(self):

        r = rospy.Rate(50)

        while not rospy.is_shutdown():

            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.map_frame, self.base_frame, rospy.Time(0)
                )

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]
                self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1,
                    "waiting for tf for the transform from {} to {}".format(
                        self.base_frame, self.map_frame
                    ),
                )
                continue

            if self.robot_state is None:
                rospy.logwarn_throttle(1, "waiting for robot state")
                continue

            rospy.loginfo_once(
                "robot state received {}".format(self.robot_state.tolist())
            )

            if (
                len(self.neupan_planner.waypoints) >= 1
                and self.neupan_planner.initial_path is None
            ):
                self.neupan_planner.set_initial_path_from_state(self.robot_state)
                # print('set initial path', self.neupan_planner.initial_path)

            if self.neupan_planner.initial_path is None:
                rospy.logwarn_throttle(1, "waiting for neupan initial path")
                continue

            rospy.loginfo_once("initial Path Received")
            self.ref_path_pub.publish(
                self.generate_path_msg(self.neupan_planner.initial_path)
            )

            if self.obstacle_points is None:
                rospy.logwarn_throttle(
                    1, "No obstacle points, only path tracking task will be performed"
                )

            action, info = self.neupan_planner(self.robot_state, self.obstacle_points)

            self.stop = info["stop"]
            self.arrive = info["arrive"]

            if info["arrive"]:
                # print(action)
                rospy.loginfo_throttle(0.1, "arrive at the target")

            # publish the path and velocity
            self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
            self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
            self.vel_pub.publish(self.generate_twist_msg(action))

            self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
            self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
            self.robot_marker_pub.publish(self.generate_robot_marker_msg())

            if info["stop"]:
                rospy.logwarn_throttle(
                    0.5,
                    "neupan stop with the min distance "
                    + str(self.neupan_planner.min_distance.detach().item())
                    + " threshold "
                    + str(self.neupan_planner.collision_threshold),
                )

            r.sleep()

    # scan callback
    def scan_callback(self, scan_msg):

        if self.robot_state is None:
            return None

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []
        # x, y, z, yaw, pitch, roll = self.lidar_offset

        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                i % self.scan_downsample == 0
                and distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            rospy.loginfo_once("No valid scan points")
            return None

        point_array = np.hstack(points)

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.map_frame, self.lidar_frame, rospy.Time(0)
            )

            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
            rospy.loginfo_once("Scan obstacle points Received")

            return self.obstacle_points

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo_throttle(
                1,
                "waiting for tf for the transform from {} to {}".format(
                    self.lidar_frame, self.map_frame
                ),
            )
            return

    def path_callback(self, path):

        initial_point_list = []

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = initial_point_list[-1][2, 0]
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo_throttle(0.1, "initial path update from given path")
            self.neupan_planner.set_initial_path(initial_point_list)
            self.neupan_planner.reset()
    
    def waypoints_callback(self, path):
        
        '''
        Utilize multiple waypoints (goals) to set the initial path
        '''

        waypoints_list = [self.robot_state]

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = waypoints_list[-1][2, 0]
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            waypoints_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo_throttle(0.1, "initial path update from waypoints")
            self.neupan_planner.update_initial_path_from_waypoints(waypoints_list)
            self.neupan_planner.reset()

    def goal_callback(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        self.goal = np.array([[x], [y], [theta]])

        print(f"set neupan goal: {[x, y, theta]}")

        rospy.loginfo_throttle(0.1, "initial path update from goal position")
        self.neupan_planner.update_initial_path_from_goal(self.robot_state, self.goal)
        self.neupan_planner.reset()


    def quat_to_yaw_list(self, quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

    # generate ros message
    def generate_path_msg(self, path_list):

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0

        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.seq = index

            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation = self.yaw_to_quat(point[2, 0])

            path.poses.append(ps)

        return path

    def generate_twist_msg(self, vel):

        if vel is None:
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        if self.stop or self.arrive:
            # print('stop flag true')
            return Twist()

        else:
            action = Twist()

            action.linear.x = speed
            action.angular.z = steer

            return action

    def generate_dune_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.dune_points is None:
            return
        else:
            points = self.neupan_planner.dune_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 160 / 255
                marker.color.g = 32 / 255
                marker.color.b = 240 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_nrmp_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.nrmp_points is None:
            return
        else:
            points = self.neupan_planner.nrmp_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 255 / 255
                marker.color.g = 128 / 255
                marker.color.b = 0 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_robot_marker_msg(self):

        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.seq = 0
        marker.header.stamp = rospy.get_rostime()

        marker.color.a = 1.0
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255

        marker.id = 0

        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            marker.type = 1

            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]

            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(self.robot_state[2, 0])

        return marker

    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw
