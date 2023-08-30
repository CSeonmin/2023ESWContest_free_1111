#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import datetime
from pathlib import Path
from typing import List
from tf.transformations import *
from typing import Iterable, Tuple
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, Vector3, Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from aruco_marker_detect.srv import MatrixCalculation
import tf
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Visp(object):

    def __init__(self):
        super(Visp, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("visual_servoing", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "indy7"
        # group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=40,
        )


        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        # print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        
        self.marker_processed = False

        self.camera_rotation = []
        self.camera_tran = []

        self.robot_rotation = []
        self.robot_tran = []
        
        self.hand_eye_m = np.array([[-0.0248467,   0.99906731, -0.03531499,-0.09401433],
                                    [-0.99962673, -0.02523101, -0.01047868, 0.02838461],
                                    [-0.01135994,  0.03504144,  0.99932129,  0.05443796],
                                    [0,            0,           0 ,           1]])
        
        self.hand_eye_H = np.array([[ 0.00343606,  0.06204391,  0.99806751, -0.22018163],
                                    [-0.97760238,  0.21023658, -0.00970355, -0.01569389],
                                    [-0.21043235, -0.97567983,  0.06137666, 0.04162407],
                                    [0,            0,           0,          1]])
        
        # self.hand_eye_H_2 = np.array([[-0.02024216,  0.01732317,  0.99964502, -0.06731279],
        #                         [-0.96749772,  0.25174268, -0.02395373, -0.02589233],
        #                         [-0.25206827, -0.96763915,  0.01166431, 0.05662955],
        #                         [0,           0,             0,         1]])
        
        # self.hand_eye_H_2 = np.array([[0,  0,  0, -0.06731279],
        #                               [0,  0, 0,  -0.02589233],
        #                               [0, 0,  0, 0.05662955],
        #                               [0,      0,        0,    1]])

        # self.hand_eye_H_2 = np.array([[0,  -1,  0, -0.06731279],
        #                                 [0,  0, 1, -0.02589233],
        #                                 [-1, 0,  0, 0.05662955],
        #                                 [0,  0,  0,     1]])

        self.hand_eye_H_2 = np.array([[0,  0,  1, -0.015],
                                [-1,  0,  0, 0],
                                [0, -1,  0, 0.06],
                                [0,  0,  0,     1]])

        # self.hand_eye_H_2 = np.array([[-0.0248467,   0.99906731, -0.03531499, -0.015],
        #                 [-0.99962673, -0.02523101, -0.01047868, 0],
        #                 [-0.01135994,  0.03504144,  0.99932129, 0.06],
        #                 [0,  0,  0,     1]])
        self.is_moving = False
        self.aruco_client = rospy.ServiceProxy('matrix_calculation', MatrixCalculation)
        path_publisher = rospy.Publisher("/robot_path_joint", PoseStamped, queue_size=10)
        self.path_publisher = path_publisher
        # self.aruco_transform_subscriber = rospy.Subscriber("/aruco_transform", Transform, self.get_cam_aruco_callback)
        # self.ignore_until = rospy.Time.now()
        # self.ignore_duration = rospy.Duration(5)
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    def get_aruco_transform(self):
        try:
            response = self.aruco_client()
            return response.transform
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None
        
    def get_robot_pose(self):

                current_joint = self.move_group.get_current_joint_values()
                current_pose = self.move_group.get_current_pose().pose
                print("joint:",current_joint)
                print("pose:", current_pose)

                position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
                quaternion = [
                                current_pose.orientation.x,
                                current_pose.orientation.y,
                                current_pose.orientation.z,
                                current_pose.orientation.w,
                                ]
                self.matrix = quaternion_matrix(quaternion)
                # position_mm = [coord * 1000 for coord in position]  # Convert position to millimeters
                self.matrix[:3, 3] = position
                print("4x4 Transformation Matrix:")
                print(self.matrix)

    def base_to_camera(self):
        current_pose = self.move_group.get_current_pose().pose
        position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
        quaternion = [
                        current_pose.orientation.x,
                        current_pose.orientation.y,
                        current_pose.orientation.z,
                        current_pose.orientation.w,
                        ]
        self.matrix = quaternion_matrix(quaternion)
        self.matrix[:3, 3] = position ## robot pose
        self.reversal_Y =np.array([[0,0,1,0],
                                    [0,1,0,0],
                                    [-1,0,0,0],
                                    [0,0,0,1]])
        self.base_camera = np.dot(self.matrix,self.hand_eye_m) ## base to camera
        # self.base_camera = np.dot(self.matrix,self.hand_eye_H) ## base to camera
        # self.base_camera = np.dot(self.matrix,self.hand_eye_H_2) ## base to camera
        # self.base_camera = np.dot(self.base_camera, self.reversal_Y)
        self.base_camera_t = self.base_camera[:3, 3]
        r = self.base_camera[:3, :3]
        r = R.from_matrix(r)
        self.base_camera_r = r.as_quat()

    def camera_axis(self):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = self.base_camera_r[3]
        pose_goal.position.x = self.base_camera_t[0] 
        pose_goal.position.y = self.base_camera_t[1] 
        pose_goal.position.z = self.base_camera_t[2]
        pose_goal.orientation.x = self.base_camera_r[0]
        pose_goal.orientation.y = self.base_camera_r[1]
        pose_goal.orientation.z = self.base_camera_r[2]
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        pose_stamped.pose = pose_goal
        self.path_publisher.publish(pose_stamped)

        return pose_goal
    
    def go_to_object(self):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = self.q[3]
        pose_goal.position.x = self.t[0] 
        pose_goal.position.y = self.t[1] 
        pose_goal.position.z = self.t[2]  if self.t[2] >= 1.0 else 0.8
        pose_goal.orientation.x = self.q[0]
        pose_goal.orientation.y = self.q[1]
        pose_goal.orientation.z = self.q[2]

        move_group.set_pose_target(pose_goal)
        self.success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.sleep(0.3)
        return pose_goal
    
    def start_visual_servo(self):
        

        rospy.spin()


def main():
    try:
        visual_servo = Visp()
        # tutorial.robot_status()

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        visual_servo.base_to_camera()
        visual_servo.camera_axis()

        # visual_servo.go_to_pose_goal()
        # while True:
        #     visual_servo.go_to_object()
        #     visual_servo.base_to_object()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
if __name__ == "__main__":
    main()
