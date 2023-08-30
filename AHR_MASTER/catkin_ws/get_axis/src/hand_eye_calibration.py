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
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, Vector3, Quaternion
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
class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
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

        self.bridge = CvBridge()
        self.MARKERLEN = 0.076
        self.DICT_GET = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[609.9130249023438, 0.0, 325.3503723144531],
        [  0.0, 609.3018798828125, 239.77639770507812],
        [  0.  ,         0.    ,      1.        ]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
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
        
        self.hand_eye_H_2 = np.array([[0,  -1,  0, -0.06731279],
                                      [0,  0, 1, -0.02589233],
                                      [-1, 0,  0, 0.05662955],
                                      [0,  0,  0,     1]])

        # self.hand_eye_H_2 = np.array([[-0.0248467,   0.99906731, -0.03531499, -0.06731279],
        #                               [-0.99962673, -0.02523101, -0.01047868, -0.02589233],
        #                               [-0.01135994,  0.03504144,  0.99932129, 0.05662955],
        #                               [0,  0,  0,     1]])
    def get_aruco_pose(self):
        try:
            # while True:
                self.frames = self.pipeline.wait_for_frames()
                self.aligned_frames = self.align.process(self.frames)
                self.color_frame = self.aligned_frames.get_color_frame()
                self.color_image = np.asanyarray(self.color_frame.get_data())
                markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(self.color_image, self.DICT_GET, parameters=self.ARUCO_PARAMETERS)
                
                if len(markerCorners) > 0 and not self.marker_processed:  # if any markers detected and not processed
                    cv2.imwrite("detected_marker.jpg", self.color_image)  # save the image
                    
                    # Reload the image from file
                    color_image_saved = cv2.imread("detected_marker.jpg")
                    
                    markerCorners_saved, markerIds_saved, _ = cv2.aruco.detectMarkers(color_image_saved, self.DICT_GET, parameters=self.ARUCO_PARAMETERS)
                    img_with_markers = cv2.aruco.drawDetectedMarkers(color_image_saved.copy(), markerCorners_saved, markerIds_saved)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners_saved, self.MARKERLEN, self.camera_matrix, self.dist_coeffs)
                    
                    for i in range(len(markerIds_saved)):
                        image_with_axis = aruco.drawAxis(img_with_markers, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                        rmat, _ = cv2.Rodrigues(rvecs[i])
                        # Form the 4x4 transformation matrix
                        self.transform_mat = np.zeros(shape=(4, 4))
                        self.transform_mat[:3, :3] = rmat
                        self.transform_mat[:3, 3] = tvecs[i].reshape(-1)
                        self.transform_mat[3, 3] = 1
                        print("Camera to Aruco Marker matrix:\n", self.transform_mat)

                        rotation = self.transform_mat[:3, :3]
                        tran = self.transform_mat[:3, 3]
                        self.camera_rotation.append(rotation)
                        self.camera_tran.append(tran)
                        # print(self.camera_rotation)

                    # cv2.imshow('Image with axis', image_with_axis)
                    # cv2.waitKey(1)

                    self.marker_processed = True  # set the flag to True after processed
        finally:
            # Stop streaming
            print("d")

    def get_robot_pose(self):
                current_joint = self.move_group.get_current_joint_values()
                current_pose = self.move_group.get_current_pose().pose
                print("joint:",current_joint)
                print("pose:", current_pose)
                print("경로 계산에 성공하였습니다!")

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

    def calibrate_hand_eye(self):
        print("rrrrrrr:", self.camera_rotation)
        print("tttttt:", self.camera_tran)
        self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye( self.robot_rotation, self.robot_tran, self.camera_rotation, self.camera_tran,)
        return  self.R_cam2gripper, self.t_cam2gripper

    def plan_joint(self):
        current_joint = self.move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose().pose
        # print("joint:",current_joint)
        # print("pose:", current_pose)
        try:
            data = [
                        [0.00, -1.085, 0.743, 0.00, 0.393, 0.00],
                        [0.15, -1.085, 0.743, 0.00, 0.393, 0.00],
                        # [0.25, -1.085, 0.743, 0.00, 0.393, 0.00],
                        [0.3, -1.085, 0.743, -0.30, 0.493, 0.00],
                        [0.4, -1.085, 0.743, -0.50, 0.493, 0.00],
                        # [0.4, -0.985, 0.643, -0.50, 0.493, 0.00],
                        [0.5, -0.985, 0.643, -0.60, 0.593, 0.00],
                        [0.5, -0.985, 0.643, -0.80, 0.693, 0.00],
                        # [0.5, -0.785, 0.493, -0.80, 0.693, 0.00],
                        [0.5, -0.735, 0.393, -0.80, 0.693, 0.00],
                        [0.5, -0.535, 0.293, -0.70, 0.643, 0.00],
                        # [0.3, -0.535, 0.293, -0.70, 0.643, 0.00],
                        [0.2, -0.535, 0.293, -0.50, 0.443, 0.00],
                        [0.1, -0.535, 0.293, -0.30, 0.443, 0.00],
                        # [0.0, -0.235, -0.157, 0.00, 0.593, 0.00],
                        [-0.2, -0.235, -0.157, 0.20, 0.693, 0.00],
                        [-0.4, -0.235, -0.157, 0.50, 0.793, 0.00],
                        # [-0.5, -0.235, -0.157, 0.70, 0.893, 0.00],
                        [-0.5, -0.535, 0.293, 0.70, 0.593, 0.00],
                        [-0.5, -0.985, 0.634, 0.90, 0.593, 0.00],
                        # [-0.65, -1.135, 0.793, 1.00, 0.593, 0.00],
                        [-0.55, -1.235, 0.943, 0.80, 0.593, 0.00],
                        [-0.45, -1.235, 0.843, 0.65, 0.593, 0.00],
                        # [-0.35, -1.235, 0.843, 0.55, 0.593, 0.00],
                        [-0.2, -1.235, 0.843, 0.40, 0.493, 0.00],
                        [0.00, -1.235, 0.843, 0.00, 0.493, 0.00],
                        # [0.2, -1.085, 0.643, -0.10, 0.593, 0.00],
                        [0.4, -1.085, 0.643, -0.40, 0.643, 0.00],
                        [0.6, -1.085, 0.643, -0.75, 0.843, 0.00]
            ]
            self.transformation_matrices = []
            self.move_group = self.move_group
            self.joint_goal = self.move_group.get_current_joint_values()
            self.hand_eye_input = []
            try:
                for id, self.p in enumerate(data):
                    # self.settings.clear()
                    self.joint_goal[0] = self.p[0]
                    self.joint_goal[1] = self.p[1]
                    self.joint_goal[2] = self.p[2]
                    self.joint_goal[3] = self.p[3]
                    self.joint_goal[4] = self.p[4]
                    self.joint_goal[5] = self.p[5]
                    success=self.move_group.go(self.joint_goal, wait=True)
                    self.get_aruco_pose()
                    self.marker_processed = False
                    # time.sleep(0.3)
                    if not success:
                        print("siab")
                    elif success:
                        current_joint = self.move_group.get_current_joint_values()
                        current_pose = self.move_group.get_current_pose().pose
                        # print("joint:",current_joint)
                        # print("pose:", current_pose)
                        # print("경로 계산에 성공하였습니다!")

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
                        self.transformation_matrices.append(self.matrix)
                        print("Base to End:", self.matrix)
                        rot = self.matrix[:3, :3]
                        tr = self.matrix[:3, 3]
                        self.robot_rotation.append(rot)
                        self.robot_tran.append(tr)
                        # print(id)
                        # print("4x4 Transformation Matrix:")
                        # print(self.matrix)

                    self.move_group.stop()
                    self.move_group.clear_pose_targets()
                # print("All 4x4 Transformation Matrices:")
                # for self.matrix in self.transformation_matrices:
                #     print(self.matrix)
                self.calibrate_hand_eye()
                print(self.R_cam2gripper)
                print(self.t_cam2gripper)
                # print("hi:", self.camera_rotation)
            except KeyboardInterrupt:
                return
        except KeyboardInterrupt:
            return

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 0.9322404
        pose_goal.orientation.w = 0.63709769
        pose_goal.position.x = 0.78713524
        pose_goal.position.y = -0.18964361
        pose_goal.position.z = 0.63221957
        pose_goal.orientation.x = 0.03654361
        pose_goal.orientation.y = 0.76961824
        pose_goal.orientation.z = 0.02142083
        # pose_goal.orientation.x =0.2558591
        # pose_goal.orientation.y =0.2558591
        # pose_goal.orientation.z =0

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return pose_goal, current_pose
    


def main():
    try:
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()
        # tutorial.robot_status()
        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        tutorial.plan_joint()
        # tutorial.get_robot_pose()
        # tutorial.go_to_checkerboard()
        # tutorial.go_to_pose_goal()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
if __name__ == "__main__":
    main()

