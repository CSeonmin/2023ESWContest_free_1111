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
from moveit_msgs.msg import OrientationConstraint, Constraints
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, Vector3, Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf
from scipy.spatial.transform import Rotation as R
from get_axis.srv import MatrixCalculation, MatrixCalculationResponse
from get_axis.srv import Boundingbox, BoundingboxResponse
import pyrealsense2 as rs
from gripper_service.srv import Srvgripper
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True
class Obstacle(object):

    def __init__(self):
        super(Obstacle, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("Obstacle_avoidance", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group = move_group
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=40,
        )

        self.obstacle_client = rospy.ServiceProxy('obstacle_translation', MatrixCalculation) ## Obstacle Translation 좌표 서비스
        self.gripper_client = rospy.ServiceProxy('gripper_srv', Srvgripper)
        # self.bounding_client = rospy.ServiceProxy('bounding_box', Boundingbox)
        self.actual_path_publisher = rospy.Publisher("/actual_robot_path", Path, queue_size=10) ## 실제 로봇 경로
        self.path_publisher = rospy.Publisher("/robot_path", Path, queue_size=10) ## Robot path 시각화
        self.marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
        self.actual_path = Path()
        self.actual_path.header.frame_id = move_group.get_planning_frame()
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update_actual_path)  # 실제 로봇 경로 0.05주기 업데이트
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.hand_eye_m = np.array([[0,  0,  1, -0.015],
                                [-1,  0,  0, 0],
                                [0, -1,  0, 0.06],
                                [0,  0,  0,     1]])
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.target_translations = []
        self.obstacle_translations = []
        self.marker_ids = []
        self.obstacle_name_counter = 0
        self.target_name_counter = 0
        self.box_names = {"obstacle": [], "target": []}

    def go_init_pose(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/2.85
        joint_goal[2] = pi/3.9
        joint_goal[3] = 0
        joint_goal[4] = pi/7.85
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def second_pose(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/2.85
        joint_goal[2] = pi/3.9
        joint_goal[3] = 0
        joint_goal[4] = pi/7.85
        joint_goal[5] = -pi/2
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def pick_end(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def store_move(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -pi/2
        joint_goal[1] = -pi/2.85
        joint_goal[2] = pi/3.9
        joint_goal[3] = 0
        joint_goal[4] = pi/7.85
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def store_move_second(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -pi/2
        joint_goal[1] = -pi/15.2
        joint_goal[2] = pi/3.6
        joint_goal[3] = 0
        joint_goal[4] = pi/3.9
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def update_actual_path(self, event):  ## 로봇의 실제 경로 업데이트 함수
        pose = self.move_group.get_current_pose().pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose
        self.actual_path.poses.append(pose_stamped)
        # print("pose_stamped", pose_stamped)
        try:
            self.actual_path_publisher.publish(self.actual_path)
        except rospy.exceptions.ROSException:
            rospy.logwarn("토픽이 닫혔습니다.")

    def obstacle_homogeneous_matrix(self):
        current_pose = self.move_group.get_current_pose().pose
        # print("joint:",current_joint)
        position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
        quaternion = [
                        current_pose.orientation.x,
                        current_pose.orientation.y,
                        current_pose.orientation.z,
                        current_pose.orientation.w,
                        ]
        current_pose = quaternion_matrix(quaternion)
        current_pose[:3, 3] = position ## robot pose
        detected_object_data = self.get_obstacle_translation()
        if detected_object_data is None:
            rospy.logerr("Failed to get obstacle translation.")
            return
        self.target_translations.clear()
        self.obstacle_translations.clear()
        for object in detected_object_data:
            self.name = object.name
            self.detect_obj = object.detect_obj
            translation, bounding_box_length = self.get_translation(current_pose, object, self.name)
            data = {'translation': translation, 'bounding_box_length': bounding_box_length}
            if self.name == "persimmon":
                self.target_translations.append(data)
            elif self.name =="bottle":
                self.obstacle_translations.append(data)
        # print("self.target_translations", self.target_translations)
        # print("self.obstacle_translations\n", self.obstacle_translations)

    def get_translation(self, current_pose, item, name):
        object_position = [(item.center_point.translation.x),(item.center_point.translation.y),(item.center_point.translation.z)]
        object_quaternion = [item.center_point.rotation.x,
                             item.center_point.rotation.y,
                             item.center_point.rotation.z,
                             item.center_point.rotation.w
                             ]
        bounding_box_length = [item.bounding_box_length.translation.x, item.bounding_box_length.translation.y, item.bounding_box_length.translation.z]
        object_matrix = quaternion_matrix(object_quaternion)
        object_matrix[:3, 3] = object_position
        base_to_camera = np.dot(current_pose, self.hand_eye_m)
        base_to_object = np.dot(base_to_camera, object_matrix)
        object_translation = base_to_object[:3,3]
        object_rotation = base_to_object[:3,:3]
        object_rotation = R.from_matrix(object_rotation)
        object_rotation = object_rotation.as_quat()
        return object_translation, bounding_box_length
    
    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def go_to_target(self):
        quaternion = tf.transformations.quaternion_from_euler(0.0 ,0.0, 0.0 )
        move_group = self.move_group
        move_group.set_start_state_to_current_state()
        current_pose = self.move_group.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        for target in self.target_translations:
            print("target", target)
            pose_goal.position.x = target['translation'][0] 
            pose_goal.position.y = target['translation'][1] + 0.06
            pose_goal.position.z = target['translation'][2] #+ 0.058
            pose_goal.orientation.x = current_pose.orientation.x
            pose_goal.orientation.y = current_pose.orientation.y
            pose_goal.orientation.z = current_pose.orientation.z
            pose_goal.orientation.w = current_pose.orientation.w
            move_group.set_pose_target(pose_goal)
            self.success = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def get_obstacle_translation(self):
        try:
            response = self.obstacle_client()
            return response.result_objects
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None
        
    def gripper(self, operation):
        try:
            # srv_gripper = Srvgripper()
            # srv_gripper = int(operation)
            response  = self.gripper_client(operation)
            rospy.loginfo("Received result: %ld", response.result)
            # return response.result
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None
def main():
    try:
        obstacle_avoid = Obstacle()
        input("============ Press `Enter` to execute a saved path ...")
        obstacle_avoid.go_init_pose()
        obstacle_avoid.obstacle_homogeneous_matrix()
        if obstacle_avoid.detect_obj == True:
            print("객체를 인식했습니다.")
            obstacle_avoid.gripper(1)
            obstacle_avoid.second_pose()
            obstacle_avoid.go_to_target()
            if obstacle_avoid.success == True:
                obstacle_avoid.gripper(2)
                obstacle_avoid.pick_end()
                obstacle_avoid.go_init_pose()
                obstacle_avoid.store_move()
                obstacle_avoid.store_move_second()
                obstacle_avoid.gripper(1)
                obstacle_avoid.store_move()
                obstacle_avoid.go_init_pose()
                obstacle_avoid.gripper(2)
            else:
                print("가지 못하는 자세입니다.")
                obstacle_avoid.gripper(2)
                obstacle_avoid.go_init_pose()
        else:
            print("객체를 인식하지 못했습니다.")
            obstacle_avoid.gripper(2)
            obstacle_avoid.go_init_pose()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
if __name__ == "__main__":
    main()