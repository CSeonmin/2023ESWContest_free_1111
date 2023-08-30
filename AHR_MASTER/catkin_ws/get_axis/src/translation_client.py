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
        # group_name = "indy7"
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group = move_group
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=40,
        )

        self.obstacle_client = rospy.ServiceProxy('obstacle_translation', MatrixCalculation) ## Obstacle Translation 좌표 서비스
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
        # self.hand_eye_m = np.array([[-0.0248467,   0.99906731, -0.03531499,-0.09401433],
        #                     [-0.99962673, -0.02523101, -0.01047868, 0.02838461],
        #                     [-0.01135994,  0.03504144,  0.99932129,  0.05443796],
        #                     [0,            0,           0 ,           1]])
        self.hand_eye_H_2 = np.array([[0,  0,  1, -0.015],
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
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
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
            if self.name == "orange":
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
        # base_to_camera = np.dot(current_pose, self.hand_eye_m)
        base_to_camera = np.dot(current_pose, self.hand_eye_H_2)
        base_to_object = np.dot(base_to_camera, object_matrix)
        object_translation = base_to_object[:3,3]
        object_rotation = base_to_object[:3,:3]
        object_rotation = R.from_matrix(object_rotation)
        object_rotation = object_rotation.as_quat()
        return object_translation, bounding_box_length
    
    def get_nearest_target(self, target_translation):
        distances = [np.linalg.norm(np.array(target['translation'])) for target in target_translation]
        sorted_indices = np.argsort(distances)
        # print("@@@", distances)
        return [target_translation[i] for i in sorted_indices]
    
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
    
    def visual_box_text(self, translation, obstacle_counter, target_counter, name_prefix, box_name):
        if name_prefix == "target_":
            marker_id = target_counter
            target_counter += 1
        else:
            marker_id = obstacle_counter + 1000
            obstacle_counter += 1
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.id = marker_id
        marker.text = box_name
        marker.pose.position.x = translation[0]
        marker.pose.position.y = translation[1]
        marker.pose.position.z = translation[2] + 0.13  # Box 위로 조금 올림
        marker.scale.z = 0.05  # 텍스트 크기
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.marker_publisher.publish(marker)
        self.marker_ids.append(marker_id)

    def add_box(self, translation, bounding_box, name_prefix, size_prefix, timeout=4):
        box_name = name_prefix + str(self.obstacle_name_counter if name_prefix == "obstacle_" else self.target_name_counter)
        if name_prefix == "obstacle_":
            self.obstacle_name_counter += 1
            self.box_names["obstacle"].append(box_name)
        else:
            self.target_name_counter += 1
            self.box_names["target"].append(box_name)
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = "link0"
        box_pose.header.frame_id = "link1"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = translation[0]
        box_pose.pose.position.y = translation[1]
        box_pose.pose.position.z = translation[2]
        scene.add_box(box_name, box_pose, size=(size_prefix, bounding_box[0], bounding_box[1]))
        self.visual_box_text(translation, self.obstacle_name_counter, self.target_name_counter, name_prefix, box_name)
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    def add_objects(self, object_type, object_data, size_prefix, timeout):
        for object in object_data:
            self.add_box(object['translation'], object['bounding_box_length'], object_type + "_", size_prefix, timeout)
            self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_object_box(self, timeout=2):
        self.target_translation_nearest = self.get_nearest_target(self.target_translations)
        print("self.target_translation_nearest", self.target_translation_nearest)
        self.box_names = {"obstacle": [], "target": []}
        self.add_objects("target", self.target_translation_nearest, 0.075, timeout)
        self.add_objects("obstacle", self.obstacle_translations, 0.065, timeout)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    # def orientationconstraint(self):
    #     move_group = self.move_group
    #     ocm = OrientationConstraint()
    #     ocm.link_name = "tcp"  # tcp 링크의 방향 제어
    #     ocm.header.frame_id = "world"  # base 링크에 대해 정의된다.
    #     ocm.orientation.w = 0.7071068
    #     ocm.orientation.x = 0
    #     ocm.orientation.y = 0.7071068
    #     ocm.orientation.z = 0
    #     ocm.absolute_x_axis_tolerance = 0.5  # x 축 회전에 대한 허용오차(라디안 값)
    #     ocm.absolute_y_axis_tolerance = 0.5  # y 축 회전에 대한 허용오차(라디안 값)
    #     ocm.absolute_z_axis_tolerance = 0.5  # z 축 회전에 대한 허용오차(라디안 값)
    #     ocm.weight = 0.5  # 제약 조건의 가중치
    #     # 경로 제약 설정
    #     test_constraints = Constraints()
    #     test_constraints.orientation_constraints.append(ocm)
    #     move_group.set_path_constraints(test_constraints)

    def go_to_target(self):
        # quaternion = tf.transformations.quaternion_from_euler(0, pi/2, 0.0 )
        quaternion = tf.transformations.quaternion_from_euler(0, 0.0, 0.0 )
        move_group = self.move_group
        move_group.set_start_state_to_current_state()
        pose_goal = geometry_msgs.msg.Pose()
        for target in self.target_translation_nearest:
            print("target", target)
            # print("target['translation'][0]", target['translation'][0])
            # print("target['translation'][0]", target['translation'][1])
            # print("target['translation'][0]", target['translation'][2])
            pose_goal.position.x = target['translation'][0] -0.02
            pose_goal.position.y = target['translation'][1]
            pose_goal.position.z = target['translation'][2]
            pose_goal.orientation.x = quaternion[0]
            pose_goal.orientation.y = quaternion[1]
            pose_goal.orientation.z = quaternion[2]
            pose_goal.orientation.w = quaternion[3]
            move_group.set_pose_target(pose_goal)
            success = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
        #     move_group.set_pose_target(pose_goal)
        #     rospy.sleep(2)
        #     success, plan, _, _ = move_group.plan()
        #     rospy.sleep(2)
        #     move_group.execute(plan, wait = True)
        #     move_group.stop()
        #     move_group.clear_pose_targets()
        #     current_pose = self.move_group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)


        #     waypoints = [pose_goal] # 카르테시안 경로를 위한 waypoints 설정
        #     (plan, fraction) = move_group.compute_cartesian_path(
        #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        # )  # jump_threshold
        #     move_group.execute(plan, wait=True)
        #     move_group.stop()
        #     move_group.clear_pose_targets()
        #     current_pose = self.move_group.get_current_pose().pose
        #     return all_close(pose_goal, current_pose, 0.01)
        #     move_group.set_pose_target(pose_goal)
        #     success = move_group.go(wait=True)
        #     if not success:
        #         rospy.logerr("로봇이 이동할 수 없는 위치입니다.", target)
        #     move_group.stop()
        #     move_group.clear_pose_targets()
        #     current_pose = self.move_group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)
    
    def remove_object_box(self):
        self.scene.remove_world_object()
        remaining_objects = self.scene.get_objects()
        for marker_id in self.marker_ids:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.action = Marker.DELETE
            marker.id = marker_id
            self.marker_publisher.publish(marker)
        self.marker_ids = []  # 마커 ID 목록 초기화
        if not remaining_objects:
            print("모는 객체가 삭제되었습니다.")
        else:
            print("일부 객체가 아직 남아 있습니다.", remaining_objects)

    def attach_target(self, timeout = 2):
        for target_name in self.box_names["target"]:
            # print("self.box_names", self.box_names)
            # print("target_name", target_name)
            robot = self.robot
            scene = self.scene
            eef_link = self.eef_link
            planning_frame = self.planning_frame
            group_names = self.group_names
            # grasping_group = "indy7"
            grasping_group = "arm"
            touch_links = robot.get_link_names(group=grasping_group)
            # print("touch_links", touch_links)
            scene.attach_box(planning_frame, target_name, touch_links=touch_links)
            self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
        # return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
    
    def detach_target(self, timeout=2):
        for target_name in self.box_names["target"]:
            scene = self.scene
            planning_frame = self.planning_frame
            eef_link = self.eef_link
            scene.remove_attached_object(planning_frame, name = target_name)
            self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
# self.target_translation_nearest [{'translation': array([0.6015205 , 0.07177122, 0.95009507]), 'bounding_box_length': [0.07735606704993647, 0.05674455250803828, 0.0]}, 
#                                  {'translation': array([ 0.63225506, -0.14323654,  0.9523136 ]), 'bounding_box_length': [0.07595595723907789, 0.06150633313808346, 0.0]}]
    def go_to_object(self):
        quaternion = tf.transformations.quaternion_from_euler(0, pi/2, 0.0 )
        move_group = self.move_group
        move_group.set_start_state_to_current_state()
        pose_goal = geometry_msgs.msg.Pose()
        # pose_stamped = PoseStamped()
        # pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        # pose_goal.position.x = 0.5315205
        # pose_goal.position.y = 0.07177122
        # pose_goal.position.z = 0.95009507
        pose_goal.position.x = 0.56225506
        pose_goal.position.y = -0.14323654
        pose_goal.position.z = 0.9523136
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        print("1")
        move_group.set_pose_target(pose_goal)
        rospy.sleep(2)
        success, plan, _, _ = move_group.plan()
        rospy.sleep(2)
        move_group.execute(plan, wait = True)
        print("4")
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    #     waypoints = [pose_goal] # 카르테시안 경로를 위한 waypoints 설정
    #     (plan, fraction) = move_group.compute_cartesian_path(
    #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    # )  # jump_threshold
    #     move_group.execute(plan, wait=True)
    #     move_group.stop()
    #     move_group.clear_pose_targets()
    #     current_pose = self.move_group.get_current_pose().pose
    #     return all_close(pose_goal, current_pose, 0.01)
    
    def get_obstacle_translation(self):
        try:
            response = self.obstacle_client()
            return response.result_objects
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None

    def start_visual_servo(self):
        rospy.spin()

    def example_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "link0"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.37
        box_pose.pose.position.y = -0.04048
        box_pose.pose.position.z = 0.96025916  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.07, 0.07, 0.3))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
def main():
    try:
        obstacle_avoid = Obstacle()
        input("============ Press `Enter` to execute a saved path ...")
        obstacle_avoid.obstacle_homogeneous_matrix()
        # distance = obstacle_avoid.get_nearest_target(obstacle_avoid.target_translations)
        # print("distance", distance)
        # obstacle_avoid.wait_for_state_update()
        # obstacle_avoid.orientationconstraint()
        # print("obstacle_avoid.target_translation_nearest", obstacle_avoid.target_translation_nearest)
        # obstacle_avoid.go_to_object()
        if obstacle_avoid.detect_obj == True:
            print("객체를 인식했습니다.")
            obstacle_avoid.add_object_box()
            # obstacle_avoid.example_box()
            obstacle_avoid.attach_target()
            # print("obstacle_avoid.target_translation_nearest", obstacle_avoid.target_translation_nearest)
            # obstacle_avoid.orientationconstraint()
            obstacle_avoid.go_to_target()
            obstacle_avoid.detach_target()
            obstacle_avoid.remove_object_box()
        else:
            print("객체를 인식하지 못했습니다.")
        # if obstacle_avoid.detect_obj == True:
        #     obstacle_avoid.add_box()
        #     obstacle_avoid.attach_box()
        # else:
        #     print("객체를 인식하지 못했습니다.")
        # obstacle_avoid.go_to_object()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
if __name__ == "__main__":
    main()
    









# def go_to_pose_goal1(self):
#     quaternion = tf.transformations.quaternion_from_euler(0, pi / 1.5, 0.0)
#     move_group = self.move_group

#     pose_goal = geometry_msgs.msg.Pose()
#     pose_goal.position.x = 0.7
#     pose_goal.position.y = 0.0
#     pose_goal.position.z = 0.8
#     pose_goal.orientation.x = quaternion[0]
#     pose_goal.orientation.y = quaternion[1]
#     pose_goal.orientation.z = quaternion[2]
#     pose_goal.orientation.w = quaternion[3]

#     # 목표 포즈 설정
#     move_group.set_pose_target(pose_goal)

#     # 경로 계획
#     plan, _ = move_group.plan()

#     # 중간 단계의 트레젝토리 포인트
#     intermediate_steps = [plan.joint_trajectory.points[i] for i in range(0, len(plan.joint_trajectory.points), len(plan.joint_trajectory.points) // 3)]

#     for step in intermediate_steps:
#         joint_goal = step.positions
#         move_group.go(joint_goal, wait=True)

#     # 로봇을 중지하고 포즈 목표 지우기
#     move_group.stop()
#     move_group.clear_pose_targets()

#     current_pose = self.move_group.get_current_pose().pose
#     return all_close(pose_goal, current_pose, 0.01)






        # self.detect_obj = obstacle.detect_obj
        # obstacle_position = [(obstacle.transform0.translation.x),(obstacle.transform0.translation.y),(obstacle.transform0.translation.z)]
        # obstacle_quaternion = [
        #                 obstacle.transform0.rotation.x,
        #                 obstacle.transform0.rotation.y,
        #                 obstacle.transform0.rotation.z,
        #                 obstacle.transform0.rotation.w,
        #                 ]
        # obstacle_matrix = quaternion_matrix(obstacle_quaternion)
        # obstacle_matrix[:3, 3] = obstacle_position  ## camera to obstacle
        # base_to_camera = np.dot(current_pose, self.hand_eye_m)
        # base_to_obstacle = np.dot(base_to_camera, obstacle_matrix)

        # self.obstacle_translation = base_to_obstacle[:3, 3]
        # self.obstacle_rotation = base_to_obstacle[:3,:3]
        # self.obstacle_rotation = R.from_matrix(self.obstacle_rotation)
        # self.obstacle_rotation = self.obstacle_rotation.as_quat()
        # print("obstacle_translation", self.obstacle_translation)
        # print("obstacle_rotation", self.obstacle_rotation)
        # print(obstacle_matrix)





    # def add_box(self, timeout=4):
    #     bounding = self.get_bounding_box()
    #     print("bounding", bounding)
    #     bounding_position = [(bounding.transform1.translation.x),(bounding.transform1.translation.y),(bounding.transform1.translation.z)]
    #     bounding_quaternion = [
    #                     bounding.transform1.rotation.x,
    #                     bounding.transform1.rotation.y,
    #                     bounding.transform1.rotation.z,
    #                     bounding.transform1.rotation.w,
    #                     ]
    #     bounding_matrix = quaternion_matrix(bounding_quaternion)
    #     bounding_matrix[:3, 3] = bounding_position  ## camera to obstacle
    #     self.bounding_translation = bounding_matrix[:3, 3]
    #     self.bounding_rotation = bounding_matrix[:3,:3]
    #     self.bounding_rotation = R.from_matrix(self.bounding_rotation)
    #     self.bounding_rotation = self.bounding_rotation.as_quat()    

    #     box_name = self.box_name
    #     scene = self.scene
    #     box_pose = geometry_msgs.msg.PoseStamped()
    #     box_pose.header.frame_id = "world"
    #     box_pose.pose.orientation.w = self.obstacle_rotation[3]
    #     box_pose.pose.position.x = self.obstacle_translation[0]
    #     box_pose.pose.position.y = self.obstacle_translation[1]
    #     box_pose.pose.position.z = self.obstacle_translation[2]
    #     print("bounding_translation_x", self.bounding_translation[0])
    #     print("bounding_translation_y", self.bounding_translation[1])
    #     print("bounding_translation_x", self.obstacle_translation[0])
    #     print("bounding_translation_y", self.obstacle_translation[1])
    #     print("bounding_translation_z", self.obstacle_translation[2])

    #     box_name = "box"
    #     scene.add_box(box_name, box_pose, size=(0.085, self.bounding_translation[0], self.bounding_translation[1]))

    #     self.box_name = box_name
    #     return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    # def add_obstacle_box(self, timeout=4):
    #     if self.name == "orange":
    #         for target in self.target_translations:
    #             translation_target = target['translation']
    #             bounding_box_target = target['bounding_box_length']
    #             target_name = "target_" + str(self.target_name_counter)
    #             self.target_name_counter += 1
    #             scene_target = self.scene
    #             target_pose = geometry_msgs.msg.PoseStamped()
    #             target_pose.header.frame_id = "world"
    #             target_pose.pose.orientation.w = 1.0
    #             target_pose.pose.position.x = translation_target[0]
    #             target_pose.pose.position.y = translation_target[1]
    #             target_pose.pose.position.z = translation_target[2]
    #             scene_target.add_box(target_name, target_pose, size = (0.075, bounding_box_target[0], bounding_box_target[1]))
    #             self.target_name = target_name
    #             self.wait_for_state_update(box_is_known=True, timeout=timeout)
    #     if self.name == "bottle":
    #         for obstacle in self.obstacle_translations:
    #             print("self.obstacle_translations\n", self.obstacle_translations)
    #             print("obstacle\n", obstacle)
    #             translation_obstacle = obstacle['translation']
    #             bounding_box_obstacle = obstacle['bounding_box_length']
    #             # box_name = self.box_name
    #             obstacle_name = "obstacle_" + str(self.obstacle_name_counter)
    #             self.obstacle_name_counter += 1
    #             scene_obstacle = self.scene
    #             obstacle_pose = geometry_msgs.msg.PoseStamped()
    #             obstacle_pose.header.frame_id = "world"
    #             obstacle_pose.pose.orientation.w = 1.0
    #             obstacle_pose.pose.position.x = translation_obstacle[0]
    #             obstacle_pose.pose.position.y = translation_obstacle[1]
    #             obstacle_pose.pose.position.z = translation_obstacle[2]
    #             scene_obstacle.add_box(obstacle_name, obstacle_pose, size=(0.065, bounding_box_obstacle[0], bounding_box_obstacle[1]))
    #             # scene.add_box(box_name, obstacle_pose, size=(0.085, 0.5, 0.5))
    #             # self.obstacle_name = box_name
    #             self.box_name = obstacle_name
    #             self.wait_for_state_update(box_is_known=True, timeout=timeout)
    #     return self.wait_for_state_update(box_is_known=True, timeout=timeout)

        # def add_object_box(self, timeout=4):
    #     if self.name == "orange":
    #         print("orange")
    #         for target in self.target_translations:
    #             self.add_box(target['translation'], target['bounding_box_length'], "target_", 0.075, timeout)
    #             self.wait_for_state_update(box_is_known=True, timeout=timeout)
    #     elif self.name =="bottle":
    #         print("bottle")
    #         for obstacle in self.obstacle_translations:
    #             self.add_box(obstacle['translation'], obstacle['bounding_box_length'], "obstacle_", 0.065, timeout)
    #             self.wait_for_state_update(box_is_known=True, timeout=timeout)
    #     return self.wait_for_state_update(box_is_known= True, timeout=timeout)