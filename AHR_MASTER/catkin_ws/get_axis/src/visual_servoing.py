# #!/usr/bin/env python3
# from __future__ import print_function
# from six.moves import input
# import time
# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# import datetime
# from pathlib import Path
# from typing import List
# from tf.transformations import *
# from typing import Iterable, Tuple
# import numpy as np
# from cv_bridge import CvBridge
# import cv2
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Transform, Vector3, Quaternion
# from aruco_marker_detect.srv import MatrixCalculation
# import tf
# from scipy.spatial.transform import Rotation as R
# from aruco_marker_detect.msg import VisibleTransform
# from aruco_marker_detect.srv import MatrixCalculation, MatrixCalculationResponse
# import pyrealsense2 as rs
# try:
#     from math import pi, tau, dist, fabs, cos
# except:  # For Python 2 compatibility
#     from math import pi, fabs, cos, sqrt
#     tau = 2.0 * pi
#     def dist(p, q):
#         return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list


# class Visp(object):

#     def __init__(self):
#         super(Visp, self).__init__()
#         rospy.init_node("visual_servoing", anonymous=True)
#         self.aruco_client = rospy.ServiceProxy('matrix_calculation', MatrixCalculation)

#     def get_aruco_transform(self):
#         try:
#             response = self.aruco_client()
#             return response.transform0, response.transform1, response.transform2
#         except rospy.ServiceException as e:
#             rospy.logerr("Service call failed: %s" % e)
#             return None, None, None
        

#     def start_visual_servo(self):
#         rospy.spin()

# def main():
#     try:
#         visual_servo = Visp()
        
#         transform0, transform1, transform2 = visual_servo.get_aruco_transform()
#         print("Aruco transform0: \n%s" % transform0)
#         print("Aruco transform1: \n%s" % transform1)
#         print("Aruco transform2: \n%s" % transform2)
#     except rospy.ROSInterruptException:
#         return
#     except KeyboardInterrupt:
#         return
# if __name__ == "__main__":
#     main()




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
from get_axis.srv import MatrixCalculation
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

        self.hand_eye_m = np.array([[-0.0248467,   0.99906731, -0.03531499,-0.09401433],
                                    [-0.99962673, -0.02523101, -0.01047868, 0.02838461],
                                    [-0.01135994,  0.03504144,  0.99932129,  0.05443796],
                                    [0,            0,           0 ,           1]])
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

    def base_to_object(self):
        current_pose = self.move_group.get_current_pose().pose
        # print("joint:",current_joint)
        print("pose:", current_pose)

        position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
        quaternion = [
                        current_pose.orientation.x,
                        current_pose.orientation.y,
                        current_pose.orientation.z,
                        current_pose.orientation.w,
                        ]
        self.matrix = quaternion_matrix(quaternion)
        self.matrix[:3, 3] = position ## robot pose
        transform = self.get_aruco_transform()
        print("Aruco transform: \n%s" % transform)
        marker_position = [(transform.translation.x), transform.translation.y, transform.translation.z]
        marker_quaternion = [
                        transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z,
                        transform.rotation.w,
                        ]
        marker_matrix = quaternion_matrix(marker_quaternion)
        marker_matrix[:3, 3] = marker_position  ## camera to aruco
        reversal_x =np.array([[1,0,0,0],
                            [0,-1,0,0],
                            [0,0,-1,0],
                            [0,0,0,1]])
        reversal_Y =np.array([[0.996717,0,-0.07846,0],
                            [0,1,0,0],
                            [0.07846,0,0.996717,0],
                            [0,0,0,1]])
        # reversal_marker = np.dot(marker_matrix, reversal_x)
        self.base_camera = np.dot(self.matrix,self.hand_eye_m) ## base to camera
        # self.base_marker = np.dot(self.base_camera, reversal_marker)
        self.base_marker = np.dot(self.base_camera, self.base_camera)
        self.t = self.base_marker[:3, 3]
        r = self.base_marker[:3, :3]
        r = R.from_matrix(r)
        self.q = r.as_quat()
        print("base_to_object_tttt",self.t )
        print("self.q", self.q)
    
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

