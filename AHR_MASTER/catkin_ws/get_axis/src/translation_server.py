# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import logging
# from geometry_msgs.msg import Point
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from geometry_msgs.msg import Transform, Vector3, Quaternion
# from get_axis.srv import MatrixCalculation, MatrixCalculationResponse, MatrixCalculationRequest
# from get_axis.srv import Boundingbox, BoundingboxResponse
# from get_axis.msg import ObjectDetect
# from get_axis.msg import BoundingBox
# from get_axis.msg import Yolodetect
# # from get_axis.srv import MatrixCalculation
# class Depth():
#     def __init__(self):
#         rospy.init_node('center_point_subscriber', anonymous=True)
#         self.bridge = CvBridge()
#         self.camera_matrix = np.array([[599.9529418945312, 0.0, 326.3251037597656],
#                                         [0.0, 599.775634765625, 246.8442840576172],
#                                          [ 0.0, 0.0, 1.0]])
#         self.dist_coeffs = np.array([0.1281878501176834, -0.43850672245025635, -0.0010290791979059577, -0.0008640193846076727, 0.39977091550827026])
#         self.last_detection_time = rospy.Time.now()
#         rospy.Timer(rospy.Duration(0.05), self.check_detection)
#         # self.camera_matrix = np.array([[606.2945556640625, 0.0, 327.8588562011719],
#         #                                 [0.0, 606.2423095703125, 243.08094787597656],
#         #                                  [ 0.0, 0.0, 1.0]])
#         # self.dist_coeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
#         self.fx = self.camera_matrix[0, 0]
#         self.fy = self.camera_matrix[1, 1]
#         self.cx = self.camera_matrix[0, 2]
#         self.cy = self.camera_matrix[1, 2]

#         self.center_point = Point()
#         self.color_image = None
#         self.depth_image = None
#         self.object_detected = False
#         self.detected_objects = []
#         self.image_received = False

#     def check_detection(self, event):
#         if rospy.Time.now() - self.last_detection_time > rospy.Duration(2.0):  # 마지막 감지 이후 2초가 지났을 경우
#             self.reset_values()

#     def detect_callback(self, data):
#         if self.image_received:
#             self.last_detection_time = rospy.Time.now()
#             self.object_point = data
#             self.detected_objects.append(self.object_point)
#             if len(self.detected_objects) == self.object_point.num:
#                 self.process_data()
#                 self.detected_objects = [] # 리스트 비우기

#     def depth_callback(self, data):
#         self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
#         self.image_received = True

#     def process_data(self):
#             try:
#                 result_objects = []
#                 for object_point in self.detected_objects:
#                     self.transform_obj = ObjectDetect()
#                     x, y = int(object_point.center.x), int(object_point.center.y)
#                     b_1_x, b_1_y = int(object_point.length_1.x), int(object_point.length_1.y)
#                     b_2_x, b_2_y = int(object_point.length_2.x), int(object_point.length_2.y)
#                     depth = self.depth_image[y,x]
#                     depth = depth/1000
#                     b_1_x = (b_1_x-self.cx)*depth/self.fx # 바운딩 박스 모서리 translation 좌표
#                     b_1_y = (b_1_y-self.cy)*depth/self.fy
#                     b_2_x = (b_2_x-self.cx)*depth/self.fx
#                     b_2_y = (b_2_y-self.cy)*depth/self.fy
#                     self.transform_obj.center_point = Transform(translation=Vector3((x - self.cx) * depth / self.fx, (y - self.cy) * depth / self.fy, depth), rotation = Quaternion(x=0,y=0,z=0,w=1)) # 바운딩 박스 center translation 좌표
#                     self.transform_obj.bounding_box_length = Transform(translation=Vector3((b_2_x - b_1_x), (b_2_y - b_1_y), 0), rotation = Quaternion(x=0,y=0,z=0,w=1))
#                     self.transform_obj.detect_obj = True
#                     self.transform_obj.name = object_point.name
#                     result_objects.append(self.transform_obj)
#                     # if len(self.objects_data.result_objects) >= self.object_point.num:
#                     #     self.objects_data.result_objects = []
#                 self.objects_data = MatrixCalculationResponse()
#                 self.objects_data.result_objects = result_objects[:self.object_point.num]  # 리스트 슬라이싱을 통해 필요한 길이만큼 자르기
#             except KeyboardInterrupt:
#                 return False

#     def reset_values(self):
#         fake_objects = []
#         self.transform_obj = ObjectDetect()
#         print("감지못함")
#         self.X = 0
#         self.Y = 0
#         self.Z = 0
#         self.b_x = 0
#         self.b_y = 0
#         self.b_z = 0
#         self.detected_objects = []
#         self.transform_obj.center_point = Transform(translation=Vector3(self.X, self.Y, self.Z), rotation = Quaternion(x=0,y=0,z=0,w=1))
#         self.transform_obj.detect_obj = False
#         self.transform_obj.bounding_box_length = Transform(translation=Vector3(self.b_x, self.b_y, self.b_z), rotation = Quaternion(x=0,y=0,z=0,w=1))
#         self.transform_obj.name = ""
#         fake_objects.append(self.transform_obj)
#         self.objects_data = MatrixCalculationResponse()
#         self.objects_data.result_objects = fake_objects

#     def obstacle_translation(self, req):
#         try:
#             print("서비스가 요청되었습니다.")
#         except KeyboardInterrupt:
#             return
#         return MatrixCalculationResponse(result_objects=self.objects_data.result_objects)
    
#     def get_rgb_depth(self):
#         rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
#         rospy.Subscriber("object_detect", Yolodetect, self.detect_callback)
#         s = rospy.Service('obstacle_translation', MatrixCalculation, self.obstacle_translation)
#         # a = rospy.Service('bounding_box', Boundingbox, self.bounding_box)
#         rospy.spin()

# def main():
#     depth = Depth()
#     depth.get_rgb_depth()
#     try:
#         rospy.spin()  # Keep the program running until it's manually terminated
#     except KeyboardInterrupt:
#         print("Streaming stopped")
#         cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()






import pyrealsense2 as rs
import numpy as np
import cv2
import logging
from geometry_msgs.msg import Point
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform, Vector3, Quaternion
from get_axis.srv import MatrixCalculation, MatrixCalculationResponse, MatrixCalculationRequest
from get_axis.srv import Boundingbox, BoundingboxResponse
from get_axis.msg import ObjectDetect
from get_axis.msg import BoundingBox
from get_axis.msg import Yolodetect
# from get_axis.srv import MatrixCalculation
class Depth():
    def __init__(self):
        rospy.init_node('center_point_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.camera_matrix = np.array([[599.9529418945312, 0.0, 326.3251037597656],
                                        [0.0, 599.775634765625, 246.8442840576172],
                                         [ 0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([0.1281878501176834, -0.43850672245025635, -0.0010290791979059577, -0.0008640193846076727, 0.39977091550827026])
        self.last_detection_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.05), self.check_detection)
        # self.camera_matrix = np.array([[606.2945556640625, 0.0, 327.8588562011719],
        #                                 [0.0, 606.2423095703125, 243.08094787597656],
        #                                  [ 0.0, 0.0, 1.0]])
        # self.dist_coeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        self.center_point = Point()
        self.color_image = None
        self.depth_image = None
        self.object_detected = False
        self.detected_objects = []
        self.image_received = False

    def check_detection(self, event):
        if rospy.Time.now() - self.last_detection_time > rospy.Duration(2.0):  # 마지막 감지 이후 2초가 지났을 경우
            self.reset_values()

    def detect_callback(self, data):
        if self.image_received:
            self.last_detection_time = rospy.Time.now()
            self.object_point = data
            self.detected_objects.append(self.object_point)
            if len(self.detected_objects) == self.object_point.num:
                self.process_data()
                self.detected_objects = [] # 리스트 비우기

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.image_received = True

    def calculate_distance(self, center_point):
        x = center_point.translation.x
        y = center_point.translation.y
        z = center_point.translation.z
        return np.linalg.norm([x, y, z])
    
    def process_data(self):
            try:
                result_objects = []
                for object_point in self.detected_objects:
                    self.transform_obj = ObjectDetect()
                    x, y = int(object_point.center.x), int(object_point.center.y)
                    b_1_x, b_1_y = int(object_point.length_1.x), int(object_point.length_1.y)
                    b_2_x, b_2_y = int(object_point.length_2.x), int(object_point.length_2.y)
                    depth = self.depth_image[y,x]
                    depth = depth/1000
                    b_1_x = (b_1_x-self.cx)*depth/self.fx # 바운딩 박스 모서리 translation 좌표
                    b_1_y = (b_1_y-self.cy)*depth/self.fy
                    b_2_x = (b_2_x-self.cx)*depth/self.fx
                    b_2_y = (b_2_y-self.cy)*depth/self.fy
                    self.transform_obj.center_point = Transform(translation=Vector3((x - self.cx) * depth / self.fx, (y - self.cy) * depth / self.fy, depth), rotation = Quaternion(x=0,y=0,z=0,w=1)) # 바운딩 박스 center translation 좌표
                    self.transform_obj.bounding_box_length = Transform(translation=Vector3((b_2_x - b_1_x), (b_2_y - b_1_y), 0), rotation = Quaternion(x=0,y=0,z=0,w=1))
                    self.transform_obj.detect_obj = True
                    self.transform_obj.name = object_point.name
                    result_objects.append(self.transform_obj)
                    # if len(self.objects_data.result_objects) >= self.object_point.num:
                    #     self.objects_data.result_objects = []
                # print("result_objects", result_objects.center_point)
                # self.objects_data = MatrixCalculationResponse()
                if len(result_objects) > 0:  # 객체가 하나 이상 감지된 경우에만
                    closest_object = min(result_objects, key=lambda obj: self.calculate_distance(obj.center_point))

                    # 가장 가까운 객체만 서비스로 보내기
                    self.objects_data = MatrixCalculationResponse()
                    self.objects_data.result_objects = [closest_object]


                # # 가장 가까운 객체 찾기
                # closest_object = min(result_objects, key=lambda obj: self.calculate_distance(result_objects.center_point))
                # # 가장 가까운 객체만 서비스로 보내기
                # self.objects_data = MatrixCalculationResponse()
                # self.objects_data.result_objects = [closest_object]
                # # self.objects_data.result_objects = result_objects[:self.object_point.num]  # 리스트 슬라이싱을 통해 필요한 길이만큼 자르기
            except KeyboardInterrupt:
                return False

    def reset_values(self):
        fake_objects = []
        self.transform_obj = ObjectDetect()
        print("감지못함")
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.b_x = 0
        self.b_y = 0
        self.b_z = 0
        self.detected_objects = []
        self.transform_obj.center_point = Transform(translation=Vector3(self.X, self.Y, self.Z), rotation = Quaternion(x=0,y=0,z=0,w=1))
        self.transform_obj.detect_obj = False
        self.transform_obj.bounding_box_length = Transform(translation=Vector3(self.b_x, self.b_y, self.b_z), rotation = Quaternion(x=0,y=0,z=0,w=1))
        self.transform_obj.name = ""
        fake_objects.append(self.transform_obj)
        self.objects_data = MatrixCalculationResponse()
        self.objects_data.result_objects = fake_objects

    def obstacle_translation(self, req):
        try:
            print("서비스가 요청되었습니다.")
        except KeyboardInterrupt:
            return
        return MatrixCalculationResponse(result_objects=self.objects_data.result_objects)
    
    def get_rgb_depth(self):
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("object_detect", Yolodetect, self.detect_callback)
        s = rospy.Service('obstacle_translation', MatrixCalculation, self.obstacle_translation)
        # a = rospy.Service('bounding_box', Boundingbox, self.bounding_box)
        rospy.spin()

def main():
    depth = Depth()
    depth.get_rgb_depth()
    try:
        rospy.spin()  # Keep the program running until it's manually terminated
    except KeyboardInterrupt:
        print("Streaming stopped")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

