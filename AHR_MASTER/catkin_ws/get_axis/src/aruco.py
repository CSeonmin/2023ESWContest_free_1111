# from cv_bridge import CvBridge
# import cv2
# import rospy
# import cv2.aruco as aruco
# import numpy as np
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Transform, Vector3, Quaternion
# from aruco_marker_detect.msg import ThreeTransforms
# from aruco_marker_detect.msg import VisibleTransform
# from aruco_marker_detect.srv import MatrixCalculation, MatrixCalculationResponse
# import tf
# from collections import deque

# class Aruco(object):
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.MARKERLEN = 0.105
#         # self.MARKERLEN = 0.026
#         self.DICT_GET = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
#         self.ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()
#         # self.camera_matrix = np.array([[903.77980799,   0.  ,       637.8486699 ], ## d435i
#         # [  0.  ,       906.89833226 ,374.06423905],
#         # [  0.  ,         0.    ,      1.        ]])
#         # self.dist_coeffs = np.array([ 4.73274074e-02,  1.50717030e-01, -8.29310966e-04, -1.63552205e-03,-8.35299486e-01])

#         self.camera_matrix = np.array([[899.929443359375, 0.0, 649.4876708984375],
#                                         [0.0, 899.6634521484375, 370.26641845703125],
#                                          [ 0.0, 0.0, 1.0]])
#         self.dist_coeffs = np.array([0.1281878501176834, -0.43850672245025635, -0.0010290791979059577, -0.0008640193846076727, 0.39977091550827026])
#         # camera_matrix = np.array([[902.10681158,   0.  ,       643.07506331,],
#         #  [  0.   ,      904.84610834, 367.66952022],
#         #  [  0.   ,        0.   ,        1.        ],])
        
#         # self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

#         # self.transform_publisher = rospy.Publisher("/aruco_transform", Transform, queue_size=10)
#         self.transform_deque = deque(maxlen=3)
#         self.transform_dict = {}
#         self.current_image = None

#     def matrix_to_transform(self, transform_mat):
#         translation_vector = transform_mat[:3,3]
#         rotation_matrix = transform_mat[:3, :3]
#         quaternion = tf.transformations.quaternion_from_matrix(transform_mat)
#         return translation_vector, quaternion
#     def image_callback(self, msg):

#         # self.transform_dict = {}
#         self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         cv2_img = self.current_image
#         markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv2_img, self.DICT_GET, parameters=self.ARUCO_PARAMETERS)
#         self.marker_visibility = {i: False for i in range(3)}
#         img_with_markers = cv2_img.copy()
#         self.image_with_axis = img_with_markers
#         if len(markerCorners) > 0:
#             img_with_markers = cv2.aruco.drawDetectedMarkers(cv2_img.copy(), markerCorners, markerIds)
#             # Estimate the pose of each marker
#             rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, self.MARKERLEN, self.camera_matrix, self.dist_coeffs)
#             for i in range(len(markerIds)):
#                 self.image_with_axis = aruco.drawAxis(img_with_markers, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
#                 # Convert the rotation vector to a rotation matrix
#                 rmat, _ = cv2.Rodrigues(rvecs[i])
#                 # Form the 4x4 transformation matrix
#                 transform_mat = np.zeros(shape=(4, 4))
#                 transform_mat[:3, :3] = rmat
#                 transform_mat[:3, 3] = tvecs[i].reshape(-1)
#                 transform_mat[3, 3] = 1
#                 # self.transform_deque.append(transform_mat)
#                 # self.transform_deque[markerIds[i][0]] = transform_mat
#                 # print("Marker ID:", markerIds[i][0])
#                 # print("Transformation Matrix for ID {}:\n".format(markerIds[i][0]), transform_mat)
#                 self.transform_dict[markerIds[i][0]] = transform_mat
#                 self.marker_visibility[markerIds[i][0]] = True
#         print("self.marker_visibility", self.marker_visibility[0])
#         cv2.imshow('Image Stream', self.image_with_axis)
#         cv2.waitKey(1)

#     def handle_matrix_calculation(self, req):


#         try:
#             print("service start")

#             # create default (empty) transforms
#             transform0 = VisibleTransform()
#             transform1 = VisibleTransform()
#             transform2 = VisibleTransform()

#             # get transform data
#             data0 = self.transform_dict.get(0)
#             data1 = self.transform_dict.get(1)
#             data2 = self.transform_dict.get(2)

#             if self.marker_visibility[0]:
#                 translation_vector0, quaternion0 = self.matrix_to_transform(data0)
#                 transform0.transform = Transform(
#                     translation=Vector3(*translation_vector0),
#                     rotation=Quaternion(*quaternion0)
#                 )
#                 transform0.is_visible = True
#             else:
#                 transform0.transform = Transform(
#                     translation=Vector3(0, 0, 0),
#                     rotation=Quaternion(0, 0, 0, 1)  # Identity quaternion, represents no rotation
#                 )
#                 transform0.is_visible = False

#             if self.marker_visibility[1]:
#                 translation_vector1, quaternion1 = self.matrix_to_transform(data1)
#                 transform1.transform = Transform(
#                     translation=Vector3(*translation_vector1),
#                     rotation=Quaternion(*quaternion1)
#                 )
#                 transform1.is_visible = True
#             else:
#                 transform1.transform = Transform(
#                     translation=Vector3(0, 0, 0),
#                     rotation=Quaternion(0, 0, 0, 1)  # Identity quaternion, represents no rotation
#                 )
#                 transform1.is_visible = False

#             if self.marker_visibility[2]:
#                 translation_vector2, quaternion2 = self.matrix_to_transform(data2)
#                 transform2.transform = Transform(
#                     translation=Vector3(*translation_vector2),
#                     rotation=Quaternion(*quaternion2)
#                 )
#                 transform2.is_visible = True
#             else:
#                 transform2.transform = Transform(
#                     translation=Vector3(0, 0, 0),
#                     rotation=Quaternion(0, 0, 0, 1)  # Identity quaternion, represents no rotation
#                 )
#                 transform2.is_visible = False

#             transform = MatrixCalculationResponse(transform0, transform1, transform2)                  
#         except KeyboardInterrupt:
#             return
#         return transform
    
#     def matrix_calculation_server(self):
#         rospy.init_node('matrix_calculation_server')
#         rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
#         s = rospy.Service('matrix_calculation', MatrixCalculation, self.handle_matrix_calculation)
#         print("Ready to calculate matrix.")
#         rospy.spin()


#     # else:
#     #     # Save your OpenCV2 image as a jpeg 
        

# def main():
#     # 노드 초기화. 이름은 listener
#     aruco = Aruco()
#     aruco.matrix_calculation_server()
# if __name__ == "__main__":
#     main()



from cv_bridge import CvBridge
import cv2
import rospy
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform, Vector3, Quaternion
from get_axis.srv import MatrixCalculation, MatrixCalculationResponse
import tf
from collections import deque

class Aruco(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.MARKERLEN = 0.076
        # self.MARKERLEN = 0.026
        self.DICT_GET = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[912.205810546875, 0.0, 641.8896484375],
        [  0.0, 912.2283935546875, 372.99859619140625],
        [  0.  ,         0.    ,      1.        ]])
        # self.camera_matrix = np.array([[912.205810546875,   0.  ,       641.8896484375 ],
        # [  0.  ,       912.2283935546875 , 372.99859619140625],
        # [  0.  ,         0.    ,      1.        ]])

        # camera_matrix = np.array([[902.10681158,   0.  ,       643.07506331,],
        #  [  0.   ,      904.84610834, 367.66952022],
        #  [  0.   ,        0.   ,        1.        ],])
        # self.dist_coeffs = np.array([ 4.73274074e-02,  1.50717030e-01, -8.29310966e-04, -1.63552205e-03,-8.35299486e-01])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # self.transform_publisher = rospy.Publisher("/aruco_transform", Transform, queue_size=10)
        self.transform_deque = deque(maxlen=1)
        self.current_image = None
    def matrix_to_transform(self, transform_mat):
        translation_vector = transform_mat[:3,3]
        rotation_matrix = transform_mat[:3, :3]
        quaternion = tf.transformations.quaternion_from_matrix(transform_mat)
        return translation_vector, quaternion
    def image_callback(self, msg):


        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = self.current_image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv2_img, self.DICT_GET, parameters=self.ARUCO_PARAMETERS)
        img_with_markers = cv2_img.copy()
        image_with_axis = img_with_markers
        if len(markerCorners) > 0:
            img_with_markers = cv2.aruco.drawDetectedMarkers(cv2_img.copy(), markerCorners, markerIds)
            # Estimate the pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, self.MARKERLEN, self.camera_matrix, self.dist_coeffs)
            for i in range(len(markerIds)):
                self.image_with_axis = aruco.drawAxis(img_with_markers, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                # Convert the rotation vector to a rotation matrix
                rmat, _ = cv2.Rodrigues(rvecs[i])
                # Form the 4x4 transformation matrix
                transform_mat = np.zeros(shape=(4, 4))
                transform_mat[:3, :3] = rmat
                transform_mat[:3, 3] = tvecs[i].reshape(-1)
                transform_mat[3, 3] = 1
                self.transform_deque.append(transform_mat)
                self.avg_transform_mat = np.mean(np.array(self.transform_deque), axis=0)
                print("Average Transformation matrix:\n", self.avg_transform_mat)
                translation_vector, quaternion = self.matrix_to_transform(self.avg_transform_mat)
                self.transform_msg = Transform(
                    translation=Vector3(*translation_vector),
                    rotation=Quaternion(*quaternion)
                )
        cv2.imshow('Image Stream', self.image_with_axis)
        cv2.waitKey(1)

    def handle_matrix_calculation(self, req):
        try:
            print("Average Transformation matrix:\n", self.avg_transform_mat)
            self.transform_deque.clear()
        except KeyboardInterrupt:
            return
        return MatrixCalculationResponse(self.transform_msg)
    
    def matrix_calculation_server(self):
        rospy.init_node('matrix_calculation_server')
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        s = rospy.Service('matrix_calculation', MatrixCalculation, self.handle_matrix_calculation)
        print("Ready to calculate matrix.")
        rospy.spin()


    # else:
    #     # Save your OpenCV2 image as a jpeg 
        

def main():
    # 노드 초기화. 이름은 listener
    aruco = Aruco()
    aruco.matrix_calculation_server()
    # rospy.init_node('listen', anonymous=True)

    # # 토픽 callback이라는 이름의 함수로 받아들이며, 메시지 타입은 test_msg
    # rospy.Subscriber("/camera/color/image_raw", Image, aruco.image_callback)
    # rospy.spin()
if __name__ == "__main__":
    main()