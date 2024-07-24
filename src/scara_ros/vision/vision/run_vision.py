import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from custom_interfaces.srv import ArucoDetect

import numpy as np
import cv2
from functools import partial
from cv_bridge import CvBridge
import tf_transformations


SIMULATION = True
Ts = 0.5

class VisionSupervisor(Node):
    def __init__(self):
        super().__init__('vision')
        
        self.distCoeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        self.cameraMatrix = np.array([[200, 0, 150],
                              [0, 200, 150],
                              [0,  0,  1]], dtype=np.float32)
        
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.image_sub = self.create_subscription(Image, 'camera/image', self.detect_aruco, 10) 
        self.image_publisher = self.create_publisher(Image, 'supervisor/image', 10)

        qos_profile = QoSProfile(
            depth=2,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.goal_publisher = self.create_publisher(Pose, 'supervisor/goal_pose', qos_profile)
        
        self.aruco_service = self.create_service(ArucoDetect, 'aruco_detect', self.detect_aruco_service_callback)
        
        self.aruco_client = self.create_client(ArucoDetect, 'aruco_detect')
        while not self.aruco_client.wait_for_service(1.0):
            self.get_logger().warm("waiting for service...")

        self.image = None
        self.timer = self.create_timer(Ts, self.control_loop)


    def detect_aruco(self, msg : Image):  
        self.image = msg

    def control_loop(self):
        if self.image is not None:
            self.call_aruco_service(self.image)

    def detect_aruco_service_callback(self, request, response):
        msg = request.image

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except CvBridgeError as e:
            print(e)
            return
        
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0


        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        gray = np.flip(gray, 0)
        gray = np.flip(gray, 1)
       

        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)

        if ids is not None:
            version = cv2.__version__
            v = version.split('.')
            #print(version.split('.'))
            if v[0] == '4' and int(v[1]) >= 9: #opencv >= 4.9.x
                corners = np.array(corners)
                aruco_length = 0.1 
                
                obj_points = np.array([
                            [-aruco_length / 2, -aruco_length / 2, 0],
                            [-aruco_length / 2, aruco_length / 2, 0],
                            [aruco_length / 2, aruco_length / 2, 0],
                            [aruco_length / 2, -aruco_length / 2, 0]
                            
                            ], dtype=np.float32)
                
                
                ret, rvecs, tvecs = cv2.solvePnP(obj_points, corners[0], self.cameraMatrix, self.distCoeffs)
                
                
                pose.position.x = 0.69 - tvecs[1][0] # 0.603
                pose.position.y = -0.217 + tvecs[0][0] #-0.217 -0.158
                pose.position.z = -0.0749 + tvecs[2][0]#0.745 - tvecs[2][0]
                #self.get_logger().info('Posição: ' + str(tvecs) + str(pose.position))
                
                rvecs = np.array(rvecs).reshape(1, -1)[0]
                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(rvecs)[0]#cv2.Rodrigues(np.array([0, 0, 0]).T)[0]#
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)
                #self.get_logger().info('Angulo: ' + str(quat))

                pose.orientation.x = round(quat[3], 6)
                pose.orientation.y = round(quat[2], 6)
                pose.orientation.z = round(quat[1], 6)
                pose.orientation.w = round(quat[0], 6)

                self.goal_publisher.publish(pose)
            
            else: #opencv < 4.7
                pose.position.x = np.inf
                pose.position.y = np.inf
                self.goal_publisher.publish(pose)
        else:
            resized = cv2.resize(cv_image, (516, 516), interpolation = cv2.INTER_AREA)
            resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

            resized = cv2.flip(resized, -1)
            pose.position.x = np.inf
            pose.position.y = np.inf
            self.goal_publisher.publish(pose)

        response.pose = pose
        return response

    def call_aruco_service(self, msg):
        
        request = ArucoDetect.Request()
        request.image = msg

        future = self.aruco_client.call_async(request)
        future.add_done_callback(partial(self.callback_aruco))
    
    def callback_aruco(self, future):
        try:
            response = future.result()
            #self.get_logger().info("Success " + str(response.pose.position))
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)

    supervisor = VisionSupervisor()

    rclpy.spin(supervisor)

    supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()