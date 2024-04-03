import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge
import tf_transformations

from custom_interfaces.srv import ArucoDetect
import numpy as np
import cv2
from functools import partial

SIMULATION = True
Ts = 0.05

class VisionSupervisor(Node):
    def __init__(self):
        super().__init__('vision')
        
        self.distCoeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        self.cameraMatrix = np.array([[200, 0, 128],
                              [0, 200, 128],
                              [0,  0,  1]], dtype=np.float32)
        
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.image_sub = self.create_subscription(Image, 'camera/image', self.detect_aruco, 10) 
        self.image_publisher = self.create_publisher(Image, 'supervisor/image', 10)

        self.goal_publisher = self.create_publisher(Pose, 'supervisor/goal_pose', 10)
        
        self.aruco_service = self.create_service(ArucoDetect, 'aruco_detect', self.detect_aruco_service_callback)
        self.aruco_client = self.create_client(ArucoDetect, 'aruco_detect')
        while not self.aruco_client.wait_for_service(1.0):
            self.get_logger().warm("waiting for service...")

        self.timer = self.create_timer(Ts, self.control_loop)


    def detect_aruco(self, msg : Image):  
        self.call_aruco_service(msg)

    def control_loop(self):
        pass


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

        #self.get_logger().info(str(ids))
        
        if ids is not None:
            version = cv2.__version__
            if version[:3] == '4.9': #opencv == 4.9.x
                corners = np.array(corners)
                aruco_length = 0.1 
                obj_points = np.array([[0, 0, 0],
                            [0, aruco_length, 0],
                            [aruco_length, aruco_length, 0],
                            [aruco_length, 0, 0]], dtype=np.float32)
                
                
                ret, rvecs, tvecs = cv2.solvePnP(obj_points, corners[0], self.cameraMatrix, self.distCoeffs)
                
                
                pose.position.x = 0.603 + tvecs[1][0]
                pose.position.y = -0.167 + tvecs[0][0] #-0.217 -0.158
                pose.position.z = 0.745 - tvecs[2][0]
                #self.get_logger().info('Posição: ' + str(tvecs) + str(pose.position))
                
                rvecs = np.array(rvecs).reshape(1, -1)[0]
                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(rvecs)[0]#cv2.Rodrigues(np.array([0, 0, 0]).T)[0]#
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)
                #self.get_logger().info('Angulo: ' + str(rvecs))

                pose.orientation.x = round(quat[3], 6)
                pose.orientation.y = round(quat[2], 6)
                pose.orientation.z = round(quat[1], 6)
                pose.orientation.w = round(quat[0], 6)
                

                # Desenhar eixos 3D para cada marcador
                axis_length = 0.1 
                center = np.array([axis_length/2, axis_length/2, 0])
                p = np.array([[0, 0, 0],
                            [0, axis_length, 0],
                            [0, 0, axis_length],
                            [axis_length, 0, 0]], dtype=np.float32)
                
                p = p + center #Projetando o eixo sobre o centro
                img_points, _ = cv2.projectPoints(p, rvecs, tvecs, self.cameraMatrix, self.distCoeffs)
                img_points = np.int32(img_points).reshape(-1, 2)

                cv_image = cv2.flip(cv_image, -1)
                cv2.line(cv_image, tuple(img_points[0]), tuple(img_points[1]), (255, 0, 0), 3)  # X-axis (Red)
                cv2.line(cv_image, tuple(img_points[0]), tuple(img_points[3]), (0, 255, 0), 3)  # Y-axis (Green)
                cv2.line(cv_image, tuple(img_points[0]), tuple(img_points[2]), (0, 0, 255), 3)  # Z-axis (Blue)
                
                axis = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                resized = cv2.resize(axis, (516, 516), interpolation = cv2.INTER_AREA)
                
                #resized = cv2.flip(resized, -1)
                '''cv2.imshow("supervisor", resized)
                cv2.waitKey(1)'''

                self.goal_publisher.publish(pose)
            
            else: #opencv >= 4.7
                rvecs, tvecs, ret = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, self.cameraMatrix, self.distCoeffs)
                #self.get_logger().info('ARUCO encontrado ' + str(rvecs) + ' ' + str(tvecs))
                axis = cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist, (rvecs[0][0]), tvecs[0][0], 0.05)
                axis = cv2.cvtColor(axis, cv2.COLOR_BGR2RGB)
                resized = cv2.resize(axis, (516, 516), interpolation = cv2.INTER_AREA)
                '''cv2.imshow("supervisor", resized)
                cv2.waitKey(1)'''
        else:
            resized = cv2.resize(cv_image, (516, 516), interpolation = cv2.INTER_AREA)
            resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

            resized = cv2.flip(resized, -1)
        
        '''cv2.imshow("supervisor", resized)
        cv2.waitKey(1)'''

        p_img = self.bridge.cv2_to_imgmsg(resized, "bgr8")
        self.image_publisher.publish(p_img)

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