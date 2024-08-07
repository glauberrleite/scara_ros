import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

from custom_interfaces.srv import GrabOrRelease

class ScaraSimul(Node):
    def __init__(self, robot_handle, joints_handles, end_effector_handle):
        super().__init__('scara_simul')
        self.current_goal = None
        self.current_joint_angles = None

        self.robot_handle = robot_handle
        self.joints_handles = joints_handles
        self.end_effector_handle = end_effector_handle
        
        self.subscription = self.create_subscription(Float64MultiArray, 'scara/target_positions', self.callback, 10)
        self.js_pub = self.create_publisher(JointState, 'scara/joint_states', 10)
        self.ee_pub = self.create_publisher(Pose, 'scara/pose', 10)

        self.grab_service = self.create_service(GrabOrRelease, 'scara/grab_release', self.grab_service_callback)
        self.grab_service

     
    def grab_service_callback(self, request, response):
        #fake gripping
        goal = sim.getObject('/goal_cube')

        if request.command == 0:
            endEffector = sim.getObject('./end_effector')

            #print('Parent\n', sim.getObjectParent(endEffector), sim.getObjectAlias(sim.getObjectParent(endEffector)), '\n')

            sim.setObjectInt32Param(goal, 3003, 1)
            sim.resetDynamicObject(goal)
            sim.setObjectParent(goal, endEffector, True)

        elif request.command == 1:
            sim.setObjectInt32Param(goal, 3003, 0)
            sim.resetDynamicObject(goal)
            sim.setObjectParent(goal, -1, True)

        return response

    def callback(self, msg):
        joint_pos = msg.data
        for i in range(4):
            sim.setJointTargetPosition(self.joints_handles[i], joint_pos[i])
    
    def publish_joints(self):
        # Filling the lists of joints variables
        joints_name = []
        joints_pos = []
        joints_vel = []
        for i in range(4):
            joints_name.append('joint'+str(i+1))
            joints_pos.append(sim.getJointPosition(self.joints_handles[i]))
            joints_vel.append(sim.getJointVelocity(self.joints_handles[i]))
        
        # Creating the JointState structure
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joints_name
        msg.position = joints_pos
        msg.velocity = joints_vel
        
        # Publishing message
        self.js_pub.publish(msg)
    
    def publish_pose(self):
        # Getting pose relative to the base
        # pose: pose array: [x y z qx qy qz qw]
        pose = sim.getObjectPose(self.end_effector_handle, self.robot_handle)
        
        # Creating the JointState structure
        msg = Pose()
        msg.position.x = pose[0]
        msg.position.y = pose[1]
        msg.position.z = pose[2]
        msg.orientation.x = pose[3]
        msg.orientation.y = pose[4]
        msg.orientation.z = pose[5]
        msg.orientation.w = pose[6]
        
        # Publishing message
        self.ee_pub.publish(msg)


def sysCall_init():
    sim = require('sim')

    joints_handle = []
    joints_handle.append(sim.getObject('./joint1'))
    joints_handle.append(sim.getObject('./joint2'))
    joints_handle.append(sim.getObject('./joint3'))
    joints_handle.append(sim.getObject('./joint4'))

    end_effector_handle = sim.getObject('./end_effector')

    robot_handle = sim.getObject('.')

    sim.setObjectInt32Param(sim.getObject('/goal_cube'), 3003, 0)

    rclpy.init()
    self.scara = ScaraSimul(robot_handle, joints_handle, end_effector_handle)

def sysCall_sensing():
    self.scara.publish_joints()
    self.scara.publish_pose()
    rclpy.spin_once(self.scara, timeout_sec=0)

def sysCall_cleanup():
    self.scara.destroy_node()
    rclpy.shutdown()
