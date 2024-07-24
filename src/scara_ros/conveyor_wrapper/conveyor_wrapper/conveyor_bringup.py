import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64
from custom_interfaces.action import VelControl

import time
import numpy as np

CurrentVel = 0.0

class ConveyorListener(Node):
    def __init__(self):
        super().__init__('conveyor_listener')
        self.group = MutuallyExclusiveCallbackGroup()
        self.subscriber = self.create_subscription(Float64, 'conveyor/vel', self.callback, 10, callback_group=self.group)
    
    def callback(self, msg):
        global CurrentVel
        CurrentVel = msg.data
        #self.get_logger().info("Valor recebido: " + str(CurrentVel))

class ConveyorControl(Node):
    def __init__(self):
        super().__init__('conveyor_controller')

        self.publisher = self.create_publisher(Float64, 'conveyor/target_vel', 10)

        self.group = ReentrantCallbackGroup()

        self._current_goal = None
        self._vel_control_action = ActionServer(self, VelControl, 'conveyor/vel_controller',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.vel_control_action_callback,
            callback_group=self.group)

    def vel_control_action_callback(self, goal_handle):
        global CurrentVel

        self.get_logger().info('Executing goal...')

        target_vel = goal_handle.request.vel

        self.get_logger().info('Velocidade desejada:' + str(target_vel))

        msg = Float64()
        msg.data = float(target_vel)
        
        self.publisher.publish(msg)

        feedback_msg = VelControl.Feedback()
        feedback_msg.current_vel = CurrentVel

        while np.abs(target_vel - CurrentVel) > 0.001 and not goal_handle.is_cancel_requested:
            if goal_handle.is_cancel_requested:
                goal_handle.abort()
                self.get_logger().info('\nGoal canceled\n')
                return VelControl.Result()
            
            self.get_logger().info('Feedback: ' + str(round(CurrentVel, 6)))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
            feedback_msg.current_vel = CurrentVel

        self.get_logger().info('Feedback: ' + str(round(CurrentVel, 6)))
        goal_handle.publish_feedback(feedback_msg)
        feedback_msg.current_vel = CurrentVel

        result = VelControl.Result()
        if round(CurrentVel, 5) == 0:
            result.status = True
        else:
            result.status = False

        self._current_goal = None

        goal_handle.succeed()
        return result
    
    def handle_accepted_callback(self, goal_handle):
        if self._current_goal is not None:
            self._current_goal.abort()
            self.get_logger().warn('\nAção atual cancelada, nova encaminhada.\n')
        self._current_goal = goal_handle
        self._current_goal.execute()
    
def main(args=None):
    rclpy.init(args=args)

    conveyorListener = ConveyorListener()
    conveyorController = ConveyorControl()

    executor = MultiThreadedExecutor()
    executor.add_node(conveyorListener)
    executor.add_node(conveyorController)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        conveyorListener.destroy_node()
        conveyorController.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()