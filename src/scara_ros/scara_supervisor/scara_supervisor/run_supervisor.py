import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

from custom_interfaces.action import VelControl
from custom_interfaces.action import ScaraControl
from custom_interfaces.srv import GrabOrRelease

import numpy as np
import time
import asyncio
from abc import ABC, abstractmethod
from scipy.spatial.transform import Rotation

Ts = 0.05
GoalPose = None
TargetJoint = None
CurrentJointState = None
lock = False

def inverse_kinematic(x, y):
    return

class SupervisorListener(Node):
    def __init__(self):
        super().__init__('supervisor_listener')
        self.group = MutuallyExclusiveCallbackGroup()

        qos_profile = QoSProfile(
            depth=2,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.conv_sub = self.create_subscription(Float64, 'conveyor/vel', self.get_velocity, qos_profile)
        self.velocity = None

        self.goal_subscriber = self.create_subscription(Pose, 'supervisor/goal_pose', self.goal_callback, qos_profile, callback_group=self.group)
        self.joint_state = self.create_subscription(JointState, 'scara/joint_states', self.get_joint_state_callback, qos_profile, callback_group=self.group)
        
    def goal_callback(self, msg):
        global GoalPose

        #Vericando se a simulação está ativa
        if self.velocity is None:
            #self.get_logger().info('Pose invalida. A simulação está pausada.\n')
            GoalPose = None     #descartando pose defasada
        else:
            #Aruco não detectado
            #self.get_logger().info('Pose valida\n')
            GoalPose = msg
            if(GoalPose.position.x == np.inf and GoalPose.position.y == np.inf):
                    GoalPose = None

        self.velocity = None
    
    def get_joint_state_callback(self, msg):
        global CurrentJointState
        CurrentJointState = msg.position
    
    def get_velocity(self, msg):
        self.velocity = msg
  
class SupervisorActionServer(Node):
    def __init__(self):
        super().__init__('supervisor_action_server')
        self.current_goal = None
        self.group = MutuallyExclusiveCallbackGroup()

        self.publisher = self.create_publisher(Float64MultiArray, 'scara/target_positions', 10)
        self.moveJoint = ActionServer(self, ScaraControl, 'scara/joint_control', execute_callback=self.jointControlExecute, handle_accepted_callback=self.handle_accepted_callback, callback_group=self.group)

    def jointControlExecute(self, goal_handle):
        global CurrentJointState

        target_angles = goal_handle.request.angles.data

        #envio da posição desejada
        self.publisher.publish(goal_handle.request.angles)

        #acompanhamento da ação
        last_dist = np.array([1000.0, 1000.0, 1000.0, 1000.0])
        alpha = np.array([1000.0, 1000.0, 1000.0, 1000.0])

        while((alpha[0] > 0.0 or alpha[1] > 0.0 or alpha[3] > 0.0 or alpha[2] > 0.0) and not goal_handle.is_cancel_requested):
            time.sleep(0.4) #await self._sleep_async(0.2)

            #Verificando se a pose desejada foi alcançada
            dist = np.array(CurrentJointState) - np.array(target_angles)
            alpha = np.abs((last_dist - dist)).copy()
            last_dist = dist.copy()

            #envio de feedback 
            feedback_msg = ScaraControl.Feedback()
            msg = Float64MultiArray()
            msg.data = [(alpha[0]), alpha[1], alpha[2], alpha[3]]
            feedback_msg.angle_shift = msg
            goal_handle.publish_feedback(feedback_msg)

        #Finalizando a ação
        result = ScaraControl.Result()
        result.status = True
        self.current_goal = None

        goal_handle.succeed()
        return result
    
    '''async def _sleep_async(self, seconds):
        await asyncio.sleep(seconds)'''
    
    def handle_accepted_callback(self, goal_handle):
        if self.current_goal is not None:
            #Cancelando ação atual para execução de uma nova
            self.current_goal.abort()
            self.get_logger().warn('\nMovimento atual cancelado, novo destino.')
            self.get_logger().info(str(self.current_goal.request.angles.data)+'\n'+str(goal_handle.request.angles.data))
       
        #Nova ação requisitada
        self.current_goal = goal_handle
        self.current_goal.execute()
       
class Command(ABC):
    @abstractmethod
    def execute(self):
        pass
    def transition(self):
        return False

class initial(Command):
    def __init__(self, node):
        self.node = node

        self._action_client = ActionClient(node, VelControl, 'conveyor/vel_controller')
        while not self._action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action service...")

    def execute(self):
        global GoalPose
        if GoalPose is not None:
            x = GoalPose.position.x
            y = GoalPose.position.y

            if y > -0.3:
                #solicitando a parada da esteira
                goal_msg = VelControl.Goal()
                goal_msg.vel = 0.0

                future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                future.add_done_callback(self.get_result_callback)

                self.node.get_logger().info("Pronto pra coleta")
                return True
            else:
                #self.node.get_logger().info("Pose encontrada, parada não solicitada")
                return False
        else:
            #self.node.get_logger().info("Pose não encontrada")
            return False
    
    def get_result_callback(self, future):
        result = future.result().status
        self.node.get_logger().info('\n Parada solicitada: ' + str(result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.current_vel
        #self.node.get_logger().info("Feedback: " + str(feedback)) 
        global lock 
        if feedback < 0.001:
            lock = True
            self.node.get_logger().info("Parada finalizada.") 
    
class MoveToGoal(Command):
    def __init__(self, node):
        self.node = node
        self.action_client = ActionClient(node, ScaraControl, 'scara/joint_control')
        
        while not self.action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action server...")

    def execute(self):
        self.node.get_logger().info("\nMoveToGoal ")

        global GoalPose
        global TargetJoint

        if GoalPose is not None:
            
            x = GoalPose.position.x
            y = GoalPose.position.y

            #calculando angulos
            s = inverse_kinematic(x, y)
            msg = Float64MultiArray()
            
            #altura
            z = 0.355 - GoalPose.position.z 

            #Transformação de coordenadas
            quart = [GoalPose.orientation.x, GoalPose.orientation.y, GoalPose.orientation.z, GoalPose.orientation.w]
            re = Rotation.from_quat(quart)
            euler = re.as_euler('xyz', degrees=False) 
            w = euler[2] - np.pi * 50 /180

            self.node.get_logger().info("\n\n Angulos definidos:" + str(s) + " " + str(z) + " " + str(w))

            msg.data = [s[0], s[1], 0.0, w]     #pose acima do cubo
            TargetJoint = [s[0], s[1], z, w]    #pose para coleta

            #Envio da pose
            goal_msg = ScaraControl.Goal()
            goal_msg.angles = msg
            future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.movimentFeedback)
            future.add_done_callback(self.acceptGoal)
            return True
        else:
            return False
        
    def movimentFeedback(self, feedback_msg):
        #Acompanhando a execução da ação
        feedback = feedback_msg.feedback.angle_shift.data
        global lock
        #self.node.get_logger().info("Feedback: " + str(feedback))
        if (feedback[0] == 0.0 and feedback[1] == 0.0 and feedback[3] == 0.0):
            lock = True
            self.node.get_logger().info("Movimento finalizado.")
    
    def acceptGoal(self, future):
        result = future.result().status
        self.node.get_logger().info('\n Movimento iniciado ->' + str(result))

class GrabGoal(Command):
    def __init__(self, node):
        self.node = node

        self.action_client = ActionClient(node, ScaraControl, 'scara/joint_control')
        
        while not self.action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action server...")
    
    def execute(self):
        global TargetJoint
        if TargetJoint is not None:
            #descida
            self.node.get_logger().info("Grab goal\n")
            msg = Float64MultiArray()
            msg.data = TargetJoint
            self.node.get_logger().info("Posição buscada:" + str(TargetJoint))
            
            goal_msg = ScaraControl.Goal()
            goal_msg.angles = msg
            future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.movimentFeedback)
            future.add_done_callback(self.acceptGoal)
            
            return True
        else:
            return False
    
    def movimentFeedback(self, feedback_msg):
        feedback = feedback_msg.feedback.angle_shift.data
        #self.node.get_logger().info("Feedback: " + str(feedback))
        global lock
        if (feedback[2] == 0.0):#< 0.0000001
            lock = True
            self.node.get_logger().info("Descida finalizada.")
    
    def acceptGoal(self, future):
        result = future.result().status
        self.node.get_logger().info('\n Descida iniciada ->' + str(result))
           
class RecoilGoal(Command):
    def __init__(self, node):
        self.node = node

        self.grab_client = self.node.create_client(GrabOrRelease, 'scara/grab_release')

        while not self.grab_client.wait_for_service(1.0):
            self.node.get_logger().info("waiting for service GrabOrRelease...")

        self.action_client = ActionClient(node, ScaraControl, 'scara/joint_control')
        
        while not self.action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action server...")
    
    def execute(self):
        global TargetJoint
        if TargetJoint is not None:
            #agarrar
            self.node.get_logger().info("Grab request\n")
            req = GrabOrRelease.Request()
            req.command = int(0) # 0 agarrar, 1 para soltar
            self.grab_client.call_async(req)

            #recuo
            t = TargetJoint.copy()
            t[2] = 0.0      #Mover somente a junta prismática
            msg = Float64MultiArray()
            msg.data = t

            goal_msg = ScaraControl.Goal()
            goal_msg.angles = msg
            future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.movimentFeedback)
            future.add_done_callback(self.acceptGoal)

            TargetJoint = None
            
            return True
        else:
            return False
    
    def movimentFeedback(self, feedback_msg):
        feedback = feedback_msg.feedback.angle_shift.data
        #self.node.get_logger().info("Feedback: " + str(feedback))
        global lock
        if (feedback[2] == 0.0):#< 0.0000001
            lock = True
            self.node.get_logger().info("Subida finalizada.")
    
    def acceptGoal(self, future):
        result = future.result().status
        self.node.get_logger().info('Subida iniciada -> ' + str(result))
        
class MoveToBox(Command):
    def __init__(self, node):
        self.node = node

        self.grab_client = self.node.create_client(GrabOrRelease, 'scara/grab_release')

        while not self.grab_client.wait_for_service(1.0):
            self.node.get_logger().info("waiting for service GrabOrRelease...")

        self.action_client = ActionClient(node, ScaraControl, 'scara/joint_control')
        
        while not self.action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action server...")
    
    def execute(self):
        self.node.get_logger().info("MoveToBox\n")
        #posição de deposito
        x = 0.046       
        y = 0.557
        #calculo dos angulos
        s = inverse_kinematic(x, y)
        
        msg = Float64MultiArray()

        msg.data = [s[0], s[1], 0.0, -np.pi * 50 /180]

        #solicitando movimento
        goal_msg = ScaraControl.Goal()
        goal_msg.angles = msg
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.movimentFeedback)
        future.add_done_callback(self.acceptGoal)
        
        return True
    
    def movimentFeedback(self, feedback_msg):
        feedback = feedback_msg.feedback.angle_shift.data
        global lock
        #self.node.get_logger().info("Feedback: " + str(feedback))
        if (feedback[0] == 0.0 and feedback[1] == 0.0 and feedback[3] == 0.0):
            lock = True
            self.node.get_logger().info("Transporte finalizado.")
    
    def acceptGoal(self, future):
        result = future.result().status
        self.node.get_logger().info('Transporte iniciado -> ' + str(result))

class DropTarget(Command):
    def __init__(self, node):
        self.node = node

        self.action_client = ActionClient(node, ScaraControl, 'scara/joint_control')
        
        while not self.action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action server...")
    
    def execute(self):
        self.node.get_logger().info("Drop\n")
        #posição de deposito
        x = 0.046
        y = 0.557
        #calculo dos angulos
        s = inverse_kinematic(x, y)
        
        msg = Float64MultiArray()

        #drop
        msg.data = [s[0], s[1], 0.2, -np.pi * 50 / 180]     #Pose

        #envio
        goal_msg = ScaraControl.Goal()
        goal_msg.angles = msg
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.movimentFeedback)
        future.add_done_callback(self.acceptGoal)
        
        return True
    
    def movimentFeedback(self, feedback_msg):
        feedback = feedback_msg.feedback.angle_shift.data
        global lock
        if (feedback[2] == 0.0):#< 0.0000001
            lock = True
    
    def acceptGoal(self, future):
        result = future.result().status
        self.node.get_logger().info('Deposito iniciado -> ' + str(result))

class ReleaseTarget(Command):
    def __init__(self, node):
        self.node = node

        self.grab_client = self.node.create_client(GrabOrRelease, 'scara/grab_release')

        while not self.grab_client.wait_for_service(1.0):
            self.node.get_logger().info("waiting for service GrabOrRelease...")
    
    def execute(self):
        time.sleep(0.5)
        #soltar cubo
        self.node.get_logger().info("Release request")
        req = GrabOrRelease.Request()
        req.command = int(1)
        self.grab_client.call_async(req)
        self.node.get_logger().info("Deposito finalizado.")
        
        return True

class EndProcess(Command):
    def __init__(self, node):
        self.node = node

        self.action_client = ActionClient(node, ScaraControl, 'scara/joint_control')
        
        while not self.action_client.wait_for_server(1.0):
            node.get_logger().info("waiting for action server...")
    
    def execute(self):
        self.node.get_logger().info("End")
        time.sleep(0.5)
        msg = Float64MultiArray()

        #posição inicial
        msg.data = [0.0, 0.0, 0.0, 0.0]

        #Envio
        goal_msg = ScaraControl.Goal()
        goal_msg.angles = msg
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.movimentFeedback)
        future.add_done_callback(self.acceptGoal)
        return True
    
    def movimentFeedback(self, feedback_msg):
        feedback = feedback_msg.feedback.angle_shift.data
        global lock
        #self.node.get_logger().info("Feedback: " + str(feedback))
        if (feedback[0] == 0.0 and feedback[1] == 0.0 and feedback[3] == 0.0):
            lock = True
            self.node.get_logger().info("Retorno finalizado.")
            self.node.get_logger().info("Reinicio")
    
    def acceptGoal(self, future):
        result = future.result().status
        self.node.get_logger().info('Retorno iniciado -> ' + str(result))

#Estado de transição     
class Waiting(Command):
    def __init__(self, node):
        self.node = node
        self.node.get_logger().info("Esperando açao ser concluida")

    def execute(self):
        #Esperando ação anterior terminar
        global lock
        time.sleep(0.5)
        if lock:
            lock = False
            return True
        else:
            #self.node.get_logger().info("Mudança estado liberada:"+ str(lock) + ". Esperando..." )
            return False

class Context():
    def __init__(self, node):
        self.node = node
        self.index = -1
        self.state = initial(node)
        self.states = [Waiting, MoveToGoal, Waiting, GrabGoal, Waiting, RecoilGoal, Waiting, MoveToBox, Waiting,
                        DropTarget, Waiting, ReleaseTarget, EndProcess, Waiting, initial]
        
        self.node.get_logger().info("Buscando cubo\n")

    def request(self):
        if(self.state.execute()):
            self.change_state()

    def change_state(self):
        self.node.get_logger().info("Troca de estado\n")
        self.index += 1
        state = self.states[self.index % len(self.states)]
        self.state = state(self.node)

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        
        self.current_context = Context(self)
        
        self.timer = self.create_timer(Ts, self.control_loop)

    def control_loop(self):
        self.current_context.request()

def main(args=None):
    rclpy.init(args=args)

    listener = SupervisorListener()
    supervisor = Supervisor()
    server = SupervisorActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(supervisor)
    executor.add_node(server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        listener.destroy_node()
        supervisor.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
