import rclpy

# This is a workaround so that Webots' Python controller classes can be used
# in this case, we need it to import MotionLibrary which needs the Motion class
import os
from ament_index_python.packages import get_package_prefix
os.environ['WEBOTS_HOME'] = get_package_prefix('webots_ros2_driver')
from utils.motion_library import MotionLibrary

# imu topic message lib
from rclpy.node import Node
from sensor_msgs.msg import Imu

class NaoDriver:
    def init(self, webots_node, properties):
        # we get the robot instance from the webots_node
        self.__robot = webots_node.robot
        # to load all the motions from the motion folder, we use the Motion_library class:
        self.__library = MotionLibrary()

        # we initialize the shoulder pitch motors using the Robot.getDevice() function:
        self.__RShoulderPitch = self.__robot.getDevice("RShoulderPitch")
        self.__LShoulderPitch = self.__robot.getDevice("LShoulderPitch")

        # to control a motor, we use the setPosition() function:
        self.__RShoulderPitch.setPosition(1.3)
        self.__LShoulderPitch.setPosition(1.3)
        # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor
        # to see the list of available devices, see the NAO documentation: https://cyberbotics.com/doc/guide/nao

        # adding a custom motion
        self.__library.add('Shove','/home/ram/projects/cyberbots_ws/src/rcbot_cyberbotics_controller/controllers/motions/Shove.motion',loop=True)

        # we add a variable to wait for the robot to stabilise
        self.wait = True

        rclpy.init(args=None)
        self.__node = rclpy.create_node('nao_driver')
        self.__library.play('Stand')

        #subscribing to imu topic 
        self.__imu_sub =self.__node.create_subscription(Imu,'IMU',self.imuCallback,10)
        self.__imu_sub # prevent unused warniings
        self.__fall_detect=False
    
    def imuCallback(self,msg): # use to detect fall
         print("acceleration x : ",msg.linear_acceleration.x)
         

    def step(self):
        # Mandatory function to go to the next simulation step
        rclpy.spin_once(self.__node, timeout_sec=0)
        if self.__library.get('Stand').isOver():
                self.__library.play('ForwardLoop')  # walk forward
                self.__library.play('Shove')     