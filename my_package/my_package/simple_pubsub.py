import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

import serial
import numpy as np
import re
import time
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

COM_PORT = '/dev/ttyACM0'    # 指定通訊埠名稱
BAUD_RATES = 9600    # 設定傳輸速率

class Simple_pubsub(Node):



    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('simple_publisher')
        self.group1 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()
        self.group2 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()

        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.group1)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group2)
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 1
        # define the variable to save the received info
        self.laser_forward = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.testall)


        self.ser = serial.Serial(COM_PORT, BAUD_RATES)   # 初始化序列通訊埠
        self.timecount = 0
    def __end__(self):
        self.ser.close()    # 清除序列通訊物件

    def move_turtlebot(self,msg):
        self.get_logger().info('receive scan')
        
        pass
    def testall(self):
        self.timecount += 1
        if self.timecount >= 10:
            self.writeArduino()
            self.timecount=0
        else:
            self.readArduino()

    def readArduino(self):
        self.get_logger().info('1')
        while self.ser.in_waiting:          # 若收到序列資料…
            data_raw = self.ser.readline()  # 讀取一行
            data = data_raw.decode()   # 用預設的UTF-8解碼
            data2=data.strip()
            self.get_logger().info('get arduino: "%s"' % str(data2))
        self.get_logger().info('2')
    def writeArduino(self):
        self.get_logger().info('writing to arduino')
        self.ser.write(f"5\n".encode())

    # def motion(self):
    #     # print the data
    #     self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
    #     # Logic of move
    #     if self.laser_forward > 5:
    #         self.cmd.linear.x = 0.5
    #         self.cmd.angular.z = 0.5
    #     elif self.laser_forward <5 and self.laser_forward>=0.5:
    #         self.cmd.linear.x = 0.2
    #         self.cmd.angular.z = 0.0
         
            
            
    #     else:
    #         self.cmd.linear.x = 0.0
    #         self.cmd.angular.z = 0.0
    #     # Publishing the cmd_vel values to topipc
    #     self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)


    # # declare the node constructor
    # simple_pubsub = Simple_pubsub()       
    # # pause the program execution, waits for a request to kill the node (ctrl+c)
    # rclpy.spin(simple_pubsub)
    # # Explicity destroy the node
    
    # simple_pubsub.destroy_node()
    # # shutdown the ROS communication
    # rclpy.shutdown()
    try:
        # declare the node constructor
        simple_pubsub = Simple_pubsub()   

        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(simple_pubsub)

        try:
            # pause the program execution, waits for a request to kill the node (ctrl+c)
            executor.spin()
        finally:
            executor.shutdown()
            simple_pubsub.destroy_node()

    finally:
        # shutdown the ROS communication
        rclpy.shutdown()

if __name__ == '__main__':
    main()