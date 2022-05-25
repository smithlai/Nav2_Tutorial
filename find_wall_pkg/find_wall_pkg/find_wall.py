# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessage module from custom_interfaces_service interface
# from custom_interfaces.srv import FindWall
# import the Empty module from std_servs service interface
from std_srvs.srv import Empty
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time
import math
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class FindwallService(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('findwall_server')
        # create the service server object
        # defines the type, name and callback function
        self.group1 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()
        self.group2 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()  # MutuallyExclusiveCallbackGroup()

        # ros2 service call /find_wall std_srvs/srv/Empty
        self.srv = self.create_service(
            Empty, 'find_wall', self.CustomService_callback, callback_group=self.group1)
        
        # create the publisher object
        # in this case the publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(
            Twist, 'cmd_vel', 10, callback_group=self.group2)
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.update_scan, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group3)
        self.laser_f = 0
        self.MAX_ROTATE_SPEED = 1.0
        self.BUFFER = 3



# smith@ubuntu:~$ ros2 topic type /scan
# sensor_msgs/msg/LaserScan

# smith@ubuntu:~$ ros2 interface show sensor_msgs/msg/LaserScan
# # Single scan from a planar laser range-finder
# #
# # If you have another ranging device with different behavior (e.g. a sonar
# # array), please find or create a different message, since applications
# # will make fairly laser-specific assumptions about this data

# std_msgs/Header header # timestamp in the header is the acquisition time of
#         builtin_interfaces/Time stamp
#                 int32 sec
#                 uint32 nanosec
#         string frame_id
#                              # the first ray in the scan.
#                              #
#                              # in frame frame_id, angles are measured around
#                              # the positive Z axis (counterclockwise, if Z is up)
#                              # with zero angle being forward along the x axis

# float32 angle_min            # start angle of the scan [rad]
# float32 angle_max            # end angle of the scan [rad]
# float32 angle_increment      # angular distance between measurements [rad]

# float32 time_increment       # time between measurements [seconds] - if your scanner
#                              # is moving, this will be used in interpolating position
#                              # of 3d points
# float32 scan_time            # time between scans [seconds]

# float32 range_min            # minimum range value [m]
# float32 range_max            # maximum range value [m]

# float32[] ranges             # range data [m]
#                              # (Note: values < range_min or > range_max should be discarded)


# len(ranges)=
# angle_increment=0.052799832075834274 # 3 degree
# angle_min=-3.141590118408203
# angle_max=3.141590118408203



    def update_scan(self, msg):  # LaserScan
        self.get_logger().info(f'XXXXXX0 {msg.ranges[180//3]}, {msg.ranges[90//3]}, {msg.ranges[270//3]}, {msg.ranges[0]}')
        self.laser_f = msg.ranges[60]

    def getMinIndex(self, msg):
        min_idx = min_range = -1
        for i in msg.ranges:
            r = msg.ranges[i]
            if r == math.inf:
                continue
            if r < min_range:
                min_range = r
                min_idx = i
        return min_idx

    def rotate(self, msg, target=179):
        buffer = self.BUFFER
        while True:
            mi = self.getMinIndex(msg)
            if mi < 0:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            elif mi+360 < target-buffer + 360:
                msg.linear.x = 0.0
                msg.angular.z = -self.MAX_ROTATE_SPEED*(target-mi)/180
            elif mi+360 > target+1+buffer + 360:
                msg.linear.x = 0.0
                msg.angular.z = self.MAX_ROTATE_SPEED*(mi-target)/180
            else:  # mi == target +- buffer
                break
            self.publisher_.publish(msg)
            time.sleep(1)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def gotowall(self, msg):
        while True:
            self.get_logger().info(f'self.laser_f={self.laser_f}')
            if self.laser_f == math.inf:
                msg.linear.x = 0.5
            elif self.laser_f <= 3:
                break
            else:
                msg.linear.x = 0.3

            self.publisher_.publish(msg)
            time.sleep(1)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def CustomService_callback(self, request, response):
        # The callback function recives the self class parameter,
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as response
        # create a Twist message
        msg = Twist()
        # self.rotate(msg, target=179)
        self.gotowall(msg)
        # self.rotate(msg, target=269)
        # response state
        # response.wallfound = True
        # response.message = "Found wall"

        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # # declare the node constructor
    # service = FindwallService()
    # # pause the program execution, waits for a request to kill the node (ctrl+c)
    # rclpy.spin(service)
    # # shutdown the ROS communication
    # rclpy.shutdown()
    try:
        # declare the node constructor
        findwall_service = FindwallService()

        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(findwall_service)

        try:
            # pause the program execution, waits for a request to kill the node (ctrl+c)
            executor.spin()
        finally:
            executor.shutdown()
            findwall_service.destroy_node()

    finally:
        # shutdown the ROS communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()
