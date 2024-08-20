import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.srv import FindWall
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.action import OdomRecord
from rclpy.action import ActionClient

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.group1 = ReentrantCallbackGroup()
        
        self.wall_finder_client = self.create_client(FindWall, 
            'find_wall', 
            callback_group=self.group1)

        while not self.wall_finder_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.find_wall_req = FindWall.Request()
        self.find_wall_response = None
       
        self.find_wall_started = False
        self.find_wall_completed = False
        self.wall_follower_can_start = False

        # Action Client: Creates an action client for the OdomRecord action
        self.odom_recorder_client = ActionClient(self, 
            OdomRecord, 
            'record_odom', 
            callback_group=self.group1)
        self.action_called = False

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.sensor_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group1)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.control_timer = self.create_timer(0.5, self.command_publisher, callback_group=self.group1)

        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        self.left_side = 0.0
        self.front = 0.0
        self.right_side = 0.0

        self.min_distance = 0.42
        self.side_min= 0.27
        self.side_max= 0.38


        self.target_distance = 4.0 # Stop the robot after traveling x meter
        self.distance_traveled = 0.0

    def sensor_callback(self, msg: LaserScan):
        self.right_side = msg.ranges[230]  #230
        self.front = msg.ranges[360]       #360
        self.left_side = msg.ranges[490]   #490
        self.get_logger().info(f'Right: {self.right_side}, Front: {self.front}, Left: {self.left_side}')

    def send_request(self):
        self.find_wall_response = self.wall_finder_client.call(self.find_wall_req)
        if self.find_wall_response is not None:
            self.get_logger().info(f'Service response: Wall found = {self.find_wall_response.wallfound}')
            self.find_wall_completed = self.find_wall_response.wallfound
        else:
            self.get_logger().error('Service call failed')
            self.find_wall_completed = False

    def action_send_goal(self):
        goal_msg = OdomRecord.Goal()

        self.odom_recorder_client.wait_for_server()
        self._send_goal_future = self.odom_recorder_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        list_of_odoms = result.list_of_odoms
        self.get_logger().info('The Action Server has finished, it has recorded: "%s" points' % str(len(list_of_odoms)))
        self.get_logger().info('Points: "%s"' % str(list_of_odoms))
        self.action_finished = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.distance_traveled = feedback.current_total
        self.get_logger().info('Total distance feedback "%s"' % str(self.distance_traveled)) 

        if self.distance_traveled >= self.target_distance:
            self.get_logger().info('5 meter traveled, stopping the robot...')
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info('Robot has been stopped.')
        self.control_timer.cancel()  # Stop the timer to stop further commands

    def simple_motion_correction(self):
        angular_z = 0.0  # Initialize angular velocity

        if self.left_side < self.side_threshold:
            angular_z = -self.angular_vel  # Turn right to correct
            self.get_logger().info('Too close to the wall on the left, turning right.')
        elif self.right_side < self.side_threshold:
            angular_z = self.angular_vel  # Turn left to correct
            self.get_logger().info('Too close to the wall on the right, turning left.')
        else:
            angular_z = 0.0  # Move straight if within acceptable limits
            self.get_logger().info('Moving straight, well-centered.')

        return angular_z


    def command_publisher(self):
        if self.distance_traveled >= self.target_distance:
            return  # Stop issuing movement commands if the robot has already traveled 1 meter

        linear_vel = 0.075
        angular_vel = 0.15

        if not self.find_wall_started:
            self.find_wall_started = True
            self.get_logger().info('SERVICE HAS STARTED')
            self.send_request()
            return

        if not self.find_wall_completed:
            self.get_logger().info('Waiting for service to complete...')
            return

        if self.find_wall_completed:
            if not self.action_called:
                self.get_logger().info('ACTION HAS BEGAN')
                self.action_called = True
                self.action_send_goal()

            if self.right_side > self.side_max and self.front > self.min_distance:
                self.cmd.linear.x = linear_vel * 0.9
                self.cmd.angular.z = -angular_vel
                self.get_logger().info('Too close to the wall on the left, turning right.')
            elif self.right_side < self.side_min and self.front > self.min_distance:
                self.cmd.linear.x = linear_vel * 0.9
                self.cmd.angular.z = angular_vel
                self.get_logger().info('Too close to the wall on the right, turning left.')
            elif self.front > self.min_distance:
                self.cmd.linear.x = linear_vel
                self.cmd.angular.z = 0.0
                self.get_logger().info('Moving forward.')
            elif self.front < self.min_distance:
                if self.left_side >= self.right_side:
                    self.cmd.linear.x = linear_vel * 0.25
                    self.cmd.angular.z = angular_vel * 2
                    self.get_logger().info('Obstacle ahead, turning left.')
                else:
                    self.cmd.linear.x = linear_vel * 0.25
                    self.cmd.angular.z = -angular_vel * 2
                    self.get_logger().info('Obstacle ahead, turning right.')
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.get_logger().info('Waiting for wall to be found...')

            self.cmd_pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollower()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(wall_follower_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        wall_follower_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

