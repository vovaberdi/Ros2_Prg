import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point32, Twist  
from nav_msgs.msg import Odometry
from custom_interfaces.action import OdomRecord

import time
import numpy as np

class OdomRecordServer(Node):

    def __init__(self):
        super().__init__('record_odom_server')
        
        self.group1 = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self, OdomRecord, 'record_odom', self.action_callback, callback_group=self.group1)

        self.odom_subscriber_ = self.create_subscription(
            Odometry, '/odom', self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE),
            callback_group=self.group1)
        
        self.vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # Publisher for stopping the robot

        self.current_point = Point32()
        self.start_point = Point32()
        self.start_point_identified = False
        self.total_distance = 0.0
        self.odom_record = []
        self.lap_finished = False
        self.check_finished_lap_flag = False

    def action_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = OdomRecord.Feedback()
        
        while not self.start_point_identified:
            time.sleep(1)

        self.odom_record.append(self.start_point)    
        N = 1

        while not self.lap_finished:
            time.sleep(1)

            self.get_logger().info('Distance to start point: "%f"' % self.distance_to_start)
            self.odom_record.append(self.current_point)
            N += 1
            delta_x = self.odom_record[N-1].x - self.odom_record[N-2].x
            delta_y = self.odom_record[N-1].y - self.odom_record[N-2].y

            dist_to_last_point = np.sqrt(np.power(delta_x, 2) + np.power(delta_y, 2))   
            self.get_logger().info('Distance to last point: "%f"' % dist_to_last_point)

            self.total_distance += dist_to_last_point         
            self.get_logger().info('Current total distance feedback: "%f"' % self.total_distance)
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            
            if N > 5:
                self.check_finished_lap_flag = True
            
        self.stop_robot()  # Stop the robot when the lap is finished

        goal_handle.succeed()
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        self.get_logger().info('The Action Server has finished, it has recorded: "%s" points' % str(len(self.odom_record)))
        return result
    
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not self.start_point_identified:
            self.start_point.x = x
            self.start_point.y = y
            self.get_logger().info('Start point: ("%f", "%f")' %(self.start_point.x, self.start_point.y))
            self.start_point_identified = True
        else:
            self.current_point = Point32()
            self.current_point.x = x
            self.current_point.y = y
            self.check_finished_lap()

    def check_finished_lap(self):
        self.distance_to_start = np.sqrt(np.power(self.current_point.x - self.start_point.x, 2) + np.power(self.current_point.y - self.start_point.y, 2))
        if self.check_finished_lap_flag and self.distance_to_start <= 0.1:  # Threshold for considering the lap complete
            self.lap_finished = True

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.vel_publisher_.publish(stop_msg)
        self.get_logger().info('Robot has been stopped.')
            
def main(args=None):
    rclpy.init(args=args)
    odom_recorder_node = OdomRecordServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(odom_recorder_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        odom_recorder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

