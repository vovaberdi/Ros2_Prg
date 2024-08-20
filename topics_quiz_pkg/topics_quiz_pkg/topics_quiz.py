import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class TopicsQuizNode(Node):

    def __init__(self):
        super().__init__('topics_quiz_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer_period = 0.1  # Update timer period for finer control
        self.laser_forward = 0
        self.laser_left = 0
        self.cmd = Twist()
        self.state = 'FOLLOW_WALL'
        self.turning_duration = 1.0  # Time to complete a 90-degree turn
        self.forward_duration = 2.0  # Time to move forward after turning
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]  # Save the frontal laser scan info at 0°
        self.laser_left = msg.ranges[89]  # Save the left laser scan info at 90°

    def motion(self):
        current_time = self.get_clock().now()
        
        if self.state == 'FOLLOW_WALL':
            if self.laser_left < 0.5:  # Wall is detected on the left
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.0
            else:  # No wall detected, start turning
                self.state = 'TURN_LEFT'
                self.start_time = current_time
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.5
        
        elif self.state == 'TURN_LEFT':
            if (current_time - self.start_time).nanoseconds * 1e-9 < self.turning_duration:
                # Continue turning
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.5
            else:
                # Finish turning, start moving forward
                self.state = 'MOVE_FORWARD'
                self.start_time = current_time
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.0
        
        elif self.state == 'MOVE_FORWARD':
            if (current_time - self.start_time).nanoseconds * 1e-9 < self.forward_duration:
                # Continue moving forward
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.0
            else:
                # Stop after moving forward for the duration
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0

        # Publish the Twist message to the 'cmd_vel' topic
        self.publisher_.publish(self.cmd)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    topics_quiz_node = TopicsQuizNode()
    rclpy.spin(topics_quiz_node)
    topics_quiz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
