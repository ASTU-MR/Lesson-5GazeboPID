import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose

from tf_transformations import euler_from_quaternion

import math                                                                    

def angle_diff(a1, a2):
    a = a1-a2
    return (a+math.pi)%(2*math.pi)-math.pi

def get_yaw_from_quaternion(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class RurMover(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_twist = self.create_publisher(Twist, '/waterstrider/twist_command', 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/waterstrider/ground_truth_to_tf_waterstrider/pose', self.pose_callback, 10)
        self.desired_heading = - math.pi / 2
        self.SIDE_TIME = 10.0
        self.LINEAR_SPEED = 1.0
        self.timer = self.create_timer(self.SIDE_TIME, self.pub_timer_callback)

    def pose_callback(self, msg):
        heading = get_yaw_from_quaternion(msg.pose.orientation)
        self.get_logger().info('Current heading: "%f"' % heading)
        self.get_logger().info('Setpoint: "%f"' % self.desired_heading)
        msg_pub = Twist()
        msg_pub.linear.x = self.LINEAR_SPEED
        error = angle_diff(self.desired_heading, heading)
        if error < 0:
            msg_pub.angular.z = math.radians(90.0)
            self.get_logger().info('Turn left')
        else:
            msg_pub.angular.z = math.radians(-90.0)
            self.get_logger().info('Turn right')
        self.publisher_twist.publish(msg_pub)

    def pub_timer_callback(self):
        self.desired_heading += math.pi / 2
        self.desired_heading %= 2 * math.pi
        self.get_logger().info('New angle setpoint: "%f"' % self.desired_heading)

def main(args=None):
    rclpy.init(args=args)

    mover = RurMover()

    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()