# 12 16 20 21

from gpiozero import Motor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from time import sleep

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Motor 1
        self.motor1 = Motor(forward=12, backward=16)

        # Motor 2
        self.motor2 = Motor(forward=20, backward=21)

        # LIDAR subscription
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # replace with the actual topic name
            self.lidar_callback,
            10
        )

        # Velocity publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def lidar_callback(self, msg):
        # Process laser scan data and make decisions
        # For demonstration purposes, we stop the motors if any obstacle is detected within 1 meter

        min_distance = min(msg.ranges)
        self.get_logger().info(min_distance)

        # if min_distance < 0.1:
        #     # If obstacle within 1 meter, stop both motors
        #     self.motor_stop(self.motor1)
        #     self.motor_stop(self.motor2)
        # else:
        #     # If no obstacle within 1 meter, move both motors forward
        #     self.motor_forward(self.motor1)
        #     self.motor_forward(self.motor2)

    def motor_forward(self, motor):
        motor.forward()

    def motor_backward(self, motor):
        motor.backward()

    def motor_stop(self, motor):
        motor.stop()

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()