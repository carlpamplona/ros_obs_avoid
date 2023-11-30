# 12 16 20 21

from gpiozero import Robot
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from time import sleep
import math  # Import the math module for the degrees() function
from std_msgs.msg import Int32, String as StringMsg
from ros2node.api import get_node_names




target_node_name = 'rosbridge_websocket'
joy2_node_name = 'joy2'
auto_mode = False
client_connected = 0
mode_state = False
is_auto = False
msg_stamp_new = ''
msg_stamp_old = ''
is_rjoy2_ok = False


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.joy2_node_name = joy2_node_name
        self.target_node_name = target_node_name
        self.node_found = False
      
        self.my_robot = Robot(left=(12, 16), right=(20, 21))

        # LIDAR subscription
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # replace with the actual topic name
            self.lidar_callback,
            10
        )

        # ConnectedClients subscription
        self.subscription = self.create_subscription(
            Int32,
            'client_count',  # replace with the actual topic name
            self.connected_clients_callback,
            10
        )

        self.subscription_joy = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        # Create a subscription for String messages
        self.subscription_string = self.create_subscription(
            StringMsg, 'joy2', self.string_callback, 10)


        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)

    def joy_callback(self, msg):
        global is_auto
        class_of_object = type(msg)
        is_auto = False
        

    def string_callback(self, msg):
        global is_auto
        class_of_object = type(msg)
        is_auto = True
       



    def connected_clients_callback(self, msg):
            global client_connected    
            print(f"Connected Clients: {msg.data}")
            client_connected = int(msg.data)
           

    
                
       
    def lidar_callback(self, msg):
            global is_auto, auto_mode, client_connected

            node = rclpy.create_node("list_nodes_example")
            available_nodes = get_node_names(node=self, include_hidden_nodes=False)
            for name, namespace, full_name in available_nodes:
                if self.target_node_name == name:
                    self.is_rosbridge_websocket_ok = True
                    break
                else:
                    self.is_rosbridge_websocket_ok = False

                if self.joy2_node_name == name:
                    self.is_rjoy2_ok = True
                    break
                else:
                    self.is_rjoy2_ok = False
            node.destroy_node()



        # Process laser scan data and make decisions
        # For demonstration purposes, we stop the motors if any obstacle is detected within 1 meter
           
            min_distance = min(msg.ranges)
            min_distance_index = msg.ranges.index(min_distance)

            # Get the angle corresponding to the minimum distance
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            angle_of_min_distance_rad = angle_min + (min_distance_index * angle_increment)

            angle_of_min_distance_deg = math.degrees(angle_of_min_distance_rad)

            # self.get_logger().info(f'Minimum distance: {min_distance} meters at angle: {angle_of_min_distance_deg} degrees')
       
            if angle_of_min_distance_deg < 0:
                side = 'left'
            else:
                side = 'right'

            # self.get_logger().info(f'Minimum distance is on the {side} side')

            # Create Joy message
            joy_msg = Joy()
            joy_msg.axes = [0.0, 0.0, 0.0, 0.0]  # Assuming your joystick has 4 axes (adjust as needed)
            joy_msg.buttons = [0, 0, 0, 0]  # Assuming your joystick has 4 buttons (adjust as needed)

            # if not msg.ranges:  # Check if the ranges list is empty
            #     self.get_logger().warn('Empty ranges list in LaserScan message')
            #     return

            # min_distance = min(msg.ranges)
            # # self.get_logger().info(f'Minimum distance: {min_distance} meters')


            if client_connected>=1:
                
                if min_distance < 0.3:
                    # If obstacle within 1 meter, stop both motors
                    # self.my_robot.stop()
                    # self.get_logger().info(f'Obstacle Detected')

                    if angle_of_min_distance_deg < 0:
                        # self.my_robot.right(0.50)
                        joy_msg.axes[0] = -1.0  # Move right
                    else:
                        joy_msg.axes[0] = 1.0  # Move left
                        # self.my_robot.left(0.50)
                    self.joy_publisher.publish(joy_msg)
                
                else:
                    # If no obstacle within 1 meter, move both motors forward
                   
                    
                    
                    if is_auto:
                        self.get_logger().info(f'Auto: No Obstacle')
                        joy_msg.axes[1] = 1.0
                        self.joy_publisher.publish(joy_msg)
                    else:
                        self.get_logger().info(f'Manual: No Obstacle')

            else:
                print("App not Connected.")



 

    
    def destroy(self):
        self.my_robot.stop()
        super().destroy_node()

def main(args=None):
    global auto_mode
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()