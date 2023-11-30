import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOv8ObjectCounterNode(Node):
    def __init__(self):
        super().__init__('yolov8_object_counter_node')
        self.publisher_object_count = self.create_publisher(Int32, 'yolov8_object_count', 10)
        self.timer = self.create_timer(0.05, self.publish_object_count)  # Increased timer frequency
        self.bridge = CvBridge()
        self.yolo_model = YOLO("yolov8TACO.pt")
        self.video_capture = cv2.VideoCapture('dev/video1')
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set desired width
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set desired height
        self.frame_count = 0

    def publish_object_count(self):
        # Capture a frame from the video source
        ret, frame = self.video_capture.read()

        if not ret:
            self.get_logger().error("Failed to capture frame.")
            return

        # Increment the frame count
        self.frame_count += 1

        # Check if it's the 10th frame
        if self.frame_count % 10 == 0:
            # Perform object detection
            detections = self.yolo_model.predict(frame)

            # Check the type of the 'detections' variable
            print(type(detections))

    
            for result in detections:
                ind=-1
                largest_tensor = None
                largest_tensor_size = 0

                for result in detections:
                    current_tensor = result.boxes.xyxy[ind+1]
                    current_tensor_size = current_tensor.numel()  # Get the number of elements in the tensor

                    if current_tensor_size > largest_tensor_size:
                        largest_tensor = current_tensor
                        largest_tensor_size = current_tensor_size

                # Print the largest tensor after the loop
                if largest_tensor is not None:
                    print(f"Largest tensor is: {largest_tensor}")
                else:
                    print("No tensors found in detections.")




            # Get the number of detected objects
            num_objects = len(detections)


            # Publish the number of detected objects
            object_count_msg = Int32()
            object_count_msg.data = num_objects
            self.publisher_object_count.publish(object_count_msg)




def main(args=None):
    rclpy.init(args=args)
    yolov8_object_counter_node = YOLOv8ObjectCounterNode()
    rclpy.spin(yolov8_object_counter_node)
    yolov8_object_counter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()