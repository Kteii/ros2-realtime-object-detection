# Full, working object_detector.py file for YOLOv8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
from ultralytics import YOLO
from my_vision_interfaces.msg import Detection, DetectionArray

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.declare_parameter('confidence_threshold', 0.5)
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Load the YOLOv8 model. We use 'yolov8n.pt' (nano) for the best performance on CPU.
        # The model is automatically downloaded on the first run.
        try:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8 model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv8 model: {e}')
            sys.exit(1)

        # Create subscriber to the image topic and publisher for the detections
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(DetectionArray, '/object_detections', 10)
        self.get_logger().info('Object Detector Node has been started.')

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Perform object detection inference
        results = self.model(cv_image, verbose=False)

        # Create a message to publish the detection results
        detection_array_msg = DetectionArray()
        detection_array_msg.header = msg.header

        # Process the results from the model
        for res in results:
            boxes = res.boxes
            for box in boxes:
                if box.conf[0] > self.confidence_threshold:
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    xyxy = box.xyxy[0]
                    x_min, y_min, x_max, y_max = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])

                    # Create a Detection message for each object found
                    detection_msg = Detection()
                    detection_msg.label = class_name
                    detection_msg.confidence = float(box.conf[0])
                    detection_msg.x_min, detection_msg.y_min = x_min, y_min
                    detection_msg.x_max, detection_msg.y_max = x_max, y_max

                    detection_array_msg.detections.append(detection_msg)

                    # Draw the bounding box on the image for visualization
                    color = (0, 255, 0) # Green
                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)
                    cv2.putText(cv_image, f'{class_name} {float(box.conf[0]):.2f}', 
                                (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Publish the array of all detections
        self.publisher.publish(detection_array_msg)

        # Display the image in a window
        cv2.imshow("Detection Result (YOLOv8)", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down.')
    finally:
        # Clean up on exit
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

