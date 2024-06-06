import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov5 import YOLOv5
import cv2
from cv_bridge import CvBridge

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.model = YOLOv5('yolov5s.pt')
        self.bridge = CvBridge()
        self.target_classes = ['person', 'car']
        
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image)
        for result in results:
            if result['class'] in self.target_classes:
                self.get_logger().info(f"Detected {result['class']} with confidence {result['confidence']}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
