"""ROS 2 node that publishes basketball detections from Hough circles."""

from basketball_interface.msg import BasketballDetection

import cv2

from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from .circle_detection import CircleConfig, find_basketball


class BasketballDetector(Node):
    """Read camera frames, detect circles, and publish results."""

    def __init__(self):
        """Declare tuning parameters and connect ROS topics."""
        super().__init__('basketball_detector')
        defaults = CircleConfig()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))

        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.detection_pub = self.create_publisher(
            BasketballDetection, '/basketball/detection', 10
        )
        self.mask_pub = self.create_publisher(Image, '/basketball/mask', 10)
        self.get_logger().info('Hough basketball detector started')

    def on_image(self, msg):
        """Process one frame and preserve its timestamp in the results."""
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            config = CircleConfig(**{
                name: self.get_parameter(name).value
                for name in CircleConfig.__dataclass_fields__
            })
            circle, mask = find_basketball(bgr, config)
        except (ValueError, cv2.error, CvBridgeError) as exc:
            self.get_logger().error(f'Detection failed: {exc}')
            return

        result = BasketballDetection()
        result.header = msg.header
        result.detected = circle is not None
        if circle is not None:
            result.center_x = circle.x
            result.center_y = circle.y
            result.radius = circle.radius
            result.x = max(0, int(circle.x - circle.radius))
            result.y = max(0, int(circle.y - circle.radius))
            result.width = min(
                bgr.shape[1], int(circle.x + circle.radius) + 1
            ) - result.x
            result.height = min(
                bgr.shape[0], int(circle.y + circle.radius) + 1
            ) - result.y
            result.confidence = circle.score
        self.detection_pub.publish(result)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)


def main(args=None):
    """Run the detector node."""
    rclpy.init(args=args)
    node = BasketballDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
