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
        self.declare_parameter('debug_enabled', False)
        self.declare_parameter('debug_log_period', 1.0)

        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.detection_pub = self.create_publisher(
            BasketballDetection, '/basketball/detection', 10
        )
        self.mask_pub = self.create_publisher(Image, '/basketball/mask', 10)
        self.last_debug_log = 0.0
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
        # ROS 2 生成的消息转换器要求原生 Python bool，不能传 numpy.bool_。
        result.detected = bool(circle is not None and circle.accepted)
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
            result.edge_score = circle.edge_score
            result.color_score = circle.color_score
        self.log_debug(circle)
        self.detection_pub.publish(result)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    def log_debug(self, circle):
        """Log candidate components at a reduced rate when debug is enabled."""
        if not self.get_parameter('debug_enabled').value:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        period = float(self.get_parameter('debug_log_period').value)
        if now - self.last_debug_log < max(0.05, period):
            return
        self.last_debug_log = now
        if circle is None:
            self.get_logger().info('candidate=none (HoughCircles returned no candidate)')
            return
        self.get_logger().info(
            f'candidate edge={circle.edge_score:.3f} '
            f'color={circle.color_score:.3f} total={circle.score:.3f} '
            f'accepted={circle.accepted}'
        )


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
        # launch 可能已经先关闭 ROS context，避免重复 shutdown 抛出 RCLError。
        if rclpy.ok():
            rclpy.shutdown()
