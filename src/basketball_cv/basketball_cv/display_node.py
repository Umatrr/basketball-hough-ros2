import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from basketball_interface.msg import BasketballDetection

import cv2
import numpy as np


class BasketballDisplay(Node):
    def __init__(self):
        super().__init__('basketball_display')

        self.declare_parameter('display_fps', 30.0)
        self.declare_parameter('debug_enabled', False)
        self.declare_parameter('debug_hough_min_score', 0.0)
        self.declare_parameter('debug_mask_min_color_score', 0.0)
        display_fps = float(self.get_parameter('display_fps').value)
        if display_fps <= 0:
            raise ValueError('display_fps must be greater than zero')

        self.bridge = CvBridge()

        # 保存最新一帧图像和最新检测结果
        self.latest_image = None
        self.latest_det = None
        self.latest_mask = None

        # 订阅原始图像
        self.sub_img = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 订阅检测结果
        self.sub_det = self.create_subscription(
            BasketballDetection,
            '/basketball/detection',
            self.det_callback,
            10
        )
        self.sub_mask = self.create_subscription(
            Image,
            '/basketball/mask',
            self.mask_callback,
            10
        )

        # 定时渲染，约 30 FPS
        self.timer = self.create_timer(1.0 / display_fps, self.render)

        self.get_logger().info('Basketball display started.')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')

    def det_callback(self, msg):
        self.latest_det = msg

    def mask_callback(self, msg):
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='mono8'
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')

    def render(self):
        if self.latest_image is None:
            return

        bgr = self.latest_image.copy()

        debug = bool(self.get_parameter('debug_enabled').value)
        det = self.latest_det

        # debug 时允许显示“候选但未通过最终门槛”的圆，方便定位误差来源。
        show_candidate = (
            debug and det is not None
            and det.radius > 0
            and det.confidence >= float(
                self.get_parameter('debug_hough_min_score').value
            )
        )
        if det is not None and (det.detected or show_candidate):
            # 轴对齐外接矩形
            x = det.x
            y = det.y
            w = det.width
            h = det.height
            cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 最小外接圆
            cx = int(det.center_x)
            cy = int(det.center_y)
            r = int(det.radius)
            cv2.circle(bgr, (cx, cy), r, (0, 0, 255), 2)
            cv2.circle(bgr, (cx, cy), 3, (255, 0, 0), -1)

            # 文字
            cv2.putText(
                bgr,
                f'edge={det.edge_score:.2f} color={det.color_score:.2f} '
                f'total={det.confidence:.2f} accepted={det.detected}',
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
        else:
            cv2.putText(
                bgr,
                'No basketball',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        cv2.imshow('Basketball Display', bgr)
        if debug and self.latest_mask is not None:
            mask = self.latest_mask.copy()
            mask_score = float(
                self.get_parameter('debug_mask_min_color_score').value
            )
            if det is not None and det.color_score < mask_score:
                mask = np.zeros_like(mask)
                text = f'color score below {mask_score:.2f}'
            elif det is None:
                text = 'no selected candidate'
            else:
                text = f'color={det.color_score:.2f}'
            cv2.putText(
                mask, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, 255, 2,
            )
            cv2.imshow('Basketball Mask Debug', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BasketballDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        # launch 可能已经先关闭 ROS context，避免重复 shutdown 抛出 RCLError。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
