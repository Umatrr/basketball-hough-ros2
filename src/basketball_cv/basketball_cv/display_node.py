import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from basketball_interface.msg import BasketballDetection

import cv2


class BasketballDisplay(Node):
    def __init__(self):
        super().__init__('basketball_display')

        self.declare_parameter('display_fps', 30.0)
        display_fps = float(self.get_parameter('display_fps').value)
        if display_fps <= 0:
            raise ValueError('display_fps must be greater than zero')

        self.bridge = CvBridge()

        # 保存最新一帧图像和最新检测结果
        self.latest_image = None
        self.latest_det = None

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

    def render(self):
        if self.latest_image is None:
            return

        bgr = self.latest_image.copy()

        # 如果检测到篮球，画框和圆
        if self.latest_det is not None and self.latest_det.detected:
            det = self.latest_det

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
                f'Basketball {det.confidence:.2f}',
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
