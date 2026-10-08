import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # 参数：摄像头设备编号，默认 0
        self.declare_parameter('device_id', 0)
        # 参数：图像宽度
        self.declare_parameter('width', 640)
        # 参数：图像高度
        self.declare_parameter('height', 480)
        # 参数：帧率
        self.declare_parameter('fps', 30)

        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        if fps <= 0:
            raise ValueError('fps must be greater than zero')

        # 打开摄像头
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera device {device_id}')
            raise RuntimeError('Camera open failed')

        self.bridge = CvBridge()

        # 发布图像话题
        self.pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # 定时器：按帧率读取并发布
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Camera node started: device={device_id}, '
            f'{width}x{height} @ {fps}fps'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return

        # 转成 ROS Image 消息
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
