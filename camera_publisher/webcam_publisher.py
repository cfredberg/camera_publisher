import rclpy
from rclpy.node import Node

import numpy as np

from cv_bridge import CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

import cv2
import pickle

from rclpy.qos import QoSProfile, ReliabilityPolicy

# from std_msgs.msg import String

class CameraNotOpenError(Exception):
    def __str__(self):
        return "The camera was not able to open properly"

class CameraStoppedReadingError(Exception):
    def __str__(self):
        return "The camera has unexpectedly stopped returning frames"

class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')

        qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )

        self.publisher = self.create_publisher(CompressedImage, f'/cameras/raw/camera_0', qos)

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise CameraNotOpenError

        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            raise CameraStoppedReadingError
        
        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')

        msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher.publish(msg)
        print(f"time: {self.get_clock().now()} frame: {str(self.i)}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()

    try:
        rclpy.spin(webcam_publisher)
    except Exception as e:
        print(e)
        if isinstance(e, CameraStoppedReadingError):
            webcam_publisher.cap.release()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        webcam_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
