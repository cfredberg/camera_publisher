import rclpy
from rclpy.node import Node

import numpy as np

from cv_bridge import CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import cv2
import pickle

from time import sleep

from rclpy.qos import QoSProfile, ReliabilityPolicy

# from std_msgs.msg import String

class CameraNotOpenError(Exception):
    def __str__(self):
        return "The camera was not able to open properly"

class CameraStoppedReadingError(Exception):
    def __str__(self):
        return "The camera has unexpectedly stopped returning frames"

class Camera():
    def __init__(self, name: str, port: float, flip: bool, cam_type: str):
        self.name = name
        self.flip = flip
        match cam_type:
            case "blue":
                print(f"{self.name} using Blue Camera Stream")
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=320,height=240,framerate=25/1 ! jpegdec ! videoconvert ! appsink'
            case "thick":
                print(f"{self.name} using Thick Camera Stream")
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            case "small":
                print(f"{self.name} using Small Camera Stream")
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            case "ir":
                print(f"{self.name} using IR Camera Stream")
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! videoconvert ! videoscale ! video/x-raw,width=320,height=240 ! appsink'
            case _:
                print(f"{self.name} using Generic Camera Stream")
                gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! videoconvert ! appsink'

        print(f"{self.name} gst_str: {gst_str}")

        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        print(f"{self.name} Opening...")
        sleep(2)
        if not self.cap.isOpened():
            print(f"{self.name} unable to be opened")
        print(f"{self.name} open!")

    def read_frame(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                if self.flip:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                return frame
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                frame = cv2.putText(frame, 'No Signal', (300,240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
                return frame
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = cv2.putText(frame, f'{self.name} capture not open', (0,240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
        return frame


class UnifiedPublisher(Node):

    def __init__(self):
        super().__init__('unified_camera_publisher')

        self.cams = []

        self.cams.append(
            Camera(
                "front",
                4.2,
                True,
                "blue"
            )
        )
        self.cams.append(
            Camera(
                "reverse",
                4.1,
                True,
                "blue"
            )
        )
        self.cams.append(
            Camera(
                "arm",
                4.4,
                False,
                "thick"
            )
        )
        self.cams.append(
            Camera(
                "palm",
                3.4,
                False,
                "small"
            )
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.publisher = self.create_publisher(Image, f'/cameras/raw/unified_camera', qos)

        self.bridge = CvBridge()

        timer_period = 1/25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        frames_0_1 = cv2.hconcat([self.cams[0].read_frame(), self.cams[1].read_frame()])
        frames_2_3 = cv2.hconcat([self.cams[2].read_frame(), self.cams[3].read_frame()])
        all_frames = cv2.vconcat([frames_0_1, frames_2_3])
        
        msg = self.bridge.cv2_to_imgmsg(all_frames, "bgr8")

        self.main_interface_frame.publish(msg)
        self.get_logger().info("sent frame " + str(self.i))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    unified_publisher = UnifiedPublisher()

    rclpy.spin(unified_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    usb_camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()