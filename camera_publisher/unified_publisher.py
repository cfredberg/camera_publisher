import rclpy
from rclpy.node import Node

import numpy as np

from cv_bridge import CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import cv2
import pickle

import time
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
        self.cap_reopen_timer = None
        match cam_type:
            case "blue":
                print(f"{self.name} using Blue Camera Stream")
                self.gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=320,height=240,framerate=25/1 ! jpegdec ! videoconvert ! appsink'
            case "thick":
                print(f"{self.name} using Thick Camera Stream")
                self.gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            case "small":
                print(f"{self.name} using Small Camera Stream")
                self.gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            case "ir":
                print(f"{self.name} using IR Camera Stream")
                self.gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! videoconvert ! videoscale ! video/x-raw,width=320,height=240 ! appsink'
            case _:
                print(f"{self.name} using Generic Camera Stream")
                self.gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! videoconvert ! appsink'

        print(f"{self.name} gst_str: {self.gst_str}")

        self.open_cap()

    def open_cap(self):
        self.cap_reopen_timer = time.time()
        self.cap = cv2.VideoCapture(self.gst_str, cv2.CAP_GSTREAMER)
        print(f"{self.name} Opening...")

    def read_frame(self):
        if self.cap_reopen_timer != None:
            diff = time.time() - self.cap_reopen_timer
            if diff > 2:
                self.cap_reopen_timer = None
        elif self.cap.isOpened():
            self.cap_reopen_timer = None
            ret, frame = self.cap.read()
            if ret:
                if self.flip:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                return frame
            else:
                frame = np.zeros((240, 320, 3), dtype=np.uint8)
                frame = cv2.putText(frame, 'No Signal', (300,240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
                return frame
        else:
            self.open_cap()
        
        print(f"{self.name} not open")

        frame = np.zeros((240, 320, 3), dtype=np.uint8)
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

        self.publisher = self.create_publisher(CompressedImage, f'/cameras/raw/unified_camera', qos)

        self.bridge = CvBridge()

        timer_period = 1/40  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        frames_0_1 = cv2.hconcat([self.cams[0].read_frame(), self.cams[1].read_frame()])
        frames_2_3 = cv2.hconcat([self.cams[2].read_frame(), self.cams[3].read_frame()])
        all_frames = cv2.vconcat([frames_0_1, frames_2_3])
        
        msg = self.bridge.cv2_to_compressed_imgmsg(all_frames, dst_format='jpg')

        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)
        self.get_logger().info("sent frame " + str(self.i))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    unified_publisher = UnifiedPublisher()

    while True:
        try:
            rclpy.spin(unified_publisher)
        except Exception as e:
            print("Something went wrong.  Restarting camera feed now.")
            unified_publisher = UnifiedPublisher()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    unified_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()