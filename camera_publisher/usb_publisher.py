import rclpy
from rclpy.node import Node

import numpy as np

from cv_bridge import CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
import pickle

from time import sleep

# from std_msgs.msg import String

class CameraNotOpenError(Exception):
    def __str__(self):
        return "The camera was not able to open properly"

class CameraStoppedReadingError(Exception):
    def __str__(self):
        return "The camera has unexpectedly stopped returning frames"

class UsbCameraPublisher(Node):

    def __init__(self):
        super().__init__('usb_camera_publisher')

        self.declare_parameter('camera_name', "")
        self.declare_parameter('camera_id', 5)
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.publisher = self.create_publisher(Image, f'/cameras/raw/camera_{camera_id}', 1)

        self.bridge = CvBridge()

        print(f"Camera name: {camera_name}")
        # gst_str = f"v4l2src device=/dev/v4l/by-id/{camera_name} ! video/x-raw,width=320,height=240,framerate=30/1 ! videoconvert ! appsink"
        gst_str = f'v4l2src device="/dev/v4l/by-id/{camera_name}" ! videoconvert ! appsink'
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        print("Camera Opening...")
        sleep(2)
        if not self.cap.isOpened():
            raise CameraNotOpenError
        print("Camera Open!")

        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            raise CameraStoppedReadingError
        pickled_frame = pickle.dumps(frame)
        
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        

        self.publisher.publish(msg)
        self.get_logger().info("sent frame " + str(self.i))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    usb_camera_publisher = UsbCameraPublisher()

    try:
        rclpy.spin(usb_camera_publisher)
    except Exception as e:
        print(e)
        if isinstance(e, CameraStoppedReadingError):
            usb_camera_publisher.cap.release()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        usb_camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
