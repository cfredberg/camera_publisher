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

class UsbCameraPublisher(Node):

    def __init__(self):
        super().__init__('usb_camera_publisher')

        self.declare_parameter('use_name', False)
        self.declare_parameter('camera_name', "")
        self.declare_parameter('port', 0.0)
        self.declare_parameter('camera_id', 5)
        self.declare_parameter('by-id', False)
        self.declare_parameter('blue', False)
        self.declare_parameter('thick', False)
        self.declare_parameter('small', False)
        self.declare_parameter('ir', False)
        self.declare_parameter('flip', False)
        use_name = self.get_parameter('use_name').get_parameter_value().bool_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().double_value
        port = str(port)
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        by_id = self.get_parameter('by-id').get_parameter_value().bool_value
        using_blue = self.get_parameter('blue').get_parameter_value().bool_value
        using_thick = self.get_parameter('thick').get_parameter_value().bool_value
        using_small = self.get_parameter('small').get_parameter_value().bool_value
        using_ir = self.get_parameter('ir').get_parameter_value().bool_value
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.publisher = self.create_publisher(CompressedImage, f'/cameras/raw/camera_{camera_id}', qos)

        self.bridge = CvBridge()

        if by_id:
            type_cam_src = "by-id"
        else:
            type_cam_src = "by-path"

        print(f"Camera name: {camera_name}")

        if using_blue:
            print("Using Blue Camera Stream")
            if use_name:
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! image/jpeg,width=640,height=480,framerate=25/1 ! jpegdec ! videoconvert ! appsink'
            else:
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=640,height=480,framerate=25/1 ! jpegdec ! videoconvert ! appsink'
        elif using_thick:
            print("Using Thick Camera Stream")
            if use_name:
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            else:
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
        elif using_small:
            print("Using Small Camera Stream")
            if use_name:
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            else:
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
        elif using_ir:
            print("Using IR Camera Stream")
            if use_name: 
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! appsink'
            else:
                gst_str = f'gst-launch-1.0 v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! appsink'
        else:
            print("Using Generic Camera Stream")
            if use_name:
                gst_str = f'v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! videoconvert ! appsink'
            else:
                gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:2.{port}:1.0-video-index0" ! videoconvert ! appsink'

        print(f"gst_str: {gst_str}")

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

        if self.flip:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')

        msg.header.stamp = self.get_clock().now().to_msg()
        

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
