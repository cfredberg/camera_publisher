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
    def __init__(self, flip: bool, gst_str: str, name: str="camera"):
        self.name = name
        self.flip = flip
        self.cap_reopen_timer = None
        self.gst_str = gst_str
        
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

class UsbCameraPublisher(Node):

    def __init__(self):
        super().__init__('usb_camera_publisher')

        self.declare_parameter('use_name', False)
        self.declare_parameter('camera_name', "")
        self.declare_parameter('port', "0.0.0")
        self.declare_parameter('camera_id', 5)
        self.declare_parameter('by_id', False)
        self.declare_parameter('blue', False)
        self.declare_parameter('thick', False)
        self.declare_parameter('small', False)
        self.declare_parameter('ir', False)
        self.declare_parameter('pi', False)
        self.declare_parameter('flip', False)
        self.declare_parameter('legion', False)
        use_name = self.get_parameter('use_name').get_parameter_value().bool_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().string_value
        port = str(port)
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        by_id = self.get_parameter('by_id').get_parameter_value().bool_value
        using_blue = self.get_parameter('blue').get_parameter_value().bool_value
        using_thick = self.get_parameter('thick').get_parameter_value().bool_value
        using_small = self.get_parameter('small').get_parameter_value().bool_value
        using_ir = self.get_parameter('ir').get_parameter_value().bool_value
        using_pi = self.get_parameter('pi').get_parameter_value().bool_value
        flip = self.get_parameter('flip').get_parameter_value().bool_value
        legion = self.get_parameter('legion').get_parameter_value().bool_value

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
                gst_str = f'v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! image/jpeg,width=320,height=240,framerate=25/1 ! jpegdec ! videoconvert ! appsink'
            else:
                gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0" extra-controls="c,num_video_buffers=2" ! image/jpeg,width=320,height=240,framerate=25/1 ! jpegdec ! videoconvert ! appsink'
        elif using_thick:
            print("Using Thick Camera Stream")
            if use_name:
                gst_str = f'v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            else:
                gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0" extra-controls="c,num_video_buffers=2" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
        elif using_small:
            print("Using Small Camera Stream")
            if use_name:
                gst_str = f'v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
            else:
                gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0" extra-controls="c,num_video_buffers=2" ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegdec ! videoconvert ! appsink'
        elif using_ir:
            print("Using IR Camera Stream")
            if use_name: 
                gst_str = f'v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! videoconvert ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink'
            else:
                if not legion:
                    gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0" extra-controls="c,num_video_buffers=2" ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink'
                else:
                    gst_str = f'v4l2src device="/dev/v4l/by-path/pci-0000:80:14.0-usb-0:{port}:1.0-video-index0" ! videoscale ! video/x-raw,width=160,height=120,framerate=9/1,format=UYVY ! videoconvert ! video/x-raw,format=BGR ! appsink'
        elif using_pi:
            gst_str = f'nvarguscamerasrc sensor-id={port} ! video/x-raw(memory:NVMM),width=320,height=240,framerate=15/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink'
            #  max-buffers=1 drop=true sync=false
        else:
            print("Using Generic Camera Stream")
            if use_name:
                gst_str = f'v4l2src device="/dev/v4l/{type_cam_src}/{camera_name}" ! videoconvert ! appsink'
            else:
                gst_str = f'v4l2src device="/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0" ! image/jpeg,width=1640,height=1232,framerate=15/1 ! jpegdec ! videoconvert ! appsink'

        print(f"gst_str: {gst_str}")

        # self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        # print("Camera Opening...")
        # sleep(3)
        # if not self.cap.isOpened():
        #     raise CameraNotOpenError
        # print("Camera Open!")

        self.cam = Camera(flip, gst_str)

        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # ret, frame = self.cap.read()
        # if not ret:
        #     raise CameraStoppedReadingError

        # if self.flip:
        #     frame = cv2.rotate(frame, cv2.ROTATE_180)

        frame = cam.read_frame()
        
        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')

        msg.header.stamp = self.get_clock().now().to_msg()
        

        self.publisher.publish(msg)
        self.get_logger().info("sent frame " + str(self.i))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    usb_camera_publisher = UsbCameraPublisher()

    while True:
        try:
            rclpy.spin(unified_publisher)
        except Exception as e:
            print("Something went wrong.  Restarting camera feed now.")
            unified_publisher = UnifiedPublisher()

    # try:
    #     rclpy.spin(usb_camera_publisher)
    # except Exception as e:
    #     print(e)
    #     if isinstance(e, CameraStoppedReadingError):
    #         usb_camera_publisher.cap.release()
    # finally:
    #     # Destroy the node explicitly
    #     # (optional - otherwise it will be done automatically
    #     # when the garbage collector destroys the node object)
    #     usb_camera_publisher.destroy_node()
    #     rclpy.shutdown()
    usb_camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
