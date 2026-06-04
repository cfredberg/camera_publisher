import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cv2
import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image


class FrameRecovery(Node):

    def __init__(self):
        super().__init__('frame_recovery')

        self.declare_parameter('sub_topic', '')
        self.declare_parameter('hang_time', 0.5)

        sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        self.hang_time = self.get_parameter('hang_time').get_parameter_value().double_value

        try:
            ret, frame = cv2.imread("no_signal.png")
        except Exception:
            ret = False
        
        if not ret:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            frame = cv2.putText(frame, 'No Signal', (0,480), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))

        self.bridge = CvBridge()
        self.no_signal_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        self.frame_publisher = self.create_publisher(Image, sub_topic, 1)
        self.fps = 120
        timer_period = 1/self.fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.last_msg_time = self.i

        self.subscription = self.create_subscription(
            Image,
            sub_topic,
            self.listener_callback,
            1)

    def timer_callback(self):
        cur_time = self.i

        if (cur_time - self.last_msg_time)/self.fps >= self.hang_time:
            self.frame_publisher.publish(self.no_signal_msg)
            '''
            
            Key issue: when msg sent after timeout, subscriber sees the msg, and resets timeout wait.  This adds a lot of delay on the main interface.
            
            '''
        
        print(f'i: {self.i}')

        self.i += 1

    def listener_callback(self, msg):
        self.last_msg_time = self.i

def main(args=None):
    rclpy.init(args=args)

    frame_recovery = FrameRecovery()

    rclpy.spin(frame_recovery)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frame_recovery.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()