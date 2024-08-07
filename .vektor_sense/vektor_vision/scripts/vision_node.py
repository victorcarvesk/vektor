#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('device', int())
        device = self.get_parameter('device').value

        self.publisher = self.create_publisher(Image, 'vision/image', 10)
        self.timer = self.create_timer((1/30), self.timer_callback)
        
        self.stream = cv.VideoCapture(device)

        if not self.stream.isOpened():
            self.get_logger().error(
                f"Cannot open camera {device}, exiting"
                )

    def timer_callback(self):
        connected, frame = self.stream.read()

        if not connected:
            return
        
        self.publisher.publish(CvBridge().cv2_to_imgmsg(frame))
       
    def __del__(self):
        if self.stream.isOpened():
            self.stream.release()


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)

    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
