#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge

from ultralytics import YOLO

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('device')
        device = self.get_parameter('device').value

        # self.get_image = self.create_subscription(
        #     Image, 'camera/image', self.listener_callback, 10
        #     )

        self.pub_image = self.create_publisher(
            Image, 'vision', 10
            )
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.stream = cv.VideoCapture(device)

        if not self.stream.isOpened():
            self.get_logger().error(
                f"Cannot open camera {device}, exiting"
                )
        
        self.model = YOLO("yolov8n.pt")

    
    def timer_callback(self):
        ret, frame = self.stream.read()

        if not ret:
            return
        
        results = self.model.track(frame, persist=True)
        # results = self.model(frame)[0]
        # print(results.xyxy[0])
        # res_plotted = results

        
        img = CvBridge().cv2_to_imgmsg(results[0].plot(), encoding="bgr8")
        self.pub_image.publish(img)

    def listener_callback(self, msg):
        ret, frame = self.stream.read()
        # frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        if not ret:
            return
        
        # results = self.model.track(frame, persist=True)
        results = self.model(frame)[0]
        # print(results.xyxy[0])
        # res_plotted = results
        
        img = CvBridge().cv2_to_imgmsg(results.plot(frame), encoding="bgr8")
        self.pub_image.publish(img)    
    
    # def __del__(self):
    #     print("the object was destroyed.")
    #     self.stream.release()


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
