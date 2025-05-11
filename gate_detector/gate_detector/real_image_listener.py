#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# ROS2 camera topic publishes images as sensor_msgs/Image message
from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

# CvBridge that convert betweeb ROS Image messages and OpenCV images 
from cv_bridge import CvBridge

#cv2 is the main module in OpenCV that provides us with an easy-to-use
#interface for working with image and video processing functions.
import cv2
import time
import numpy as np

class GatePilot(Node):
    def __init__(self):
        super().__init__('gate_pilot')
        self.bridge = CvBridge()

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service client for takeoff
        self.takeoff_client = self.create_client(TelloAction, '/tello_action')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Waiting for /tello_action service...')

        # Request for takeoff
        req = TelloAction.Request()
        req.cmd = 'takeoff'

        # taking the drone to a specific height
        time.sleep(15)
        twist = Twist()
        twist.linear.z = 0.7
        self.cmd_pub.publish(twist)
        time.sleep(2)
        twist.linear.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.1)

        self.takeoff_client.call_async(req).add_done_callback(
            lambda future: self.get_logger().info('Takeoff complete'))
        self.took_off = True

        # Subscribe to camera, when an image will be available in /image_raw topic cb_image() method will recieve that image
        self.sub = self.create_subscription(
            Image, '/image_raw', self.cb_image, qos_profile_sensor_data)

        # Gate detection: only green border

        self.green_low  = np.array((50,  100,  10))
        self.green_high = np.array((90, 220, 140))
        self.draw_color = (0, 255, 0)

        # Control parameters
        self.center_x = None
        self.center_y = None
        self.kp_yaw = 0.002
        self.kp_alt = 0.001
        self.forward_speed = 0.3

        # Pass-through buffer
        self.pass_through_frames = 0
        self.PASS_FRAMES_MAX = 70

    def preprocess(self, mask):
        k1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))        # An ellipse of size 5×5 for opening (gentler on curved edges)
        k2 = cv2.getStructuringElement(cv2.MORPH_RECT,   (7,7))         # A rectangle of size 7×7 for closing (good at filling wider gaps).
        m = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k1, iterations=2)   # MORPH_OPEN with the small ellipse kernel, run twice, strips out tiny white speckles—cleanup of salt-and-pepper noise.
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k2, iterations=2)   #closes small black holes inside your white regions

    def classify_contour(self, cnt):
        area = cv2.contourArea(cnt)
        peri = cv2.arcLength(cnt, True)
        
        # square test
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            x,y,w,h = cv2.boundingRect(approx)
            ar = float(w)/h
            if 0.8 < ar < 1.2:
                return 'square'

        # circle test via circularity
        circ = (4 * np.pi * area) / (peri * peri) if peri > 0 else 0
        if circ > 0.8:
            return 'circle'
        return None

    def centroid(self, cnt):
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            return int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        x,y,w,h = cv2.boundingRect(cnt)
        return x + w//2, y + h//2

    def cb_image(self, msg):
        if not self.took_off:
            return

        # convert ROS image to OpenCV, save carent camera output to frame
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # In OpenCV (and NumPy), an image array’s shape is a tuple like (rows, cols, channels)
        # Grabbing the first two elements of frame.shape, height and weight
        h, w = frame.shape[:2]

        # Store the center of the camera frame
        if self.center_x is None:
            self.center_x = w // 2
            self.center_y = h // 2

        # BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # h_, s_, v_ = cv2.split(hsv)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        # v_eq = clahe.apply(v_)
        # hsv_eq = cv2.merge((h_, s_, v_eq))

        # 2) Threshold for green border
        mask = cv2.inRange(hsv, self.green_low, self.green_high)
        mask = self.preprocess(mask)

        # 3) Find hole‐contours inside the green border
        cnts, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        candidate = None  # (area, cnt, cx, cy, shape)
        if hier is not None:
            for i, hinfo in enumerate(hier[0]):
                if hinfo[3] < 0:
                    continue  # skip outer frame
                cnt = cnts[i]
                area = cv2.contourArea(cnt)
                if area < 1000:
                    continue
                shape = self.classify_contour(cnt)
                if not shape:
                    continue
                cx, cy = self.centroid(cnt)
                if candidate is None or area > candidate[0]:
                    candidate = (area, cnt, cx, cy+50, shape)

        # 4) Build and publish command
        twist = Twist()
        if candidate:
            _, cnt, cx, cy, shape = candidate
            # draw detection
            cv2.drawContours(frame, [cnt], -1, self.draw_color, 3)
            cv2.circle(frame, (cx, cy), 5, self.draw_color, -1)
            cv2.putText(frame, shape, (cx-30, cy-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.draw_color, 2)

            # compute errors
            err_x = cx - self.center_x
            err_y = cy - self.center_y

            # control
            twist.angular.z = -self.kp_yaw * err_x
            twist.linear.z  = -self.kp_alt * err_y
            if abs(err_x) < 0.1*w and abs(err_y) < 0.1*h:
                twist.linear.x  = self.forward_speed

                # reset pass-through
                self.pass_through_frames = self.PASS_FRAMES_MAX

        elif self.pass_through_frames > 0:
            # carry forward if detection drops
            twist.linear.x = self.forward_speed
            self.pass_through_frames -= 1
        # else hover

        self.cmd_pub.publish(twist)

        # visualize
        cv2.imshow('Gate Pilot', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = GatePilot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
