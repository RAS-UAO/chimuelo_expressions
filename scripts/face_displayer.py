#!/usr/bin/env python3
# ROS2 libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import String
from sensor_msgs.msg import Image
# OpenCV libraries
import numpy as np # Numpy library
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
# Python libraries
from ament_index_python import get_package_share_directory
import os
import time

class ChimueloFaceDisplayer(Node):
    def __init__(self):
        super().__init__("chimuelo_face_displayer")
        self.cv_bridge = CvBridge()
        self.is_face_changed = False

        self.pkg_share_filepath = get_package_share_directory("chimuelo_expressions")
        self.videos_folder_name = "videos"

        self.blink_video = "Blink.mov"
        self.confused_video = "Confused.mov"
        self.finding_video = "Finding.mov"
        self.gossip_video = "Gossip.mov"
        self.excited_video = "Excited.mov"
        self.surprise_video = "Surprise.mov"
        self.talkie_talkie_video = "Talkie Talkie.mov"
        self.urgent_message_video = "Urgent Message.mov"
        self.wise_guy_video = "Wise Guy.mov"

        self.selected_video = self.excited_video
        self.chimuelo_face = os.path.join(
            self.pkg_share_filepath,
            self.videos_folder_name,
            self.selected_video
        )
        self.is_face_changed = False

        self.image_pub = self.create_publisher(
            Image,
            '/chimuelo_face/current_face', 
            10
        )

        self.str_sub = self.create_subscription(
            String, 
            '/chimuelo_face/face_selected', 
            self.str_callback, 
            10
        )

        self.chimuelo_face_timer = self.create_timer(0.1, self.display_chimuelo_face)
        
        print("Availables Chimuelo's faces:")
        print("1 - Blinking Chimuelo")
        print("2 - Confused Chimuelo")
        print("3 - Excited Chimuelo")
        print("4 - Chimuelo Found Something Cool")
        print("6 - Gossip Chimuelo")
        print("5 - Surprised Chimuelo")
        print("6 - Talkie Talkie Chimuelo")
        print("7 - Chimuelo has an Urgent Message")
        print("8 - Wise Chimuelo")

    
    def display_chimuelo_face(self):
        print(f"Selected video is: {self.selected_video}")
        capture = cv2.VideoCapture(self.chimuelo_face)

        while True:
            # Starting video display
            ret, frame = capture.read()
            if ret:
                ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
                cv2.imshow("Chimuelo Face", frame)

                # When ESC keyboard is pressed, the video is closed
                if cv2.waitKey(1) and 0xFF == ord('q'):
                    break

                if self.is_face_changed:
                    self.is_face_changed = False
                    break
            # Ending video display
            else:
                break
    
    def str_callback(self, str_msg):
        if(str_msg.data == "1"):
            self.selected_video = self.blink_video
        elif(str_msg.data == "2"):
            self.selected_video = self.confused_video
        elif(str_msg.data == "3"):
            self.selected_video = self.excited_video
        elif(str_msg.data == "4"):
            self.selected_video = self.finding_video
        elif(str_msg.data == "5"):
            self.selected_video = self.gossip_video
        elif(str_msg.data == "6"):
            self.selected_video = self.surprise_video
        elif(str_msg.data == "7"):
            self.selected_video = self.talkie_talkie_video
        elif(str_msg.data == "8"):
            self.selected_video = self.urgent_message_video
        elif(str_msg.data == "wise_guy"):
            self.selected_video = self.wise_guy_video
        else:
            print("No available option. Stay in the same face")
            self.selected_video = self.selected_video

        self.is_face_changed = True
        
        self.chimuelo_face = os.path.join(
            self.pkg_share_filepath,
            self.videos_folder_name,
            self.selected_video
        )

def main(args = None):
    rclpy.init(args = args)
    chimuelo_face_displayer_node = ChimueloFaceDisplayer()
    try:
        rclpy.spin(chimuelo_face_displayer_node)
    except KeyboardInterrupt:
        chimuelo_face_displayer_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()