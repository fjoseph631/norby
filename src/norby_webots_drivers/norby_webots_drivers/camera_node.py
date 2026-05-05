# ROS Client Library for Python
import rclpy
import numpy
import camera_calibration
from numpy import asarray
from rclpy.qos import qos_profile_sensor_data
# Handles the creation of nodes
from rclpy.node import Node
from rclpy.time import Time

# Handles string messages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

import cv2  # OpenCV library
bridge = CvBridge()


class CameraSubscriber(Node):
   """
   Create a subscriber node
   """
   last_image = None

   def __init__(self):

      # Initiate the Node class's constructor and give it a name
      super().__init__('camera_subscriber')
      last_image = None
      x = 0
      # The node subscribes to messages of type std_msgs/String,
      # over a topic named: /addison
      # The callback function is called as soon as a message is received.
      # The maximum number of queued messages is 10.
      self.publisher = self.create_publisher(
         Image, '/front_camera/undistorted_image', qos_profile_sensor_data)

      self.subscription = self.create_subscription(
         Image,
         '/front_camera/camera_raw',
         self.listener_callback,
         qos_profile_sensor_data)

      # calibrate camera
      #self.mtx
      #self.dist
      self.subscription  # prevent unused variable warning
      self.camera_timer_ = self.create_timer(
         0.01, self.timer_callback)

   def listener_callback(self, msg):
      # Display a message on the console every time a message is received on the
      # addison topic
      current_frame = bridge.imgmsg_to_cv2(msg)
      self.last_image = current_frame
      # self.publisher_.publish(msg)
      # Display image

      # cv2.waitKey(1)
   def timer_callback(self):
      if(type(self.last_image) != type(None)):
         numpydata = asarray(self.last_image)
         img = cv2.resize(self.last_image,((1280,960)))
         new_img = img
         #new_img = cv2.undistort(
         #      img, self.mtx, self.dist, None, self.mtx)
         msg = bridge.cv2_to_imgmsg(new_img)
         msg.header.frame_id = 'base_link'
         msg.header.stamp = self.get_clock().now().to_msg()

         self.publisher.publish(msg)


def main(args=None):

   # Initialize the rclpy library
   rclpy.init(args=args)

   # Create a subscriber
   camera_subscriber = CameraSubscriber()

   # Spin the node so the callback function is called.
   # Pull messages from any topics this node is subscribed to.
   rclpy.spin(camera_subscriber)

   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   camera_subscriber.destroy_node()

   # Shutdown the ROS client library for Python
   rclpy.shutdown()


if __name__ == '__main__':
   main()
