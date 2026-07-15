# ROS Client Library for Python
import rclpy
from rclpy.qos import qos_profile_sensor_data
# Handles the creation of nodes
from rclpy.node import Node
# Handles string messages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from object_tracking import track_objects
from object_tracking import object_tracker
bridge = CvBridge()

class ObjectTrackerSubscriber(Node):
    """
    Create a subscriber node
    """
    last_image = None

    def __init__(self):

        # Initiate the Node class's constructor and give it a name
        super().__init__('object_tracker_subscriber')
        self.last_image = None
        # The node subscribes to messages of type std_msgs/String,
        # The callback function is called as soon as a message is received.
        # The maximum number of queued messages is 10.
        self.publisher = self.create_publisher(
            Image, '/front_camera/to_image', qos_profile_sensor_data)

        self.subscription = self.create_subscription(
            Image,
            '/front_camera/undistorted_image',
            self.listener_callback,
            qos_profile_sensor_data)

        self.subscription  # prevent unused variable warning
        self.camera_timer_ = self.create_timer(
            0.1, self.timer_callback)
        self.object_tracker = object_tracker.Object_Tracker(0.1)

    def listener_callback(self, msg):
        # Display a message on the console every time a message is received on the
        # addison topic
        current_frame = bridge.imgmsg_to_cv2(msg)
        self.last_image = current_frame
        # self.publisher_.publish(msg)
        # Display image

        # cv2.waitKey(1)
    def timer_callback(self):
        if((self.last_image) is not None):
            frame = self.last_image.copy()
            new_img = track_objects.track_objects(
                frame, self.object_tracker)
            msg = bridge.cv2_to_imgmsg(new_img, encoding='bgr8')
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.get_clock().now().to_msg()
            #msg.header.stamp = Time(seconds = self.get_clock().now()).to_msg()

            self.publisher.publish(msg)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create a subscriber
    object_tracker_subscriber = ObjectTrackerSubscriber()

    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    rclpy.spin(object_tracker_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_tracker_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
