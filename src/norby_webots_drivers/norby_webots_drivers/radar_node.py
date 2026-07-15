import object_tracking
from object_tracking.radar_object_tracker_node import Radar_Object_Tracker
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from radar_msgs.msg import RadarTracks, RadarScan, RadarTrack, RadarReturn
from geometry_msgs.msg import PointStamped, Point, Vector3, PoseStamped
from unique_identifier_msgs import *
from scipy.spatial.transform import Rotation
import math
from math import copysign, cos, sin, pi, atan2, asin, sqrt
import numpy as np

import unique_identifier_msgs


class RadarSubscriber(Node):

    """
    Create a subscriber node
    """

    def __init__(self):

        # Initiate the Node class's constructor and give it a name
        super().__init__('radar_subscriber')

        self.speed = 0.0
        self.absolute = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gps_offset = 0.1
        self.rotationdict = {
            "front_left": pi/4,
            "front_right": 7 * pi/4,
            "back_left": 5 * pi/4,
            "back_right": 3 * pi/4
        }
        self.tracks = []
        self.track_publisher = self.create_publisher(
            RadarTracks, '/radar_tracks', 1)
        self.track_pose_publisher = self.create_publisher(
            PoseStamped, '/radar_tracks_poses', 1)

        self.time_step = 0.032
        self.object_tracker = object_tracking.radar_object_tracker_node.Radar_Object_Tracker(
            self.time_step)
        self.create_timer(self.time_step * 1e-3, self.track_pub)
        self.create_timer(self.time_step * 1e-3, self.track_poses_pub)

        self.subscription = self.create_subscription(
            RadarScan,
            '/base_radar',
            self.listener_callback,
            qos_profile_sensor_data

        )
        self.gps_subscription = self.create_subscription(
            PointStamped,
            '/gps_sensor',
            self.gps_callback,
            qos_profile_sensor_data

        )
        self.subscription  # prevent unused variable warning

        self.gps_subscription = self.create_subscription(
            Float32,
            '/gps_sensor/speed',
            self.gps_speed_callback,
            qos_profile_sensor_data
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

    def gps_speed_callback(self, msg):
        self.speed = msg.data

    def gps_callback(self, msg):
        self.absolute = [msg.point.x, msg.point.y, msg.point.z]

    def listener_callback(self, msg):
        boxes = []
        for ret in msg.returns:
            box = self.create_track(msg.header.frame_id, ret)
            boxes.append(box)
        if(boxes != []):
            tracks = self.object_tracker.track_objects(boxes, self.tracks)
            self.tracks = self.convert_tracks_to_msg(tracks)

    def convert_tracks_to_msg(self, tracks):
        ret = []
        for track in tracks:
            msg = RadarTrack()
            vec = track.KF.x.flatten()
            msg.position.x = vec[0]
            msg.position.z = vec[1]
            msg.velocity.x = vec[2]
            msg.velocity.z = vec[3]
            velo = sqrt(vec[2] ** 2 + vec[3] ** 2)
            if (velo < 0.3):
                msg.classification = RadarTrack.STATIC
            else:
                msg.classification = RadarTrack.DYNAMIC
            id = track.id
            arr = np.array([id])
            id_arr = np.zeros(15, dtype=int)
            id_arr = np.insert(id_arr, 0, id)
            msg.uuid = unique_identifier_msgs.msg.UUID(uuid=id_arr)
            msg.size.x = 0.1
            msg.size.y = 0.1
            msg.size.z = 0.1
            ret.append(msg)
        return ret

    def track_pub(self):
        msg = RadarTracks()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.tracks = self.tracks
        if(msg.tracks != []):
            self.track_publisher.publish(msg)

    def track_poses_pub(self):
        for track in self.tracks:
            msg = PoseStamped()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position = track.position
            if(self.tracks != []):
                self.track_pose_publisher.publish(msg)

    def imu_callback(self, msg):
        self.roll, self.pitch, self.yaw = quaternion_to_euler(msg.orientation)

    def create_track(self, radar_name, det):
        msg = RadarTrack()
        extra_rotation = self.rotationdict[radar_name.replace(
            "_radar_sensor", "")]
        rotation = self.pitch + (det.azimuth) + extra_rotation
        self.absolute[2] = (self.absolute[2] * sin(self.pitch)
                            ) + self.gps_offset * sin(self.pitch)
        x = (cos(rotation) * det.range + self.absolute[0])
        y = (sin(rotation) * det.range + self.absolute[2])
        rx = (cos(self.pitch) * self.speed)
        ry = (sin(self.pitch) * self.speed)
        xhat = (cos(rotation) * det.doppler_velocity) + rx
        yhat = (sin(rotation) * det.doppler_velocity) + ry
        pt = Point()
        pt.x = x
        pt.y = y
        msg.position = pt
        velo = Vector3()
        velo.x = xhat
        velo.y = yhat
        msg.velocity = velo
        msg.size.x = 0.05
        msg.size.y = 0.05
        msg.size.z = 0.05
        ret = np.array([x - msg.size.x, y - msg.size.z, x +
                       msg.size.x, y + msg.size.z, velo.x, velo.y])
        # self.tracks.append(msg)
        return ret


def quaternion_to_euler(q1):
    qx = q1.x
    qy = q1.y
    qz = q1.z
    qw = q1.w
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    # roll
    roll = atan2(sinr_cosp, cosr_cosp)
    #heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy ** 2 - 2*qz ** 2)
    # pitch
    sinp = 2 * (qw * qy - qz * qx)
    pitch = 0.0
    if (abs(sinp) >= 1):
        pitch = copysign(pi / 2, sinp)
    else:
        pitch = asin(sinp)
    # yaw
    siny_cosp = 2 * (qw * qz + qx * qz)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def euler_to_quaternion(yaw, pitch, roll):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
        math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
        math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    if math.isnan(qx):
        qx = 0.0
    if math.isnan(qy):
        qy = 0.0
    if math.isnan(qz):
        qz = 0.0
    if math.isnan(qw):
        qw = 1.0
    return ([qx, qy, qz, qw])


def main():

    # Initialize the rclpy library
    rclpy.init(args=None)

    # Create a subscriber
    radar_subscriber = RadarSubscriber()

    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    rclpy.spin(radar_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    radar_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
