import rclpy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Range, Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
import math
from math import cos, isnan, sin
import os


class BaseController:

	def __init__(self, webots_node, properties):
		self.robot = webots_node.robot
		# Wheels
		self.left_front_motor = self.robot.getDevice(
			'left_front_rotational_motor')
		self.right_front_motor = self.robot.getDevice(
			'right_front_rotational_motor')
		self.left_back_motor = self.robot.getDevice(
			'left_back_rotational_motor')
		self.right_back_motor = self.robot.getDevice(
			'right_back_rotational_motor')

		self.left_front_wheel_sensor = self.robot.getDevice(
			'left_front_position_sensor')
		self.right_front_wheel_sensor = self.robot.getDevice(
			'right_front_position_sensor')
		self.left_back_wheel_sensor = self.robot.getDevice(
			'left_back_position_sensor')
		self.right_back_wheel_sensor = self.robot.getDevice(
			'right_back_position_sensor')

		self.wheels = ['left_front_rotational_motor', 'right_front_rotational_motor',
					   'right_back_rotational_motor', 'left_back_rotational_motor']

		self.motor_max_speed = self.left_front_motor.getMaxVelocity()
		# Wheel Encoders
		self.initWheels()
		self.cmdVelSubscriber = self.create_subscription(Twist, 'cmd_vel',
														 self.cmdVel_callback, 10)
		pkg_path = get_package_share_directory(
			'norby_webots_drivers')
		with open(os.path.join(pkg_path, 'webots.urdf'), 'w') as file:
			file.write(self.robot.getUrdf())
		# Odom
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vth = 0.0
		self.time_step = 0.032
		self.sampling_period = 50
		self.left_omega = 0.0
		self.right_omega = 0.0
		self.prev_angle = 0.0
		self.prev_left_wheel_ticks = 0.0
		self.prev_right_wheel_ticks = 0.0
		self.rho = 0.0
		self.theta = 0.0
		self.phi = 0.0
		self.wheel_gap = 0.12
		self.wheel_radius = 0.04
		self.front_back = 0.1
		self.last_time = 0.1
		self.odom_pub = self.create_publisher(Odometry, "odom", 1)
		self.odom_timer = self.create_timer(self.time_step, self.odom_callback)

		# enable sensors
		self.sensors = ['lidar_sensor', 'camera_sensor', 'gps_sensor', 'accelerometer_sensor', 'gyro_sensor', 'inertial_unit_sensor',
										'sonar_front_left_sensor', 'sonar_front_right_sensor', 'sonar_back_left_sensor', 'sonar_back_right_sensor',
										'left_front_position_sensor', 'right_front_position_sensor',
										'left_back_position_sensor', 'right_back_position_sensor',
										'back_left_radar_sensor', 'back_right_radar_sensor', 'front_left_radar_sensor', 'front_right_radar_sensor']
		self.sonar_sensors = [
			sensor for sensor in self.sensors if 'sonar' in sensor]
		self.radar_sensors = [
			sensor for sensor in self.sensors if 'radar' in sensor]

		self.sonar_ind = 0
		self.radar_ind = 0
		self.devices = []
		for sensor in range(self.robot.getNumberOfDevices()):
			device = (self.robot.getDeviceByIndex(sensor))
			name = (device.getName())
			
			print(name)
			print(device.getNodeType())
			# == Node.CAMERA
			if ('motor' not in name):
				device.enable(self.sampling_period)
				if('lidar' in name ):
					device.enablePointCloud()
			self.devices.append(device)
		for sensor in self.sensors:

			self.enable_sensor(sensor)

		# Lidar & Laser
		self.lidar_publisher = self.create_publisher(
			PointCloud2, '/base_cloud', 1)
		self.laser_publisher = self.create_publisher(
			LaserScan, '/laser_scan', 1)
		self.lidar_sensor = self.robot.getDevice('lidar_sensor')
		self.lidar_sensor.enablePointCloud()

		self.lidar_timer = self.create_timer(self.time_step, self.lidar_pub)
		self.laser_timer = self.create_timer(self.time_step, self.laser_pub)

		# Sonar
		self.sonar_publisher = self.create_publisher(Range, '/base_sonar', 1)
		#self.create_timer(self.timestep * 1e-3, self.sonar_pub)

		# Camera
		self.cam_publisher = self.create_publisher(
			Image, '/front_camera/camera_raw', 1)
		#self.create_timer(self.timestep * 1e-3, self.camera_pub)

		# Radar
		self.radar_publisher = self.create_publisher(
			RadarScan, '/base_radar', 1)
		#self.create_timer(self.timestep * 1e-3, self.radar_pub)
	def enable_sensor(self, name):
		samplingPeriod = 50
		self.robot.getDevice(name).enable(samplingPeriod)

	def motor_callback(self, request, response):
		# only two wheels
		if(self.left_back_motor == None):
			self.left_front_motor.setVelocity(request.left_speed * 2)
			self.right_front_motor.setVelocity(request.right_speed * 2)
		# four wheels
		else:
			self.left_front_motor.setVelocity(request.left_speed)
			self.right_front_motor.setVelocity(request.right_speed)
			self.left_back_motor.setVelocity(request.left_speed * 2)
			self.right_back_motor.setVelocity(request.right_speed * 2)
		return response

	def initWheels(self):
		for wheel in self.wheels:
			self.robot.getDevice(wheel).setPosition(float('inf'))
			self.robot.getDevice(wheel).setVelocity(0)
			self.motor_max_speed = self.robot.getDevice(wheel).getMaxVelocity()

	def lidar_pub(self):
		if(not self.lidar_sensor.isPointCloudEnabled()):
			return
		#print(self.lidar_sensor)
		#print(self.lidar_sensor.getSamplingPeriod())
		#print (self.lidar_sensor.getLayerRangeImage(0))
		#return
		msg_lidar = PointCloud2()
		msg_lidar.header.frame_id = 'base_link'
		self.lidar_sensor.enable(50)


		stamp = Time(seconds=self.robot.getTime()).to_msg()
		msg_lidar.header.stamp = stamp
		msg_lidar.height = 1
		msg_lidar.data = self.lidar_sensor.getPointCloud(data_type='buffer')
		msg_lidar.width = self.lidar_sensor.getNumberOfPoints()
		msg_lidar.point_step = 20
		msg_lidar.row_step = 20 * self.lidar_sensor.getNumberOfPoints()
		msg_lidar.is_dense = False
		msg_lidar.fields = [
			PointField(name='x', offset=0,
					   datatype=PointField.FLOAT32, count=1),
			PointField(name='y', offset=4,
					   datatype=PointField.FLOAT32, count=1),
			PointField(name='z', offset=8,
					   datatype=PointField.FLOAT32, count=1)
		]
		msg_lidar.is_bigendian = False
		self.lidar_publisher.publish(msg_lidar)

	def laser_pub(self):
		ranges = self.lidar_sensor.getLayerRangeImage(0)
		if ranges:
			msg = LaserScan()
			stamp = Time(seconds=self.robot.getTime()).to_msg()
			msg.header.stamp = stamp
			msg.header.frame_id = 'base_link'
			msg.angle_min = -0.5 * self.lidar_sensor.getFov()
			msg.angle_max = 0.5 * self.lidar_sensor.getFov()
			msg.angle_increment = self.lidar_sensor.getFov() / ((self.lidar_sensor.getHorizontalResolution() - 1)
																* self.lidar_sensor.getNumberOfLayers())
			msg.scan_time = self.lidar_sensor.getSamplingPeriod() / 1000.0
			msg.range_min = self.lidar_sensor.getMinRange()
			msg.range_max = self.lidar_sensor.getFov()
			msg.ranges = ranges
			self.laser_publisher.publish(msg)

	def odom_callback(self):
		stamp = Time(seconds=self.robot.getTime()).to_msg()
		self.odom_broadcaster = TransformBroadcaster(self)
		time_diff_s = self.robot.getTime() - self.last_time
		left_wheel_ticks = (self.left_front_wheel_sensor.getValue() + self.left_back_wheel_sensor.getValue()) / 2 
		right_wheel_ticks = (self.right_front_wheel_sensor.getValue() + self.right_back_wheel_sensor.getValue()) / 2 
		print('fl_value', self.left_front_wheel_sensor.getValue())
		print('fr_value', self.right_front_wheel_sensor.getValue())
		print('bl_value', self.left_back_wheel_sensor.getValue())
		print('br_value', self.right_back_wheel_sensor.getValue())
		print(self.x)
		print(self.y)
		print(self.th)
		if time_diff_s == 0.0:
			return

		v_left_rad = (left_wheel_ticks -
					  self.prev_left_wheel_ticks) / time_diff_s
		v_right_rad = (right_wheel_ticks -
					   self.prev_right_wheel_ticks) / time_diff_s
		v_left = v_left_rad * self.wheel_radius
		v_right = v_right_rad * self.wheel_radius

		vx = (v_left + v_right) / 2
		vy = 0.0
		omega = (v_right - v_left) / 2 * 4 * self.wheel_gap

		self.x += (vx * cos(self.prev_angle)) * time_diff_s
		if(isnan(self.x)):
			self.x = 0.0
		self.y += (vy * sin(self.prev_angle)) * time_diff_s
		self.th += omega

		# reset section
		self.prev_ange = self.th
		self.prev_left_wheel_ticks = left_wheel_ticks
		self.prev_right_wheel_ticks = right_wheel_ticks
		self.last_time = self.robot.getTime()
		ang_vel = self.robot.getDevice('gyro_sensor').getValues()
		#ang_vel = [0.0,0.0,0.0]
		odom_quat = euler_to_quaternion(ang_vel[0], ang_vel[1], ang_vel[2])
		# tf publish
		odom_transform = TransformStamped()
		odom_transform.header.stamp = stamp
		odom_transform.header.frame_id = 'odom'
		odom_transform.child_frame_id = 'base_link'
		odom_transform.transform.rotation.x = odom_quat[0]
		odom_transform.transform.rotation.y = odom_quat[1]
		odom_transform.transform.rotation.z = odom_quat[2]
		odom_transform.transform.rotation.w = odom_quat[3]
		odom_transform.transform.translation.x = self.x
		odom_transform.transform.translation.y = self.y
		odom_transform.transform.translation.z = 0.0
		self.odom_broadcaster.sendTransform(odom_transform)

		odom = Odometry()
		odom.header.stamp = stamp
		odom.header.frame_id = 'odom'
		odom.child_frame_id = 'base_link'

		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		if math.isnan(odom_quat[0]):
			odom.pose.pose.orientation.x = 0.0
		else:
			odom.pose.pose.orientation.x = odom_quat[0]

		if math.isnan(odom_quat[1]):
			odom.pose.pose.orientation.y = 0.0
		else:
			odom.pose.pose.orientation.y = odom_quat[1]

		if math.isnan(odom_quat[2]):
			odom.pose.pose.orientation.z = 0.0
		else:
			odom.pose.pose.orientation.z = odom_quat[2]

		if math.isnan(odom_quat[3]):
			odom.pose.pose.orientation.w = 1.
		else:
			odom.pose.pose.orientation.w = odom_quat[3]
		odom.twist.twist.linear.x = self.vx
		odom.twist.twist.angular.z = self.vth
		self.th = (left_wheel_ticks + right_wheel_ticks) / 2
		self.odom_pub.publish(odom)

	def camera_pub(self):
		msg_cam = Image()
		msg_cam.header.frame_id = 'base_link'
		stamp = Time(seconds=self.robot.getTime()).to_msg()
		msg_cam.header.stamp = stamp
		msg_cam.encoding = 'bgra8'
		msg_cam.height = self.robot.getDevice('camera_sensor').getHeight()
		msg_cam.width = self.robot.getDevice('camera_sensor').getWidth()
		msg_cam.data = self.robot.getDevice('camera_sensor').getImage()
		self.cam_publisher.publish(msg_cam)

	def sonar_pub(self):
		msg = Range()
		msg.header.stamp = self.get_clock().now().to_msg()
		ind = self.sonar_ind % len(self.sonar_sensors)
		name = self.sonar_sensors[ind]
		sensor = self.robot.getDevice(name)
		msg.header.frame_id = 'base_link'
		table = sensor.getLookupTable()
		msg.field_of_view = sensor.getAperture()
		msg.min_range = min(table[0], table[-3])
		msg.max_range = max(table[0], table[-3])
		msg.range = min(sensor.getValue(), msg.max_range)
		msg.radiation_type = 1
		self.sonar_publisher.publish(msg)
		self.sonar_ind += 1

	def radar_pub(self):
		msg = RadarScan()
		msg.header.stamp = self.get_clock().now().to_msg()
		ind = self.radar_ind % len(self.radar_sensors)
		name = self.radar_sensors[ind]
		msg.header.frame_id = name
		sensor = self.robot.getDevice(name)
		returns = []
		for track in sensor.getTargets():
			ret = RadarReturn()
			ret.doppler_velocity = track.speed
			ret.amplitude = track.received_power
			ret.elevation = 0.0
			ret.azimuth = track.azimuth
			ret.range = track.distance
			returns.append(ret)
		msg.returns = returns
		self.radar_publisher.publish(msg)
		self.radar_ind += 1

	def cmdVel_callback(self, msg):
		self.vx = msg.linear.x
		self.vz = msg.angular.z

		leftSpeed = ((2.0 * msg.linear.x - msg.angular.z * self.wheel_gap) /
					 (2.0 * self.wheel_radius))
		rightSpeed = ((2.0 * msg.linear.x + msg.angular.z * self.wheel_gap) /
					  (2.0 * self.wheel_radius))
		leftSpeed = min(self.motor_max_speed, max(-self.motor_max_speed,
												  leftSpeed))
		rightSpeed = min(self.motor_max_speed, max(-self.motor_max_speed,
												   rightSpeed))
		# only two wheels
		if(self.left_back_motor == None):
			self.left_front_motor.setVelocity(leftSpeed * 2)
			self.right_front_motor.setVelocity(rightSpeed * 2)
		# four wheels
		else:
			self.left_front_motor.setVelocity(leftSpeed)
			self.right_front_motor.setVelocity(rightSpeed)
			self.left_back_motor.setVelocity(leftSpeed * 1.5)
			self.right_back_motor.setVelocity(rightSpeed * 1.5)


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


def main(args=None):
	rclpy.init(args=args)
	exampleController = BaseController(args=args)
	#exampleController.start_device_manager()
	exampleController.start_device_manager({
            'gyro_sensor': {
                'enable': True
            },
				'lidar_sensor': {
                'enable': True
            }
        })
	rclpy.spin(exampleController)


if __name__ == '__main__':
	main()
