import importlib.util
import os
import unittest

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def _package_share_dir() -> str:
    try:
        return get_package_share_directory('norby_webots_drivers')
    except PackageNotFoundError:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def _load_launch_module(share_dir: str, filename: str):
    path = os.path.join(share_dir, 'launch', filename)
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestNorbyIntegration(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    cls.share_dir = _package_share_dir()

  def test_required_package_files_exist(self):
    required = [
      os.path.join(self.share_dir, 'urdf', 'TurtleBot3Burger.urdf'),
      os.path.join(self.share_dir, 'config', 'slam_config.yml'),
      os.path.join(self.share_dir, 'config', 'nav_params.yml'),
      os.path.join(self.share_dir, 'config', 'ros2control.yml'),
      os.path.join(self.share_dir, 'worlds', 'apartment.wbt'),
    ]
    for path in required:
      self.assertTrue(os.path.isfile(path), f'missing file: {path}')

  def test_urdf_declares_webots_sensor_interfaces(self):
    urdf_path = os.path.join(self.share_dir, 'urdf', 'TurtleBot3Burger.urdf')
    with open(urdf_path, encoding='utf-8') as urdf_file:
      urdf = urdf_file.read()
    self.assertIn('reference="gps"', urdf)
    self.assertIn('reference="camera"', urdf)
    self.assertIn('reference="LDS-01"', urdf)
    self.assertIn('webots_ros2_driver::Ros2IMU', urdf)
    self.assertIn('<topicName>/gps_sensor</topicName>', urdf)
    self.assertIn('<topicName>/front_camera/camera_raw</topicName>', urdf)
    self.assertIn('<topicName>/base_cloud</topicName>', urdf)
    self.assertIn('<topicName>/imu</topicName>', urdf)

  def test_apartment_world_contains_required_devices(self):
    world_path = os.path.join(self.share_dir, 'worlds', 'apartment.wbt')
    with open(world_path, encoding='utf-8') as world_file:
      world = world_file.read()
    for token in (
      'GPS {',
      'InertialUnit {',
      'Accelerometer {',
      'Gyro {',
      'name "camera"',
      'RobotisLds01 {',
      'name "accelerometer(1)"',
      'name "gyro(1)"',
    ):
      self.assertIn(token, world, f'missing world device marker: {token}')

  def test_world_externproto_pinned_to_r2025a(self):
    worlds_dir = os.path.join(self.share_dir, 'worlds')
    for filename in os.listdir(worlds_dir):
      if not filename.endswith('.wbt'):
        continue
      with open(os.path.join(worlds_dir, filename), encoding='utf-8') as world_file:
        content = world_file.read()
      if 'EXTERNPROTO' not in content:
        continue
      self.assertNotIn('/develop/projects/', content, filename)
      self.assertNotIn('/R2023b/projects/', content, filename)
      self.assertIn('R2025a', content, filename)

  def test_slam_config_uses_expected_scan_topic(self):
    config_path = os.path.join(self.share_dir, 'config', 'slam_config.yml')
    with open(config_path, encoding='utf-8') as config_file:
      config = config_file.read()
    self.assertIn('scan_topic: /laser_scan', config)

  def test_launch_descriptions_generate(self):
    for launch_file in (
      'controllers.launch.py',
      'slam.launch.py',
      'navigation.launch.py',
      'webots_only.launch.py',
      'robot.launch.py',
    ):
      module = _load_launch_module(self.share_dir, launch_file)
      description = module.generate_launch_description()
      self.assertIsNotNone(description, launch_file)


if __name__ == '__main__':
  unittest.main()
