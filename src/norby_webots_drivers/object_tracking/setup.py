import os
import glob
from glob import glob
from setuptools import setup

package_name = 'object_tracking'

setup(
   name=package_name,
   version='0.0.0',
   packages=[package_name],
   py_modules=[package_name + '.track_objects'],
   data_files=[
      ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
      (os.path.join('share', package_name, 'configs'), glob(package_name + '/configs/' + '*')),
      (os.path.join('share', package_name, 'models'), glob(package_name + '/models/' + '*.data')),
      (os.path.join('share', package_name, 'models', 'data'), glob(package_name + '/models/data/' + '*.list')),
      (os.path.join('share', package_name, 'models', 'data'), glob(package_name + '/models/data/' + '*.tree')),
      (os.path.join('share', package_name, 'models', 'data'), glob(package_name + '/models/data/' + '*.txt')),
      (os.path.join('share', package_name, 'models', 'data'), glob(package_name + '/models/data/' + '*.names')),
      (os.path.join('share', package_name, 'models', 'data'), glob(package_name + '/models/data/' + '*.maps')),
      (os.path.join('share', package_name, 'models', 'data', 'labels'), glob(package_name + '/models/data/labels/' + '*')),
      (os.path.join('share', package_name, 'weights'), glob(package_name + '/weights/' + '*'))
   ],
   install_requires=['setuptools'],
   zip_safe=True,
   maintainer='frantzcito',
   maintainer_email='frantzcito@todo.todo',
   description='TODO: Package description',
   license='TODO: License declaration',
   tests_require=['pytest'],
   entry_points={
      'console_scripts': [
      ],
   },
)
