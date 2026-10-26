from glob import glob
from setuptools import setup

setup(name='turtlebot_swerve', version='0.1.0', packages=['turtlebot_swerve'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/turtlebot_swerve']),
                  ('share/turtlebot_swerve', ['package.xml']),
                  ('share/turtlebot_swerve/launch', glob('launch/*.launch.py')),
                  ('share/turtlebot_swerve/urdf', glob('assets/m38_arm_description/urdf/*.urdf')),
                  ('share/turtlebot_swerve/meshes', glob('assets/m38_arm_description/meshes/*.stl'))],
      entry_points={'console_scripts': ['swerve_controller = turtlebot_swerve.controller:main']})
