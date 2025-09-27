from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'package_pickup_delivery'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'transforms3d',
        'transformations',
    ],
    zip_safe=True,
    maintainer='AI Innovation Lab, RMIT University',
    maintainer_email='timothy.wiley@rmit.edu.au',
    description='The aiil_rosbot_demo package',
    license='RMIT IP - Not for distribution',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"aruco_detector = {package_name}.aruco_detector_node:main",
            f"tf_pose_transformer = {package_name}.tf_pose_transformer_node:main",
            f"marker_touch_verifier = {package_name}.marker_touch_verifier_node:main",
            f"task_planner = {package_name}.task_planner_node:main",
            f"autonomous_explorer = {package_name}.explorer_node:main",
            f"visual_servo_pointer = {package_name}.visual_servo_pointer_node:main",
            f"status_tracker_node = {package_name}.status_tracker_node:main",

        ],
    },
)
