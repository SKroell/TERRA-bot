import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'odrive_terra_dev'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Fixed comma
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stefan Kroell',
    maintainer_email='stra@di.ku.dk',
    description='TODO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_human = trackers.detect_human:main',
            'detect_human_with_wave = trackers.detect_human_with_wave:main',
            'detect_human_yolo = trackers.detect_human_yolo:main',
            'detect_human_yolo_wave = trackers.detect_human_yolo_wave:main',
            'detect_aruco = trackers.detect_aruco:main',
            'base_detection = trackers.base_detection',
            # 'drive_square = odrive_terra_dev.ball_tracker.drive_square:main',  # Uncomment if needed
        ],
    },
)
