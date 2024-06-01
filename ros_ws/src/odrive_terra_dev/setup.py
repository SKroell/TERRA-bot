from setuptools import setup
from glob import glob

package_name = 'odrive_terra_dev'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        #('share/ament_index/resource_index/packages',
        #    ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
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
            'detect_aruco = trackers.detect_aruco:main',
            'drive_square = ball_tracker.drive_square:main',
        ],
    },
)
