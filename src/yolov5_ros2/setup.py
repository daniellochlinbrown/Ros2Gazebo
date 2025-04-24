from setuptools import setup
from glob import glob

package_name = 'yolov5_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name + '/models', glob('models/*.pt'))
    ],
    install_requires=[ 'setuptools',
    'torch',
    'rospkg',
    'cv_bridge',],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='YOLOv5 ROS2 Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolov5_ros2.yolo_node:main',
        ],
    },
)
