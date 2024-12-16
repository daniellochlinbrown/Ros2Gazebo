from setuptools import setup

package_name = 'yolov5_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
