from setuptools import find_packages, setup

package_name = 'student_pubsub_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_all_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monsednuer',
    maintainer_email='ruihaoz73@gmail.com',
    description='ROS2作业',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'publisher1 = student_pubsub_ros2.publisher1:main',
        'publisher2 = student_pubsub_ros2.publisher2:main',
        'subscriber1 = student_pubsub_ros2.subscriber1:main',
        ],
    },
)
