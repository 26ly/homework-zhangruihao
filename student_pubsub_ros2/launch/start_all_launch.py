from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 发布者1：学生信息
        Node(
            package='student_pubsub_ros2',
            executable='publisher1',
            name='student_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{'format_type': 'json'}]  # 可以传递参数
        ),
        
        # 发布者2：ROS想法
        Node(
            package='student_pubsub_ros2',
            executable='publisher2',
            name='thoughts_publisher',
            output='screen',
            emulate_tty=True,
        ),
        
        # 订阅者
        Node(
            package='student_pubsub_ros2',
            executable='subscriber',
            name='multi_topic_subscriber',
            output='screen',
            emulate_tty=True,
        ),
    ])
