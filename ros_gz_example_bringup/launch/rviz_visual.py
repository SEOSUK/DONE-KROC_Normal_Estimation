from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 기존 URDF 파일 로드
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='manipulator_state_publisher',
            arguments=['/home/mrlseuk/ros2_ws/src/ros_gz_project_template/ros_gz_example_description/models/manipulator/model.urdf'],
        ),


        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])

