from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic', default='/robot0/odom')
    fix_topic = LaunchConfiguration('fix_topic', default='/fix')
    imu_output_topic = LaunchConfiguration('imu_output_topic', default='/imu/gps_heading')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel_out')  # ✅ 추가
    return LaunchDescription([
        Node(
            package='gps_heading_init',
            executable='gps_heading_init_node',
            name='gps_heading_init',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'fix_topic': fix_topic,
                'imu_output_topic': imu_output_topic,
                'cmd_vel_topic': cmd_vel_topic,
            }]
        )
    ])
