from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform publisher between base_link and camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub',
            output='screen',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'base_link', 'camera_link'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_pub2',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),
        
        # RTAB-Map node
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='rtabmap',
            output='screen',
            parameters=[{
                'rtabmap_args': '--delete_db_on_start --Vis/CorFlowMaxLevel 5 --Stereo/MaxDisparity 300',
                'frame_id': 'base_link',
                'approx_sync': True,
                'approx_sync_max_interval': 0.01,
                'sync_queue_size': 20,
                'qos': 2,
                # 'subscribe_stereo': True,
                # more args: --Stereo/FlowMaxLevel 5 --Vis/MinInliers 10
                'log_level': 'debug'
            }],
        ),
    ])
