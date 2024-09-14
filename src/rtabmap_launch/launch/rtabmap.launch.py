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
            arguments=['0', '0', '0', '1.5708', '0', '1.5708', 'base_link', 'camera_link'],
        ),
        
        # RTAB-Map node
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'stereo': True,
                'left_image_topic': '/camera/left/image_rect',
                'right_image_topic': '/camera/right/image_rect',
                'imu_topic': '/imu',
                'wait_imu_to_init': True,
                'rtabmap_args': '--delete_db_on_start --Vis/CorFlowMaxLevel 5 --Stereo/MaxDisparity 200',
            }],
        ),
    ])
