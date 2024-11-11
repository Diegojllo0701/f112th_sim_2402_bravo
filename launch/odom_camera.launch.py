from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': False
    }]

    remappings = [
        ('imu', '/imu/data_raw'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/depth/image_rect_raw')
    ]

    return LaunchDescription([

        # RGBD Odometry Node - Running on the PC
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),

    ])
