import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Base Controller Node
    base_controller_node = Node(
        package='robot_base',
        executable='base_controller',
        name='base_controller',
        output='screen',
        parameters=[{
            'wheel_diameter': 0.083,  # From URDF: wheel_radius 0.0415
            'wheel_base_width': 0.205, # From URDF: distance between left and right wheels
            'ticks_per_rev': 340.0,
            'serial_port': '/dev/ttyTHS0',
            'baud_rate': 115200,
            'max_pwm': 255,
            'speed_to_pwm_ratio': 200.0,
        }]
    )

    # RealSense Camera Node
    # Needs realsense2_camera package installed
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'false', # RTAB-Map creates pointcloud from depth
        }.items()
    )

    # RTAB-Map SLAM Node
    rtabmap_node = Node(
        package='rtabmap_ros', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'visual_odometry': False, # Use wheel odometry
            'odom_frame_id': 'odom',
            'database_path': '~/.ros/rtabmap.db',
            
            # RTAB-Map parameters tuned for Jetson Xavier NX
            'Rtabmap/TimeThr': '700',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
            'Grid/RangeMax': '4.0',
            'Mem/UseOdomGravity': 'true',
            'Optimizer/GravitySigma': '0.3',
            'Reg/Strategy': '0', # Visual registration
            'Reg/Force3DoF': 'true', # 2D mapping
        }],
        remappings=[
            ('odom', '/odom'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw')
        ],
        arguments=['--delete_db_on_start']
    )

    return LaunchDescription([
        robot_state_publisher_node,
        base_controller_node,
        realsense_node,
        rtabmap_node
    ])
