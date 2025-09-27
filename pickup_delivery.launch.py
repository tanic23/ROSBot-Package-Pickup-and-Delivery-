# pickup_delivery.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '0.0','0.0','0.10',
                '0','0','0',
                'base_link','camera_link'
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_pointer_tip',
            arguments=[
                '0.255', '0.0', '0.08',   # ← updated to your 25.5 cm pointer
                '0', '0', '0',
                'base_link', 'pointer_tip'
            ],
            output='screen',
        ),
        # ArUco marker detector (unchanged)
        Node(
            package='package_pickup_delivery',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[{
                'image_topic':       '/oak/rgb/image_raw',
                'camera_info_topic': '/oak/rgb/camera_info',
                'marker_size':        0.095,
                'aruco_dictionary':   'DICT_4X4_50',
            }],
            remappings=[
                ('/camera/image_raw',   '/oak/rgb/image_raw'),
                ('/camera/camera_info', '/oak/rgb/camera_info'),
            ],
            output='screen',
        ),

        # TF Pose Transformer (unchanged)
        Node(
            package='package_pickup_delivery',
            executable='tf_pose_transformer',
            name='tf_pose_transformer',
            parameters=[{
                'input_pose_topic':  '/aruco/pose',
                'input_id_topic':    '/aruco/id',
                'output_pose_topic': '/aruco/map_pose',
                'target_frame':      'map',
            }],
            output='screen',
        ),

        # Task Planner (unchanged)
        Node(
            package='package_pickup_delivery',
            executable='task_planner',
            name='task_planner',
            output='screen',
        ),

        # Visual Servo Pointer (updated to use /aruco/map_pose & pointer length)
        Node(
            package='package_pickup_delivery',
            executable='visual_servo_pointer',
            name='visual_servo_pointer',
            parameters=[{
                'pose_topic':     '/aruco/map_pose',
                'cmd_vel_topic':  '/cmd_vel',
                'xy_gain':        0.7,
                'pointer_length': 0.255,
                'stop_tolerance': 0.02,
            }],
            output='screen',
        ),

        # Status Tracker Node (unchanged)
        Node(
            package='package_pickup_delivery',
            executable='status_tracker_node',
            name='status_tracker_node',
            output='screen',
        ),
        # Autonomous Explorer Node
        Node(
            package='package_pickup_delivery',
            executable='autonomous_explorer',
            name='explorer_node',
            output='screen',
        ),

    ])

