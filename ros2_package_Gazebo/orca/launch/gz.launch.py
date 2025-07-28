  #!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    orca_path = get_package_share_directory("orca")
    world_file = LaunchConfiguration("world_file", default = join(orca_path, "worlds", "orca.sdf"))

    # position_x = LaunchConfiguration("position_x")
    # position_y = LaunchConfiguration("position_y")
    # orientation_yaw = LaunchConfiguration("orientation_yaw")
    # camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    # stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    # two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    # odometry_source = LaunchConfiguration("odometry_source")

    # robot_description_content = get_xacro_to_doc(
    #     join(orca_path, "worlds", "orca.urdf"),
    #     {"sim_gz": "true",
    #      "two_d_lidar_enabled": "true",
    #      "conveyor_enabled": "false",
    #      "camera_enabled": "true"
    #     }
    # ).toxml()

    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     parameters=[{"use_sim_time": use_sim_time},
    #                 {'robot_description': Command( \
    #                 ['xacro ', join(orca_path, 'worlds/orca.urdf'),
    #                 # ' camera_enabled:=', camera_enabled,
    #                 # ' stereo_camera_enabled:=', stereo_camera_enabled,
    #                 # ' two_d_lidar_enabled:=', two_d_lidar_enabled,
    #                 ' odometry_source:=', odometry_source,
    #                 ' sim_gz:=', "true"
    #                 ])}],
    #     remappings=[
    #         ('/joint_states', 'orca/joint_states'),
    #     ]
    # )
 
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    # gz_spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         "-topic", "/robot_description",
    #         "-name", "orca",
    #         "-allow_renaming", "true",
    #         "-z", "0.28",
    #         "-x", position_x,
    #         "-y", position_y,
    #         "-Y", orientation_yaw
    #     ]
    # )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # "/model/orca/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/model/Orca/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/Orca/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose",
            "/model/Orca/joint/propeller0_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/Orca/joint/propeller1_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/Orca/joint/propeller2_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/Orca/joint/propeller3_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/Orca/joint/propeller4_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/Orca/joint/propeller5_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/Orca/joint/propeller6_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/Orca/joint/propeller7_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double",
            # "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            
            # "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            # "stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            
            # "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # "/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/world/default/model/Orca/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/Orca/joint_state', 'Orca/joint_states'),
            ('/model/Orca/odometry', 'Orca/odom'),
            # ('/scan', 'Orca/scan'),
            ("/model/Orca/joint/propeller0_joint/cmd_thrust","Orca/propeller0"),
            ("/model/Orca/joint/propeller1_joint/cmd_thrust","Orca/propeller1"),
            ("/model/Orca/joint/propeller2_joint/cmd_thrust","Orca/propeller2"),
            ("/model/Orca/joint/propeller3_joint/cmd_thrust","Orca/propeller3"),
            ("/model/Orca/joint/propeller4_joint/cmd_thrust","Orca/propeller4"),
            ("/model/Orca/joint/propeller5_joint/cmd_thrust","Orca/propeller5"),
            ("/model/Orca/joint/propeller6_joint/cmd_thrust","Orca/propeller6"),
            ("/model/Orca/joint/propeller7_joint/cmd_thrust","Orca/propeller7"),

            ('/camera', 'Orca/camera'),
            # ('/stereo_camera/left/image_raw', 'Orca/stereo_camera/left/image_raw'),
            # ('/stereo_camera/right/image_raw', 'Orca/stereo_camera/right/image_raw'),
            ('/imu', 'Orca/imu'),
            # ('model/Orca/cmd_vel', 'Orca/cmd_vel'),
            ('camera_info', 'Orca/camera/camera_info'),
            # ('stereo_camera/left/camera_info', 'Orca/stereo_camera/left/camera_info'),
            # ('stereo_camera/right/camera_info', 'Orca/stereo_camera/right/camera_info'),
            # ('/rgbd_camera/points', 'Orca/rgbd_camera/points'),
        ]
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "rgbd_camera",
                    "--child-frame-id", "Orca/rgbd_camera"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        # DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        # DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        # DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        # DeclareLaunchArgument("position_x", default_value="0.0"),
        # DeclareLaunchArgument("position_y", default_value="0.0"),
        # DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        # robot_state_publisher,
            gz_sim, 
            # gz_spawn_entity,
          gz_ros2_bridge,
        transform_publisher
    ])