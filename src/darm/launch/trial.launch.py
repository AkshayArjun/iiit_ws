import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import xacro

def generate_launch_description():


    package_name = 'darm'
    pkg_project_description = get_package_share_directory(package_name)


    xacro_file = os.path.join(pkg_project_description, 'urdf', 'x500_base', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)

    # ros_gz_share = get_package_share_directory('ros_gz_sim')

    
    # world_file = os.path.join(pkg_project_description, 'worlds', 'test.sdf')
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
    #     ]),
    #     launch_arguments={'gz_args': f'-r {world_file}'}.items()
    # )

    robot_desc = doc.toxml().replace(
        "package://darm",
        pkg_project_description
    )



    # params = {'robot_description': doc.toxml(), 'use_sim_time': True}
    params = {'robot_description': robot_desc, 'use_sim_time': True}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('darm'),
            'config',
            'arm_p_control.yaml',
        ]
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    use_gpu = os.environ.get('USE_GPU', '1') == '1'
    launch_env_vars = []
    if use_gpu:
        launch_env_vars.extend([
            SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
            SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        ])

    # gz_spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     parameters=[{'use_sim_time': True}],
    #     arguments=[
    #         '-name', 'x500_base',
    #         '-topic', 'robot_description',
    #         '-x', '0',
    #         '-y', '0',
    #         '-z', '0.2',  # <--- 20 cm above ground
    #     ],
    #     output='screen'
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    #     output='screen'
    # )


    # control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[yaml_file],
    #     output='screen',
    #     emulate_tty=True
    # )
    # left_tilt_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['yaw_position_controller'],
    #     output='screen'
    # )

    # right_tilt_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['pitch_position_controller'],
    #     output='screen'
    # )
    spawners = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            ),
            # change <controller type_<controller_type>_controller as needed here: 
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['yaw_position_controller', '--param-file' , robot_controllers],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['pitch_position_controller', '--param-file', robot_controllers],
                output='screen'
            )
        ]
    )


    gzbridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        "/world/default/model/x500_0/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
    ],
    remappings=[
        ('/world/default/model/x500_0/joint_state', '/joint_states'),
    ],
    )
    return LaunchDescription([
        *launch_env_vars,
        rsp,
        rviz,
        gzbridge,
        spawners
        # gazebo,
        # gz_spawn_entity,
        # control_node,
        # joint_state_broadcaster_spawner,
        # left_tilt_spawner,
        # right_tilt_spawner,
        
    ])

