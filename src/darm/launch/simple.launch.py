import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    package_name = 'bicopter'
    pkg_share = get_package_share_directory(package_name)
    yaml_file = os.path.join(pkg_share, 'config', 'bicopter_controllers.yaml')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    '''--------------------------------------------------------------------------------'''
    # # to setup gazebo and load the sdf model. 

    # GZ path
    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    # World
    world_file = os.path.join(pkg_share, 'worlds', 'bicopter_world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    #minimal gazebo launch
    # gazebo = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
    #         launch_arguments={'gz_args': '-r empty.sdf'}.items()
    # )
    '''---------------------------------------------------------------------------------'''
    # to setup robot state publisher and controllers.
    # Process Xacro → robot_description param
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    # Spawn the robot in GZ -> no need for this, PX4 auto takes care of this. 
    # spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', 'bicopter',
    #         '-topic', 'robot_description',
    #         '-x', '0',
    #         '-y', '0',
    #         '-z', '0.2',  # <--- 20 cm above ground
    #     ],
    #     output='screen'


    # )
   
    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[yaml_file],
        output='screen',
        emulate_tty=True,
        prefix=['xterm -e gdb -ex run --args']  
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )


    left_tilt_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_tilt_position_controller'],
        output='screen'
    )

    right_tilt_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_tilt_position_controller'],
        output='screen'
    )

    ignbridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            "/world/default/model/x500_0/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model",
        ],
        remappings=[
            ('/world/bicopter/model/bicopter/joint_state', '/joint_states'),

        ],
        output='screen'
    )

  
    '''---------------------------------------------------------------------------------'''
    # to setup RViz for visualization.


    # RViz
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
    
    '''---------------------------------------------------------------------------------'''

    return LaunchDescription([
        set_gz_path,
        gazebo,
        rsp,
        control_node,
        joint_state_broadcaster_spawner,
        left_tilt_spawner,
        right_tilt_spawner,
        ignbridge,  
        rviz,
        *launch_env_vars
    ])