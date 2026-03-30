import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_package_path = get_package_share_directory('bot_description')
    gazebo_package_path = get_package_share_directory('bot_gazebo')
    default_xacro_path = os.path.join(
        urdf_package_path, 
        'urdf', 'robot', 
        'robot.urdf.xacro'
    )
    default_gazebo_config_path = os.path.join(
        gazebo_package_path, 
        'world', 
        'warehouse/warehouse2.world'
    )
    
    action_declare_urdf_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_xacro_path,
        description='Path to robot xacro file'
    )
    
    substitutions_command_result = launch.substitutions.Command([
        'xacro ',
        launch.substitutions.LaunchConfiguration('model')
    ])
    
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': substitutions_command_result},
            {'use_sim_time': True},
        ]
    )
    
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(   
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments=[
            ('world', default_gazebo_config_path),
            ('verbose', 'true')
        ]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    
    action_load_joint_state_broadcaster = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller bot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )
    
    # action_load_effort_controller = launch.actions.ExecuteProcess(
    #     cmd='ros2 control load_controller bot_effort_controller --set-state active'.split(' '),
    #     output='screen'
    # )
    
    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller bot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_urdf_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_broadcaster],
            )
        ),
    #    launch.actions.RegisterEventHandler(
    #         event_handler=launch.event_handlers.OnProcessExit(
    #             target_action=action_load_joint_state_broadcaster,
    #             on_exit=[action_load_effort_controller],
    #         )
    #     ),
       launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_broadcaster,
                on_exit=[action_load_diff_drive_controller],
            )
        ),
       
    ])