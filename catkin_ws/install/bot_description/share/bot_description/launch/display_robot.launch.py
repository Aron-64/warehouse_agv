import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

    
def generate_launch_description():
    #获取默认的URDF文件路径
    urdf_package_path = get_package_share_directory('bot_description')
    urdf_path = os.path.join(urdf_package_path, 'urdf/robot', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    
    #声明URDF文件路径的Launch参数
    action_declare_urdf_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=urdf_path,
        description='加载的模型文件路径'
    )
    
    # #通过文件路径，获取内容，并转换成参数值对象，以供转入robot_state_publisher节点
    # substitutions_command_result = launch.substitutions.Command([
    #     'cat ', 
    #      launch.substitutions.LaunchConfiguration('model')
    # ])
    #通过文件路径，xacro获取内容，并转换成参数值对象，以供转入robot_state_publisher节点
    substitutions_command_result = launch.substitutions.Command([
        'xacro ', 
         launch.substitutions.LaunchConfiguration('model')
    ])

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': substitutions_command_result}]
    )
    
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
     
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )
        
    return launch.LaunchDescription([
        action_declare_urdf_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])