#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取表格抽屉世界文件的路径
    world_file = os.path.join(get_package_share_directory('table_drawer'), 'world.sdf')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 启动Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )
    
    # 读取机器人描述
    robot_description = open(os.path.join(get_package_share_directory('kortex_description'), 'robots/gen3_2f85.urdf'), 'r').read()
    
    # 启动机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # 启动关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 启动控制器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
            os.path.join(get_package_share_directory('grasp_training'), 'config/gen3_controllers.yaml')
        ],
        output='screen'
    )
    
    # 加载和启动控制器
    load_controllers = []
    for controller in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']:
        load_controllers.append(
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
                output='screen'
            )
        )
    
    # 启动抓取训练节点
    grasp_training_node = Node(
        package='grasp_training',
        executable='grasp_cube',
        name='grasp_training',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))
    
    # 添加动作
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    
    # 添加控制器
    for controller in load_controllers:
        ld.add_action(controller)
    
    # 等待Gazebo启动完成后再启动抓取训练节点
    start_grasp_training = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[grasp_training_node]
        )
    )
    ld.add_action(start_grasp_training)
    
    return ld 