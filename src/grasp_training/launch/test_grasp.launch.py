#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
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
    
    # 启动测试节点
    test_grasp_node = Node(
        package='grasp_training',
        executable='test_grasp',
        name='test_grasp',
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
    ld.add_action(test_grasp_node)
    
    return ld 