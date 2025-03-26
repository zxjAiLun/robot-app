#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取当前包的目录
    pkg_table_plate_demo = get_package_share_directory('table_plate_demo')
    
    # 获取ros-gz-sim的启动文件
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    
    # 设置模型路径
    my_table_model_path = os.path.join(os.getcwd(), 'src', 'my_table_model')
    my_plate_model_path = os.path.join(os.getcwd(), 'src', 'my_plate_model')
    
    # 确保gz可以找到模型
    os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join([my_table_model_path, my_plate_model_path, os.environ.get("GZ_SIM_RESOURCE_PATH", "")])
    
    # 生成桌子的spawn命令
    spawn_table = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'table',
            '-file', os.path.join(my_table_model_path, 'table.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ],
        output='screen',
    )
    
    # 生成盘子的spawn命令
    spawn_plate = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'plate',
            '-file', os.path.join(my_plate_model_path, 'plate.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.335',  # 桌子高度0.3 + 桌子厚度0.03 + 一点点间隙
        ],
        output='screen',
    )
    
    return LaunchDescription([
        gz_sim,
        spawn_table,
        spawn_plate,
    ]) 