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
    
    # 确保gz可以找到模型
    os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join([my_table_model_path, os.environ.get("GZ_SIM_RESOURCE_PATH", "")])

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

    # 创建简单的盘子SDF模型
    plate_sdf = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="plate">
    <static>false</static>
    <link name="plate">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.01</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 1.0 1.0 1</ambient>
          <diffuse>1.0 1.0 1.0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.01</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.5</mu>
              <mu2>0.5</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.005</ixx>
          <iyy>0.005</iyy>
          <izz>0.0072</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>"""

    # 将SDF内容写入临时文件
    plate_sdf_path = os.path.join(pkg_table_plate_demo, "plate_temp.sdf")
    with open(plate_sdf_path, "w") as f:
        f.write(plate_sdf)

    # 生成盘子的spawn命令
    spawn_plate = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'plate',
            '-file', plate_sdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.335',  # 桌子高度0.3 + 桌子厚度0.03 + 一点点间隙
        ],
        output='screen',
    )

    # 加载修复后的机器人URDF文件
    robot_description_path = os.path.join(
        os.getcwd(), 'src', 'table_plate_demo', 'urdf', 'kinova_gen3_fixed.urdf'
    )
    
    # 直接读取URDF文件
    with open(robot_description_path, 'r') as file:
        robot_description_content = file.read()
    
    robot_description = {"robot_description": robot_description_content}
    
    # 机器人状态发布
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 关节状态发布
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description]
    )
    
    # 关节状态发布GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # 加载机器人模型到Gazebo
    spawn_kinova = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'kinova_gen3',
            '-x', '0.4',  # 将机械臂放置在桌子旁边
            '-y', '0.0',  # 可以根据需要调整机械臂的y坐标
            '-z', '0.3',  # 保持机械臂的高度与桌子相同
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        spawn_table,
        spawn_plate,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        spawn_kinova,
    ])
