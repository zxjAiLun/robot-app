#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
import numpy as np
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Vector3
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
import time

class DomainRandomizer(Node):
    """
    域随机化节点，用于在仿真环境中随机化各种参数
    """
    def __init__(self):
        super().__init__('domain_randomizer')
        
        # 创建服务客户端
        self.set_entity_state_client = self.create_client(SetEntityState, '/world/table_world/set_entity_state')
        self.pause_physics_client = self.create_client(Empty, '/world/table_world/pause_physics')
        self.unpause_physics_client = self.create_client(Empty, '/world/table_world/unpause_physics')
        
        # 等待服务可用
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_entity_state 服务...')
        
        # 定义随机化参数范围
        self.light_intensity_range = (0.6, 1.0)  # 光照强度范围
        self.camera_position_noise = 0.02  # 相机位置噪声 (±2cm)
        self.camera_angle_noise = 0.1  # 相机角度噪声 (±0.1弧度)
        self.friction_range = (0.8, 1.2)  # 摩擦系数范围
        self.table_position_noise = 0.05  # 桌子位置噪声 (±5cm)
        
        # 创建定时器，每10秒随机化一次环境
        self.timer = self.create_timer(10.0, self.randomize_environment)
        self.get_logger().info('域随机化节点已启动')
    
    def randomize_environment(self):
        """随机化整个环境"""
        self.get_logger().info('开始随机化环境...')
        
        # 暂停物理引擎以进行修改
        self.pause_physics()
        
        # 执行各种随机化
        self.randomize_lighting()
        self.randomize_camera_pose()
        self.randomize_table_pose()
        
        # 恢复物理引擎
        self.unpause_physics()
        self.get_logger().info('环境随机化完成')
    
    def pause_physics(self):
        """暂停物理引擎"""
        req = Empty.Request()
        future = self.pause_physics_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
    
    def unpause_physics(self):
        """恢复物理引擎"""
        req = Empty.Request()
        future = self.unpause_physics_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
    
    def randomize_lighting(self):
        """随机化光照条件"""
        self.get_logger().info('随机化光照...')
        
        # 随机光照强度
        intensity = random.uniform(*self.light_intensity_range)
        
        # 随机光照方向
        direction_x = random.uniform(-0.7, -0.3)
        direction_y = random.uniform(-0.1, 0.3)
        direction_z = random.uniform(-1.0, -0.8)
        
        # 设置光源状态
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'sun'
        req.state.pose = Pose()
        
        # 设置光源属性 (通过自定义消息或服务可能需要额外实现)
        # 这里我们只是记录日志，实际实现可能需要使用Gazebo的特定API
        self.get_logger().info(f'设置光照强度: {intensity}, 方向: [{direction_x}, {direction_y}, {direction_z}]')
    
    def randomize_camera_pose(self):
        """随机化相机位置和角度"""
        self.get_logger().info('随机化相机位置...')
        
        # 获取相机的基础位置和方向
        base_x = 0.0
        base_y = -0.0675
        base_z = -0.05044
        
        # 添加随机噪声
        noise_x = random.uniform(-self.camera_position_noise, self.camera_position_noise)
        noise_y = random.uniform(-self.camera_position_noise, self.camera_position_noise)
        noise_z = random.uniform(-self.camera_position_noise, self.camera_position_noise)
        
        # 随机化角度
        roll = 0.0
        pitch = 1.5708 + random.uniform(-self.camera_angle_noise, self.camera_angle_noise)
        yaw = -1.5708 + random.uniform(-self.camera_angle_noise, self.camera_angle_noise)
        
        self.get_logger().info(f'相机位置噪声: [{noise_x}, {noise_y}, {noise_z}], 角度噪声: [{roll}, {pitch-1.5708}, {yaw+1.5708}]')
        
        # 实际修改相机位置需要修改URDF或使用Gazebo的API
        # 这里我们只是记录日志，实际实现可能需要额外的步骤
    
    def randomize_table_pose(self):
        """随机化桌子的位置"""
        self.get_logger().info('随机化桌子位置...')
        
        # 基础位置
        base_x = 0.8
        base_y = 0.0
        base_z = 0.0
        
        # 添加随机噪声
        noise_x = random.uniform(-self.table_position_noise, self.table_position_noise)
        noise_y = random.uniform(-self.table_position_noise, self.table_position_noise)
        
        # 设置桌子状态
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'table_with_drawers'
        req.state.pose = Pose()
        req.state.pose.position.x = base_x + noise_x
        req.state.pose.position.y = base_y + noise_y
        req.state.pose.position.z = base_z
        
        # 发送请求
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info(f'桌子位置设置为: [{base_x + noise_x}, {base_y + noise_y}, {base_z}]')

def main(args=None):
    rclpy.init(args=args)
    randomizer = DomainRandomizer()
    
    try:
        rclpy.spin(randomizer)
    except KeyboardInterrupt:
        pass
    finally:
        randomizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 