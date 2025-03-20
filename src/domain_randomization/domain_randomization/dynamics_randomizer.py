#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import numpy as np
import xml.etree.ElementTree as ET
import os
import subprocess
from std_srvs.srv import Empty

class DynamicsRandomizer(Node):
    """
    动力学随机化节点，用于随机化机器人的物理参数
    """
    def __init__(self):
        super().__init__('dynamics_randomizer')
        
        # 创建服务客户端
        self.pause_physics_client = self.create_client(Empty, '/world/table_world/pause_physics')
        self.unpause_physics_client = self.create_client(Empty, '/world/table_world/unpause_physics')
        
        # 定义随机化参数范围
        self.mass_range = (0.9, 1.1)  # 质量变化范围 (±10%)
        self.inertia_range = (0.9, 1.1)  # 惯性变化范围 (±10%)
        self.friction_range = (0.8, 1.2)  # 摩擦系数变化范围 (±20%)
        self.damping_range = (0.9, 1.1)  # 阻尼系数变化范围 (±10%)
        
        # 定义文件路径
        self.urdf_file = '/home/ubuntu/ros2_ws/src/ros2_kortex/kortex_description/robots/gen3_2f85.urdf'
        
        # 创建定时器，每60秒随机化一次动力学参数
        self.timer = self.create_timer(60.0, self.randomize_dynamics)
        self.get_logger().info('动力学随机化节点已启动')
    
    def randomize_dynamics(self):
        """随机化动力学参数"""
        self.get_logger().info('开始随机化动力学参数...')
        
        # 暂停物理引擎以进行修改
        self.pause_physics()
        
        # 随机化URDF文件中的动力学参数
        self.randomize_urdf_dynamics()
        
        # 恢复物理引擎
        self.unpause_physics()
        self.get_logger().info('动力学参数随机化完成')
    
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
    
    def randomize_urdf_dynamics(self):
        """随机化URDF文件中的动力学参数"""
        self.get_logger().info(f'随机化URDF动力学参数: {self.urdf_file}')
        
        try:
            # 解析URDF文件
            tree = ET.parse(self.urdf_file)
            root = tree.getroot()
            
            # 随机化质量和惯性
            self.randomize_mass_and_inertia(root)
            
            # 随机化摩擦系数
            self.randomize_friction(root)
            
            # 保存修改后的URDF文件
            tree.write(self.urdf_file)
            self.get_logger().info('URDF动力学参数随机化完成')
            
            # 重新构建包以应用更改
            self.rebuild_package()
            
        except Exception as e:
            self.get_logger().error(f'随机化URDF动力学参数时出错: {str(e)}')
    
    def randomize_mass_and_inertia(self, root):
        """随机化质量和惯性参数"""
        # 找到所有惯性定义
        inertials = root.findall(".//inertial")
        
        for inertial in inertials:
            # 随机化质量
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                original_mass = float(mass_elem.get("value"))
                mass_factor = random.uniform(*self.mass_range)
                new_mass = original_mass * mass_factor
                mass_elem.set("value", f"{new_mass:.6f}")
                
                self.get_logger().info(f'质量从 {original_mass:.4f} 变为 {new_mass:.4f} (因子: {mass_factor:.2f})')
            
            # 随机化惯性
            inertia_elem = inertial.find("inertia")
            if inertia_elem is not None:
                for attr in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                    if inertia_elem.get(attr) is not None:
                        original_value = float(inertia_elem.get(attr))
                        # 对角惯性项使用相同的随机因子
                        if attr in ["ixx", "iyy", "izz"]:
                            inertia_factor = random.uniform(*self.inertia_range)
                        # 非对角项可以有不同的随机因子
                        else:
                            inertia_factor = random.uniform(*self.inertia_range)
                        
                        new_value = original_value * inertia_factor
                        inertia_elem.set(attr, f"{new_value:.8f}")
                        
                        self.get_logger().debug(f'惯性 {attr} 从 {original_value:.8f} 变为 {new_value:.8f}')
    
    def randomize_friction(self, root):
        """随机化摩擦系数"""
        # 找到所有摩擦定义
        frictions = root.findall(".//friction")
        
        for friction in frictions:
            # 随机化mu1和mu2
            for mu in ["mu1", "mu2"]:
                mu_elem = friction.find(f".//{mu}")
                if mu_elem is not None and mu_elem.text:
                    original_mu = float(mu_elem.text)
                    mu_factor = random.uniform(*self.friction_range)
                    new_mu = original_mu * mu_factor
                    mu_elem.text = f"{new_mu:.6f}"
                    
                    self.get_logger().info(f'摩擦系数 {mu} 从 {original_mu:.4f} 变为 {new_mu:.4f} (因子: {mu_factor:.2f})')
        
        # 找到所有接触参数
        contacts = root.findall(".//contact")
        
        for contact in contacts:
            # 随机化kp和kd
            for param in ["kp", "kd"]:
                param_elem = contact.find(f".//{param}")
                if param_elem is not None and param_elem.text:
                    original_value = float(param_elem.text)
                    factor = random.uniform(*self.damping_range)
                    new_value = original_value * factor
                    param_elem.text = f"{new_value:.6f}"
                    
                    self.get_logger().info(f'接触参数 {param} 从 {original_value:.4f} 变为 {new_value:.4f} (因子: {factor:.2f})')
    
    def rebuild_package(self):
        """重新构建包以应用URDF更改"""
        try:
            cmd = "cd /home/ubuntu/ros2_ws && colcon build --packages-select kortex_description && source install/setup.bash"
            subprocess.run(cmd, shell=True, check=True)
            self.get_logger().info('包重新构建完成')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'重新构建包时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    randomizer = DynamicsRandomizer()
    
    try:
        rclpy.spin(randomizer)
    except KeyboardInterrupt:
        pass
    finally:
        randomizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 