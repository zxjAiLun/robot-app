#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import numpy as np
import xml.etree.ElementTree as ET
import os
import tempfile
import subprocess
from ament_index_python.packages import get_package_share_directory

class MaterialRandomizer(Node):
    """
    材质随机化节点，用于随机化URDF和SDF文件中的材质和颜色
    """
    def __init__(self):
        super().__init__('material_randomizer')
        
        # 定义随机化参数范围
        self.color_range = {
            'r': (0.1, 1.0),
            'g': (0.1, 1.0),
            'b': (0.1, 1.0),
            'a': (0.8, 1.0)
        }
        
        # 定义文件路径
        self.urdf_file = '/home/ubuntu/ros2_ws/src/ros2_kortex/kortex_description/robots/gen3_2f85.urdf'
        self.world_file = '/home/ubuntu/ros2_ws/src/table_drawer/world.sdf'
        
        # 创建定时器，每30秒随机化一次材质
        self.timer = self.create_timer(30.0, self.randomize_materials)
        self.get_logger().info('材质随机化节点已启动')
    
    def randomize_materials(self):
        """随机化所有材质"""
        self.get_logger().info('开始随机化材质...')
        
        # 随机化URDF文件中的材质
        self.randomize_urdf_materials()
        
        # 随机化SDF文件中的材质
        self.randomize_sdf_materials()
        
        self.get_logger().info('材质随机化完成')
    
    def generate_random_color(self):
        """生成随机颜色"""
        r = random.uniform(*self.color_range['r'])
        g = random.uniform(*self.color_range['g'])
        b = random.uniform(*self.color_range['b'])
        a = random.uniform(*self.color_range['a'])
        return r, g, b, a
    
    def randomize_urdf_materials(self):
        """随机化URDF文件中的材质"""
        self.get_logger().info(f'随机化URDF材质: {self.urdf_file}')
        
        try:
            # 解析URDF文件
            tree = ET.parse(self.urdf_file)
            root = tree.getroot()
            
            # 找到所有材质定义
            materials = root.findall(".//material")
            
            # 随机化每个材质
            for material in materials:
                # 检查是否有颜色定义
                color = material.find("color")
                if color is not None:
                    # 生成随机颜色
                    r, g, b, a = self.generate_random_color()
                    
                    # 更新颜色属性
                    color.set("rgba", f"{r:.6f} {g:.6f} {b:.6f} {a:.6f}")
                    
                    # 记录更改
                    material_name = material.get("name", "unnamed")
                    self.get_logger().info(f'更新材质 {material_name}: rgba=({r:.2f}, {g:.2f}, {b:.2f}, {a:.2f})')
            
            # 特别处理相机材质
            camera_material = root.find(".//link[@name='wrist_mounted_camera_link']/visual/material")
            if camera_material is not None:
                # 相机使用深色调
                r, g, b = random.uniform(0.05, 0.2), random.uniform(0.05, 0.2), random.uniform(0.05, 0.2)
                a = 1.0
                
                color = camera_material.find("color")
                if color is not None:
                    color.set("rgba", f"{r:.6f} {g:.6f} {b:.6f} {a:.6f}")
                    self.get_logger().info(f'更新相机材质: rgba=({r:.2f}, {g:.2f}, {b:.2f}, {a:.2f})')
            
            # 保存修改后的URDF文件
            tree.write(self.urdf_file)
            self.get_logger().info('URDF材质随机化完成')
            
            # 重新构建包以应用更改
            self.rebuild_package()
            
        except Exception as e:
            self.get_logger().error(f'随机化URDF材质时出错: {str(e)}')
    
    def randomize_sdf_materials(self):
        """随机化SDF文件中的材质"""
        self.get_logger().info(f'随机化SDF材质: {self.world_file}')
        
        try:
            # 解析SDF文件
            tree = ET.parse(self.world_file)
            root = tree.getroot()
            
            # 找到所有材质定义
            materials = root.findall(".//material")
            
            # 随机化每个材质
            for material in materials:
                # 检查是否有ambient, diffuse, specular定义
                for prop in ["ambient", "diffuse", "specular"]:
                    elem = material.find(prop)
                    if elem is not None:
                        # 生成随机颜色
                        r, g, b, a = self.generate_random_color()
                        
                        # 更新颜色文本
                        elem.text = f"{r:.6f} {g:.6f} {b:.6f} {a:.6f}"
                        
                        # 记录更改
                        self.get_logger().info(f'更新SDF {prop}: ({r:.2f}, {g:.2f}, {b:.2f}, {a:.2f})')
            
            # 保存修改后的SDF文件
            tree.write(self.world_file)
            self.get_logger().info('SDF材质随机化完成')
            
        except Exception as e:
            self.get_logger().error(f'随机化SDF材质时出错: {str(e)}')
    
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
    randomizer = MaterialRandomizer()
    
    try:
        rclpy.spin(randomizer)
    except KeyboardInterrupt:
        pass
    finally:
        randomizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 