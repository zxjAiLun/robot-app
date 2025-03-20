#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
import sys
import signal

# 导入各个随机化器
from domain_randomization.domain_randomizer import DomainRandomizer
from domain_randomization.material_randomizer import MaterialRandomizer
from domain_randomization.dynamics_randomizer import DynamicsRandomizer

class DomainRandomizationManager(Node):
    """
    域随机化管理器，用于启动和管理所有随机化器
    """
    def __init__(self):
        super().__init__('domain_randomization_manager')
        
        self.get_logger().info('域随机化管理器启动中...')
        
        # 创建随机化器实例
        self.domain_randomizer = DomainRandomizer()
        self.material_randomizer = MaterialRandomizer()
        self.dynamics_randomizer = DynamicsRandomizer()
        
        # 创建线程来运行各个随机化器
        self.threads = []
        
        # 创建定时器，每5秒检查一次随机化器状态
        self.timer = self.create_timer(5.0, self.check_randomizers)
        
        self.get_logger().info('域随机化管理器已启动')
    
    def check_randomizers(self):
        """检查随机化器状态"""
        self.get_logger().debug('检查随机化器状态...')
        # 这里可以添加检查逻辑，例如检查各个随机化器是否仍在运行
    
    def spin_randomizers(self):
        """启动所有随机化器"""
        # 创建并启动线程
        domain_thread = threading.Thread(target=lambda: rclpy.spin(self.domain_randomizer))
        material_thread = threading.Thread(target=lambda: rclpy.spin(self.material_randomizer))
        dynamics_thread = threading.Thread(target=lambda: rclpy.spin(self.dynamics_randomizer))
        
        domain_thread.daemon = True
        material_thread.daemon = True
        dynamics_thread.daemon = True
        
        domain_thread.start()
        self.get_logger().info('域随机化器已启动')
        
        material_thread.start()
        self.get_logger().info('材质随机化器已启动')
        
        dynamics_thread.start()
        self.get_logger().info('动力学随机化器已启动')
        
        self.threads = [domain_thread, material_thread, dynamics_thread]
    
    def shutdown(self):
        """关闭所有随机化器"""
        self.get_logger().info('正在关闭域随机化管理器...')
        
        # 销毁随机化器节点
        self.domain_randomizer.destroy_node()
        self.material_randomizer.destroy_node()
        self.dynamics_randomizer.destroy_node()
        
        # 等待线程结束
        for thread in self.threads:
            thread.join(timeout=1.0)
        
        # 销毁自身
        self.destroy_node()
        self.get_logger().info('域随机化管理器已关闭')

def main(args=None):
    rclpy.init(args=args)
    manager = DomainRandomizationManager()
    
    # 设置信号处理
    def signal_handler(sig, frame):
        print('接收到中断信号，正在关闭...')
        manager.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # 启动随机化器
    manager.spin_randomizers()
    
    try:
        # 主线程运行管理器
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭所有节点
        manager.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 