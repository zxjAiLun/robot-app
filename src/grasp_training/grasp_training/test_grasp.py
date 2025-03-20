#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time

from grasp_training.simple_moveit_interface import SimpleMoveitInterface

class TestGrasp(Node):
    """
    测试机械臂抓取姿态的节点
    """
    def __init__(self):
        super().__init__('test_grasp')
        
        # 初始化MoveIt接口
        self.get_logger().info('初始化MoveIt接口...')
        self.moveit_interface = SimpleMoveitInterface()
        
        self.get_logger().info('初始化完成，开始测试抓取姿态')
        
        # 等待一段时间，确保机械臂已经初始化
        time.sleep(2.0)
        
        # 执行测试
        self.test_grasp_poses()
    
    def test_grasp_poses(self):
        """测试不同的抓取姿态"""
        self.get_logger().info('测试抓取姿态...')
        
        # 测试初始姿态
        self.get_logger().info('1. 测试初始姿态')
        initial_positions = [0.0, -0.25, 0.0, -2.0, 0.0, 0.5, 1.57]
        self.moveit_interface.move_to_joint_positions(initial_positions, 3.0)
        time.sleep(2.0)
        
        # 测试预抓取姿态
        self.get_logger().info('2. 测试预抓取姿态')
        pre_grasp_positions = [0.0, 0.0, 0.0, -2.0, 0.0, 0.8, 1.57]
        self.moveit_interface.move_to_joint_positions(pre_grasp_positions, 3.0)
        time.sleep(2.0)
        
        # 测试抓取姿态
        self.get_logger().info('3. 测试抓取姿态')
        grasp_positions = [0.0, 0.3, 0.0, -2.0, 0.0, 1.0, 1.57]
        self.moveit_interface.move_to_joint_positions(grasp_positions, 3.0)
        time.sleep(2.0)
        
        # 测试夹爪操作
        self.get_logger().info('4. 测试夹爪操作')
        self.moveit_interface.open_gripper()
        time.sleep(1.0)
        self.moveit_interface.close_gripper()
        time.sleep(1.0)
        
        # 返回初始姿态
        self.get_logger().info('5. 返回初始姿态')
        self.moveit_interface.move_to_joint_positions(initial_positions, 3.0)
        
        self.get_logger().info('测试完成')

def main(args=None):
    rclpy.init(args=args)
    
    test_grasp = TestGrasp()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(test_grasp)
    
    try:
        test_grasp.get_logger().info('开始执行...')
        executor.spin()
    except KeyboardInterrupt:
        test_grasp.get_logger().info('用户中断')
    finally:
        executor.shutdown()
        test_grasp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 