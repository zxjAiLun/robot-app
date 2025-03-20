#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped

import numpy as np
import time
import threading

class SimpleMoveitInterface(Node):
    """
    简单的MoveIt接口，用于控制Kinova Gen3机械臂
    """
    def __init__(self):
        super().__init__('simple_moveit_interface')
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建关节状态订阅者
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 创建关节轨迹发布者（用于控制机械臂）
        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # 创建夹爪控制发布者
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )
        
        # 存储当前关节状态
        self.current_joint_states = {}
        self.joint_states_lock = threading.Lock()
        
        # 机械臂关节名称
        self.arm_joint_names = [
            'gen3_joint_1', 'gen3_joint_2', 'gen3_joint_3', 
            'gen3_joint_4', 'gen3_joint_5', 'gen3_joint_6', 'gen3_joint_7'
        ]
        
        # 夹爪关节名称
        self.gripper_joint_names = ['gen3_robotiq_85_left_knuckle_joint']
        
        # 等待关节状态
        self.get_logger().info('等待关节状态...')
        while not hasattr(self, 'current_joint_states') or not self.current_joint_states:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('已接收关节状态')
        
        # 设置机械臂到安全的初始位置
        self.set_to_initial_pose()
    
    def joint_states_callback(self, msg):
        """关节状态回调函数"""
        with self.joint_states_lock:
            for i, name in enumerate(msg.name):
                self.current_joint_states[name] = msg.position[i]
    
    def get_current_arm_joint_positions(self):
        """获取当前机械臂关节位置"""
        positions = []
        with self.joint_states_lock:
            for joint_name in self.arm_joint_names:
                if joint_name in self.current_joint_states:
                    positions.append(self.current_joint_states[joint_name])
                else:
                    positions.append(0.0)
        return positions
    
    def move_to_joint_positions(self, joint_positions, duration=2.0):
        """移动到指定的关节位置"""
        self.get_logger().info(f'移动到关节位置: {joint_positions}')
        
        # 创建关节轨迹消息
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points.append(point)
        
        # 发布轨迹
        self.arm_trajectory_pub.publish(trajectory)
        
        # 等待执行完成
        time.sleep(duration + 0.5)
    
    def set_to_initial_pose(self):
        """设置机械臂到安全的初始位置"""
        self.get_logger().info('设置机械臂到初始位置...')
        
        # 安全的初始位置
        initial_positions = [0.0, -0.25, 0.0, -2.0, 0.0, 0.5, 1.57]
        self.move_to_joint_positions(initial_positions, 3.0)
    
    def move_to_pose(self, pose, duration=2.0):
        """移动到指定的位姿（简化版，仅支持预定义的姿态）"""
        self.get_logger().info(f'移动到位姿: {pose.position}')
        
        # 这里简化处理，使用预定义的关节位置
        # 在实际应用中，应该使用逆运动学计算
        
        # 根据目标位置调整关节角度
        if pose.position.z > 0.9:  # 提起位置
            joint_positions = [0.0, -0.25, 0.0, -2.0, 0.0, 0.5, 1.57]
        elif pose.position.z > 0.8:  # 预抓取位置
            joint_positions = [0.0, 0.0, 0.0, -2.0, 0.0, 0.8, 1.57]
        else:  # 抓取位置
            joint_positions = [0.0, 0.3, 0.0, -2.0, 0.0, 1.0, 1.57]
        
        self.move_to_joint_positions(joint_positions, duration)
    
    def open_gripper(self):
        """打开夹爪"""
        self.get_logger().info('打开夹爪')
        
        # 创建夹爪命令
        cmd = Float64MultiArray()
        cmd.data = [0.0]  # 打开位置
        
        # 发布命令
        self.gripper_pub.publish(cmd)
        
        # 等待执行完成
        time.sleep(1.0)
    
    def close_gripper(self):
        """闭合夹爪"""
        self.get_logger().info('闭合夹爪')
        
        # 创建夹爪命令
        cmd = Float64MultiArray()
        cmd.data = [0.8]  # 闭合位置
        
        # 发布命令
        self.gripper_pub.publish(cmd)
        
        # 等待执行完成
        time.sleep(1.5)  # 增加等待时间，确保夹爪完全闭合

def main(args=None):
    rclpy.init(args=args)
    
    interface = SimpleMoveitInterface()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(interface)
    
    try:
        interface.get_logger().info('开始执行...')
        executor.spin()
    except KeyboardInterrupt:
        interface.get_logger().info('用户中断')
    finally:
        executor.shutdown()
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 