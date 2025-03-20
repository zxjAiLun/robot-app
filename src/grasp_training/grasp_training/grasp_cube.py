#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener

import time
import threading

from grasp_training.simple_moveit_interface import SimpleMoveitInterface

class GraspTraining(Node):
    """
    用于训练Kinova Gen3机械臂抓取立方体的节点
    """
    def __init__(self):
        super().__init__('grasp_training')
        
        # 创建回调组和定时器
        self.callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(15.0, self.timer_callback, callback_group=self.callback_group)
        
        # 创建TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 初始化MoveIt接口
        self.get_logger().info('初始化MoveIt接口...')
        self.moveit_interface = SimpleMoveitInterface()
        
        self.get_logger().info('初始化完成，准备开始抓取训练')
        
        # 设置目标立方体的位置
        self.cube_position = Point(x=0.8, y=0.0, z=0.775)
        
        # 设置抓取状态
        self.is_executing = False
        self.grasp_completed = False
    
    def timer_callback(self):
        """定时器回调函数，用于触发抓取动作"""
        if not self.is_executing and not self.grasp_completed:
            self.is_executing = True
            # 创建线程执行抓取动作
            threading.Thread(target=self.execute_grasp).start()
    
    def execute_grasp(self):
        """执行抓取动作的主函数"""
        try:
            self.get_logger().info('开始执行抓取动作...')
            
            # 1. 移动到预抓取位置
            self.move_to_pre_grasp()
            
            # 2. 打开夹爪
            self.open_gripper()
            
            # 3. 移动到抓取位置
            self.move_to_grasp()
            
            # 4. 闭合夹爪
            self.close_gripper()
            
            # 5. 提起物体
            self.lift_object()
            
            self.get_logger().info('抓取动作执行完成！')
            self.grasp_completed = True
            
        except Exception as e:
            self.get_logger().error(f'执行抓取动作时出错: {str(e)}')
        finally:
            self.is_executing = False
    
    def move_to_pre_grasp(self):
        """移动到预抓取位置"""
        self.get_logger().info('移动到预抓取位置...')
        
        # 设置预抓取位置（在立方体上方约10cm处）
        target_pose = Pose()
        target_pose.position.x = self.cube_position.x
        target_pose.position.y = self.cube_position.y
        target_pose.position.z = self.cube_position.z + 0.15  # 在立方体上方15cm
        
        # 设置夹爪朝下的姿态
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707
        
        # 使用MoveIt接口移动
        self.moveit_interface.move_to_pose(target_pose)
    
    def move_to_grasp(self):
        """移动到抓取位置"""
        self.get_logger().info('移动到抓取位置...')
        
        # 设置抓取位置（立方体位置，考虑夹爪的偏移）
        target_pose = Pose()
        target_pose.position.x = self.cube_position.x
        target_pose.position.y = self.cube_position.y
        target_pose.position.z = self.cube_position.z + 0.02  # 考虑立方体高度和夹爪偏移
        
        # 设置夹爪朝下的姿态
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707
        
        # 使用MoveIt接口移动
        self.moveit_interface.move_to_pose(target_pose)
    
    def lift_object(self):
        """提起物体"""
        self.get_logger().info('提起物体...')
        
        # 设置提起位置（在当前位置上方约20cm处）
        target_pose = Pose()
        target_pose.position.x = self.cube_position.x
        target_pose.position.y = self.cube_position.y
        target_pose.position.z = self.cube_position.z + 0.3  # 提起30cm
        
        # 设置夹爪朝下的姿态
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707
        
        # 使用MoveIt接口移动
        self.moveit_interface.move_to_pose(target_pose)
    
    def open_gripper(self):
        """打开夹爪"""
        self.get_logger().info('打开夹爪...')
        self.moveit_interface.open_gripper()
    
    def close_gripper(self):
        """闭合夹爪"""
        self.get_logger().info('闭合夹爪...')
        self.moveit_interface.close_gripper()

def main(args=None):
    rclpy.init(args=args)
    
    grasp_training = GraspTraining()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(grasp_training)
    
    try:
        grasp_training.get_logger().info('开始执行...')
        executor.spin()
    except KeyboardInterrupt:
        grasp_training.get_logger().info('用户中断')
    except Exception as e:
        grasp_training.get_logger().error(f'发生错误: {str(e)}')
    finally:
        executor.shutdown()
        grasp_training.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 