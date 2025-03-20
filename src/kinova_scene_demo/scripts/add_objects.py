#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class SceneObjectsPublisher(Node):
    def __init__(self):
        super().__init__('scene_objects_publisher')
        
        # 创建MarkerArray发布器
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # 创建定时器，每秒发布一次场景对象
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        # 初始化marker ID
        self.marker_id = 0
        
    def create_box_marker(self, position, scale, color, frame_id="base_link"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene_objects"
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        return marker
        
    def publish_markers(self):
        marker_array = MarkerArray()
        
        # 添加桌子
        table = self.create_box_marker(
            position=[0.5, 0.0, -0.1],  # 位置
            scale=[0.6, 1.0, 0.2],      # 尺寸
            color=[0.8, 0.8, 0.8, 1.0]  # 灰色
        )
        marker_array.markers.append(table)
        
        # 添加立方体
        cube = self.create_box_marker(
            position=[0.5, 0.0, 0.1],    # 位置
            scale=[0.05, 0.05, 0.05],    # 尺寸
            color=[1.0, 0.0, 0.0, 1.0]   # 红色
        )
        marker_array.markers.append(cube)
        
        # 发布marker数组
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = SceneObjectsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 