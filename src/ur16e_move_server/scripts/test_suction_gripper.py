#!/usr/bin/env python3
"""
Suction Gripper MTP Action Client Example
这个脚本演示如何调用suction_gripper_mtp action server来执行pick and place任务
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ur16e_move_server.action import MoveToPose
from geometry_msgs.msg import PoseStamped
import sys


class SuctionGripperClient(Node):
    """Action客户端用于发送pick and place目标"""
    
    def __init__(self):
        super().__init__('suction_gripper_client')
        
        # 创建action客户端
        self._action_client = ActionClient(
            self, 
            MoveToPose, 
            'suction_pick_place'
        )
        
        self.get_logger().info('Suction Gripper Client initialized')
        
    def send_goal(self, x, y, z, ox=1.0, oy=0.0, oz=0.0, ow=0.0):
        """
        发送pick位置目标
        
        Args:
            x, y, z: Pick位置坐标 (米)
            ox, oy, oz, ow: 方向四元数 (默认为竖直向下)
        """
        goal_msg = MoveToPose.Goal()
        
        # 设置目标位置
        goal_msg.target_pose.header.frame_id = 'world'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = z
        
        goal_msg.target_pose.pose.orientation.x = ox
        goal_msg.target_pose.pose.orientation.y = oy
        goal_msg.target_pose.pose.orientation.z = oz
        goal_msg.target_pose.pose.orientation.w = ow
        
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: position=[{x:.3f}, {y:.3f}, {z:.3f}]')
        
        # 发送目标并设置回调
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future
    
    def goal_response_callback(self, future):
        """处理目标接受响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return
        
        self.get_logger().info('Goal accepted by server, waiting for result...')
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """处理最终结果"""
        result = future.result().result
        
        if result.result:
            self.get_logger().info('✓ Pick and place completed successfully!')
        else:
            self.get_logger().error('✗ Pick and place failed')
        
        # 结束ROS
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        """处理反馈（如果action提供的话）"""
        feedback = feedback_msg.feedback
        # 如果实现了feedback，可以在这里处理
        # self.get_logger().info(f'Feedback: {feedback.stage} - {feedback.progress}%')
        pass


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    client = SuctionGripperClient()
    
    # 从命令行参数获取目标位置，或使用默认值
    if len(sys.argv) >= 4:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
        except ValueError:
            client.get_logger().error('Invalid coordinates. Usage: script.py x y z')
            return
    else:
        # 默认pick位置
        x, y, z = 0.5, 0.0, 0.1
        client.get_logger().info(f'Using default position: [{x}, {y}, {z}]')
    
    # 发送目标
    future = client.send_goal(x, y, z)
    
    # Spin等待完成
    rclpy.spin(client)


if __name__ == '__main__':
    main()
