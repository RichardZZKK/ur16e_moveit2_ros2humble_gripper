#!/usr/bin/env python3
"""
Mock Perception Service for Simulation Testing
模拟感知服务，用于在没有真实GroundingDINO时进行测试
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import random

# TODO: Replace with actual message type when available
# from open_set_object_detection_msgs.srv import GetObjectLocations


class MockPerceptionServer(Node):
    """
    模拟的感知服务器
    返回预定义的物体位置，用于测试pick-and-place流程
    """
    
    def __init__(self):
        super().__init__('mock_perception_server')
        
        # Create service (使用std_srvs的Trigger)
        # TODO: 当有实际的GetObjectLocations service时，替换这里
        self.srv = self.create_service(
            Trigger,
            'get_object_locations',  # 或 'left_get_object_locations'
            self.perception_callback
        )
        
        # Predefined object locations for testing
        self.test_objects = [
            {'x': 0.5, 'y': 0.0, 'z': 0.05, 'name': 'center_object'},
            {'x': 0.4, 'y': 0.2, 'z': 0.05, 'name': 'right_object'},
            {'x': 0.4, 'y': -0.2, 'z': 0.05, 'name': 'left_object'},
            {'x': 0.6, 'y': 0.1, 'z': 0.05, 'name': 'far_right_object'},
        ]
        
        self.current_object_index = 0
        
        self.get_logger().info('Mock Perception Service started')
        self.get_logger().info(f'Available test objects: {len(self.test_objects)}')
        for obj in self.test_objects:
            self.get_logger().info(f'  - {obj["name"]}: [{obj["x"]:.2f}, {obj["y"]:.2f}, {obj["z"]:.2f}]')
    
    def perception_callback(self, request, response):
        """
        处理感知请求
        
        这是一个临时实现，当实际的GetObjectLocations service可用时需要修改
        """
        # Get current test object
        obj = self.test_objects[self.current_object_index]
        
        # Add small random noise to simulate real perception
        noise_x = random.uniform(-0.01, 0.01)
        noise_y = random.uniform(-0.01, 0.01)
        noise_z = random.uniform(-0.005, 0.005)
        
        detected_x = obj['x'] + noise_x
        detected_y = obj['y'] + noise_y
        detected_z = obj['z'] + noise_z
        
        self.get_logger().info(f'Detected object: {obj["name"]}')
        self.get_logger().info(f'  Position: [{detected_x:.3f}, {detected_y:.3f}, {detected_z:.3f}]')
        self.get_logger().info(f'  (Original: [{obj["x"]:.3f}, {obj["y"]:.3f}, {obj["z"]:.3f}])')
        
        # TODO: 替换为实际的response构造
        # 这里只是一个占位符
        response.success = True
        response.message = f'{detected_x},{detected_y},{detected_z}'
        
        # Cycle to next object for next call
        self.current_object_index = (self.current_object_index + 1) % len(self.test_objects)
        
        return response


class ImprovedMockPerceptionServer(Node):
    """
    改进的模拟感知服务器
    提供更详细的物体信息和更多配置选项
    """
    
    def __init__(self):
        super().__init__('improved_mock_perception_server')
        
        # Parameters
        self.declare_parameter('add_noise', True)
        self.declare_parameter('noise_stddev', 0.01)
        self.declare_parameter('detection_delay', 0.5)
        self.declare_parameter('always_detect', True)
        
        self.add_noise = self.get_parameter('add_noise').value
        self.noise_stddev = self.get_parameter('noise_stddev').value
        self.detection_delay = self.get_parameter('detection_delay').value
        self.always_detect = self.get_parameter('always_detect').value
        
        # Service (使用std_srvs的Trigger)
        self.srv = self.create_service(
            Trigger,
            'get_object_locations',
            self.perception_callback
        )
        
        # Test scenarios
        self.scenarios = {
            'single_center': [
                {'x': 0.5, 'y': 0.0, 'z': 0.05}
            ],
            'line_of_objects': [
                {'x': 0.4, 'y': -0.2, 'z': 0.05},
                {'x': 0.4, 'y': 0.0, 'z': 0.05},
                {'x': 0.4, 'y': 0.2, 'z': 0.05},
            ],
            'scattered': [
                {'x': 0.45, 'y': 0.15, 'z': 0.05},
                {'x': 0.55, 'y': -0.1, 'z': 0.05},
                {'x': 0.6, 'y': 0.2, 'z': 0.05},
            ],
            'first_quadrant': [
                {'x': 0.3, 'y': 0.3, 'z': 0.05},  # Test origin waypoint logic
                {'x': 0.2, 'y': 0.2, 'z': 0.05},
            ],
        }
        
        # Current scenario
        self.declare_parameter('scenario', 'single_center')
        scenario_name = self.get_parameter('scenario').value
        self.current_scenario = self.scenarios.get(scenario_name, self.scenarios['single_center'])
        self.object_index = 0
        
        self.get_logger().info('=== Improved Mock Perception Service ===')
        self.get_logger().info(f'Scenario: {scenario_name}')
        self.get_logger().info(f'Add noise: {self.add_noise}')
        self.get_logger().info(f'Always detect: {self.always_detect}')
        self.get_logger().info(f'Objects in scenario: {len(self.current_scenario)}')
    
    def perception_callback(self, request, response):
        """处理感知请求"""
        import time
        
        # Simulate processing delay
        time.sleep(self.detection_delay)
        
        if self.always_detect or random.random() > 0.1:  # 90% detection rate
            obj = self.current_scenario[self.object_index]
            
            # Add noise if enabled
            if self.add_noise:
                noise_x = random.gauss(0, self.noise_stddev)
                noise_y = random.gauss(0, self.noise_stddev)
                noise_z = random.gauss(0, self.noise_stddev * 0.5)
            else:
                noise_x = noise_y = noise_z = 0.0
            
            x = obj['x'] + noise_x
            y = obj['y'] + noise_y
            z = obj['z'] + noise_z
            
            self.get_logger().info(f'✓ Object detected at: [{x:.3f}, {y:.3f}, {z:.3f}]')
            
            response.success = True
            response.message = f'{x},{y},{z}'
            
            # Move to next object
            self.object_index = (self.object_index + 1) % len(self.current_scenario)
        else:
            self.get_logger().warn('✗ No object detected (simulated failure)')
            response.success = False
            response.message = 'No objects detected'
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    # You can switch between basic and improved versions
    # node = MockPerceptionServer()
    node = ImprovedMockPerceptionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
