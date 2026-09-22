#!/usr/bin/env python3
"""
Test script to send a grasp action goal compatible with GraspActionNode.
"""

import rclpy
from rclpy.action import ActionClient
from robot_interfaces.action import XArm

def main():
    rclpy.init()
    node = rclpy.create_node('test_grasp_client')
    
    action_client = ActionClient(node, XArm, 'xarm_grasp_action')
    
    # Wait for action server
    while not action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info("Waiting for grasp action server...")
    
    node.get_logger().info("Connected to grasp action server")
    
    # Create a goal with just the object ID
    goal = XArm.Goal()
    goal.id = 1  # Example: send object ID 1
    
    node.get_logger().info(f"Sending goal with object ID: {goal.id}")
    
    # Send goal asynchronously
    send_goal_future = action_client.send_goal_async(goal)
    
    # Wait for result
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    
    if not goal_handle.accepted:
        node.get_logger().info('Goal rejected :(')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    node.get_logger().info('Goal accepted :)')
    
    # Get result asynchronously
    result_future = goal_handle.get_result_async()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        if result_future.done():
            result_response = result_future.result()
            result = result_response.result if hasattr(result_response, 'result') else result_response
            node.get_logger().info(f'Action completed with result: {result}')
            if hasattr(result, 'current_number'):
                node.get_logger().info(f'Grasp success flag: {result.current_number}')
            break
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()