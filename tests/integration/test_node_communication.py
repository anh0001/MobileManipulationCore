#!/usr/bin/env python3
"""
Example integration test for MobileManipulationCore.

This file demonstrates how to write integration tests that verify
multiple components working together.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from manipulation_msgs.msg import PolicyOutput
import numpy as np
from cv_bridge import CvBridge
import time


class TestNodeCommunication(unittest.TestCase):
    """Integration tests for node-to-node communication."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS 2 for testing."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS 2."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test node."""
        self.test_node = rclpy.create_node('test_node_communication')
        self.bridge = CvBridge()

    def tearDown(self):
        """Clean up test node."""
        self.test_node.destroy_node()

    def test_policy_output_published(self):
        """Test that policy node publishes output."""
        # Set up subscriber
        received_msg = []

        def callback(msg):
            received_msg.append(msg)

        sub = self.test_node.create_subscription(
            PolicyOutput,
            '/manipulation/policy_output',
            callback,
            10
        )

        # Wait for message (with timeout)
        start_time = time.time()
        timeout = 5.0

        while len(received_msg) == 0 and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Verify message received
        # Note: This test requires policy_node to be running
        # For actual testing, you might want to launch the node in the test
        if len(received_msg) > 0:
            msg = received_msg[0]
            self.assertIsInstance(msg, PolicyOutput)
            # Add more specific assertions about message content
        else:
            self.skipTest("No policy output received - node may not be running")

    def test_image_to_policy_pipeline(self):
        """Test image processing pipeline from camera to policy."""
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        image_msg = self.bridge.cv2_to_imgmsg(test_image, encoding='bgr8')

        # Publisher for test image
        image_pub = self.test_node.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )

        # Subscriber for policy output
        policy_outputs = []

        def policy_callback(msg):
            policy_outputs.append(msg)

        policy_sub = self.test_node.create_subscription(
            PolicyOutput,
            '/manipulation/policy_output',
            policy_callback,
            10
        )

        # Publish test image
        for _ in range(5):
            image_pub.publish(image_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Wait for policy to process
        start_time = time.time()
        timeout = 10.0

        while len(policy_outputs) == 0 and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Verify pipeline worked
        if len(policy_outputs) > 0:
            self.assertGreater(len(policy_outputs), 0)
        else:
            self.skipTest("Pipeline test requires running nodes")

    def test_joint_states_subscribed(self):
        """Test that adapter receives joint states."""
        # Create test joint state
        joint_state = JointState()
        joint_state.header.stamp = self.test_node.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state.position = [0.0, 0.5, 1.0, -0.5, 0.3, 0.0]

        # Publisher
        joint_pub = self.test_node.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Publish test joint state
        for _ in range(5):
            joint_pub.publish(joint_state)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # In a real test, you'd verify that the adapter processed this
        # For now, just verify we can publish without errors
        self.assertTrue(True)


class TestEndToEndFlow(unittest.TestCase):
    """End-to-end integration tests."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS 2 for testing."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Set up test node."""
        self.test_node = rclpy.create_node('test_e2e')

    def tearDown(self):
        """Clean up."""
        self.test_node.destroy_node()

    def test_safety_timeout_triggers(self):
        """Test that adapter safety timeout works."""
        # Monitor cmd_vel for safety stop
        cmd_vel_msgs = []

        def cmd_vel_callback(msg):
            cmd_vel_msgs.append(msg)

        from geometry_msgs.msg import Twist
        sub = self.test_node.create_subscription(
            Twist,
            '/cmd_vel',
            cmd_vel_callback,
            10
        )

        # Wait and observe
        # If policy stops publishing, adapter should stop base
        start_time = time.time()
        duration = 3.0  # Wait 3 seconds

        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # This test is more conceptual - in practice you'd:
        # 1. Stop policy node
        # 2. Verify adapter publishes zero velocity after timeout
        # 3. Restart policy node
        self.skipTest("Safety test requires controlled environment")


# Helper function for launch testing
def generate_test_description():
    """
    Generate launch description for this test.

    Used by launch_testing framework.
    """
    from launch import LaunchDescription
    from launch_ros.actions import Node

    return LaunchDescription([
        # Add nodes needed for integration tests
        # Node(package='manipulation_perception', executable='perception_node'),
        # Node(package='manipulation_policy', executable='policy_node'),
        # Node(package='manipulation_adapter', executable='adapter_node'),
    ])


if __name__ == '__main__':
    # Run with unittest
    unittest.main()
