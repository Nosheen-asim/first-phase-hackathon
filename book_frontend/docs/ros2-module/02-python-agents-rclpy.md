---
sidebar_position: 2
---

# Chapter 2: Python Agents with rclpy

This chapter focuses on using rclpy, the Python client library for ROS 2, to create agents that can interact with the ROS 2 ecosystem. We'll explore how to bridge AI logic to robot controllers using Python.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the rclpy Python client library and its purpose
- Create ROS 2 nodes using Python
- Implement publishers and subscribers with rclpy
- Bridge AI logic to robot controllers via rclpy
- Use rclpy effectively for robot communication

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API that allows you to write ROS 2 nodes, publish and subscribe to topics, provide and use services, and manage parameters.

### Key Features of rclpy:
- Pythonic interface to ROS 2 concepts
- Support for all ROS 2 client library features
- Integration with Python's async/await for asynchronous operations
- Type support for ROS message definitions

### Basic Setup:
```python
import rclpy
from rclpy.node import Node

# Initialize the ROS 2 client library
rclpy.init()

# Create a node
node = Node('my_node')

# Your node logic here

# Shutdown when done
rclpy.shutdown()
```

## Creating ROS 2 Nodes Using Python

Creating a node in rclpy involves subclassing the Node class and implementing the desired functionality.

### Basic Node Structure:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('MyNode has been initialized')

def main(args=None):
    rclpy.init(args=args)

    my_node = MyNode()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Best Practices:
- Use meaningful node names
- Initialize resources in the constructor
- Clean up resources in destroy_node()
- Use the logger for output instead of print()

## Publishers and Subscribers

rclpy provides a straightforward way to implement the publish-subscribe pattern for message passing.

### Publisher Example:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

### Subscriber Example:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
```

## Bridging AI Logic to Robot Controllers

One of the key applications of ROS 2 with Python is bridging AI logic to robot controllers. This enables sophisticated decision-making and planning algorithms to control physical robots.

### Example: AI Controller Bridge
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class AIControllerBridge(Node):
    def __init__(self):
        super().__init__('ai_controller_bridge')

        # Publisher to send commands to robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to receive sensor data
        self.sensor_subscriber = self.create_subscription(
            String,
            '/sensor_data',
            self.sensor_callback,
            10
        )

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_loop)

        self.latest_sensor_data = None

    def sensor_callback(self, msg):
        self.latest_sensor_data = msg.data
        # Process sensor data for AI logic

    def ai_decision_loop(self):
        if self.latest_sensor_data:
            # Implement AI logic here
            command = self.make_decision(self.latest_sensor_data)

            # Send command to robot
            cmd_msg = Twist()
            cmd_msg.linear.x = command['linear']
            cmd_msg.angular.z = command['angular']
            self.cmd_vel_publisher.publish(cmd_msg)

    def make_decision(self, sensor_data):
        # Placeholder for AI decision-making logic
        # In a real implementation, this could use machine learning models
        # or other AI algorithms
        return {'linear': 0.5, 'angular': 0.0}

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIControllerBridge()
    rclpy.spin(ai_bridge)
    ai_bridge.destroy_node()
    rclpy.shutdown()
```

## Advanced rclpy Features

### Using Services
```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Using Parameters
```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')

        # Get parameter value
        param_value = self.get_parameter('my_parameter').value
        self.get_logger().info(f'Parameter value: {param_value}')
```

## Summary

rclpy provides a powerful and Pythonic way to interact with ROS 2. By creating nodes, publishers, subscribers, and services with rclpy, you can effectively bridge AI logic to robot controllers. The library supports all the essential ROS 2 concepts while maintaining the familiar Python programming patterns.

## Exercises

1. Create a simple publisher that publishes the current time every second.
2. Implement a subscriber that logs all received messages to a file.
3. Design a node that bridges sensor data from a robot to an AI model for processing.