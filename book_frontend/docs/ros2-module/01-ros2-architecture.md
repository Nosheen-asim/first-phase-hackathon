---
sidebar_position: 1
---

# Chapter 1: ROS 2 Architecture

This chapter provides an introduction to ROS 2 architecture, focusing on core concepts for humanoid robot control and AI integration.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the purpose of ROS 2 as robotic middleware
- Explain the concepts of nodes, topics, and services in ROS 2
- Describe the communication model used in ROS 2
- Understand how these components work together in a robotic system

## Introduction to ROS 2 as Robotic Middleware

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 serves as middleware that provides services designed for a heterogeneous computer cluster such as:
- Hardware abstraction
- Low-level device control
- Implementation of commonly used functionality
- Message-passing between processes
- Package management

## Nodes

In ROS 2, a node is an executable that uses ROS 2 client libraries to communicate with other nodes. Nodes can publish or subscribe to messages, provide or use services, and manage parameters.

### Key Characteristics of Nodes:
- Each node is designed to perform a specific task
- Nodes communicate with each other through messages
- Multiple nodes can run simultaneously on the same machine or across multiple machines
- Nodes are language-agnostic (can be written in different programming languages)

### Example Node Structure:
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Message Passing

Topics are named buses over which nodes exchange messages. Topic-based communication in ROS 2 follows a publish-subscribe pattern where publishers send messages to a topic and subscribers receive messages from a topic.

### Key Concepts:
- Topics enable asynchronous communication
- Multiple publishers can publish to the same topic
- Multiple subscribers can subscribe to the same topic
- Message types must be agreed upon beforehand

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

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

## Services

Services provide a request-response communication pattern in ROS 2. A service has a service server that processes the request and sends back a response, and a service client that makes the request and receives the response.

### Key Concepts:
- Services enable synchronous communication
- Each service has a specific request and response message type
- Services are identified by name
- Services can be called on demand

## Communication Model

ROS 2 uses a distributed communication model based on the Data Distribution Service (DDS) standard. This provides:

- **Discovery**: Nodes automatically discover each other
- **Transport**: Multiple transport protocols (TCP, UDP, shared memory)
- **Quality of Service (QoS)**: Configurable policies for reliability, durability, etc.
- **Security**: Authentication, encryption, and access control

### QoS Profiles:
- Reliability: Best effort or reliable delivery
- Durability: Volatile or transient local
- History: Keep last N samples or keep all samples

## Summary

ROS 2 architecture provides a flexible framework for robot software development through its node-based design, topic-based message passing, and service-based request-response communication. The use of DDS as the underlying communication layer provides robust, scalable, and configurable communication between robot components.

## Exercises

1. Identify three key differences between nodes and services in ROS 2.
2. Explain why the publish-subscribe pattern is beneficial for robot communication.
3. Describe a scenario where you would use a service instead of a topic.