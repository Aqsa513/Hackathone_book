---
sidebar_position: 2
title: Introduction to ROS 2
---

# Introduction to ROS 2

## ROS 2 Architecture and Purpose

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Components of ROS 2

- **Nodes**: Processes that perform computation
- **Topics**: Communication channels for passing messages between nodes
- **Services**: Request/response communication pattern
- **Actions**: Goal-oriented communication with feedback and status
- **Parameters**: Configuration values that can be changed at runtime
- **Lifecycle**: State management for nodes

### Why ROS 2?

ROS 2 addresses the limitations of ROS 1 by providing:

- **Real-time support**: Deterministic behavior for time-critical applications
- **Multi-robot systems**: Better support for multiple robots working together
- **Security**: Built-in security features for safe robot operation
- **Quality of Service (QoS)**: Configurable communication behavior
- **Cross-platform support**: Runs on various operating systems and architectures

## Nodes, Graphs, and DDS

### Nodes

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 application. Each node can perform specific tasks such as:

- Sensor data processing
- Control algorithm execution
- User interface management
- Data logging and visualization

Nodes communicate with each other through topics, services, and actions.

### Computation Graph

The collection of all nodes and their connections is called the computation graph. This graph represents:

- How data flows through the system
- The relationships between different components
- The dependencies between various robot functions

### DDS (Data Distribution Service)

DDS is the middleware that enables communication between nodes in ROS 2. It provides:

- **Discovery**: Nodes automatically find each other
- **Transport**: Reliable delivery of messages
- **Quality of Service**: Configurable reliability and performance settings
- **Data-centricity**: Communication based on data rather than connections

DDS implementations used in ROS 2 include:
- Fast DDS
- Cyclone DDS
- RTI Connext DDS

## ROS 2 as the Robotic Nervous System

ROS 2 functions as the nervous system of a robot by:

- **Sensing**: Collecting data from various sensors (cameras, LIDAR, IMU, etc.)
- **Processing**: Analyzing sensor data and making decisions
- **Acting**: Sending commands to actuators and motors
- **Communicating**: Coordinating between different components

This distributed architecture allows for modular robot design where different components can be developed and tested independently while maintaining seamless integration through the ROS 2 communication layer.

## Getting Started with ROS 2

To work with ROS 2, you'll need to:

1. Install ROS 2 (Humble Hawksbill or newer recommended)
2. Set up your ROS 2 environment
3. Create a workspace for your robot projects
4. Learn the basic ROS 2 commands

### Basic ROS 2 Commands

```bash
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Check available nodes
ros2 node list

# Check available topics
ros2 topic list

# Run a node
ros2 run <package_name> <node_name>

# Launch multiple nodes
ros2 launch <package_name> <launch_file>.py
```

## Summary

ROS 2 provides the foundational architecture for modern robotic systems. Understanding its components, particularly nodes, the computation graph, and DDS, is essential for developing complex robotic applications. As the robotic nervous system, ROS 2 enables seamless communication and coordination between different components of a robot system.