---
sidebar_position: 7
---

# ROS 2 Communication & Python AI - Quiz

Test your understanding of ROS 2 communication and Python AI integration.

## Multiple Choice Questions

### 1. What is rclpy?
A) A Python library for ROS 1
B) A Python client library for ROS 2
C) A C++ library for ROS 2
D) A visualization tool for ROS 2

**Answer: B) A Python client library for ROS 2**

### 2. Which communication pattern is best for continuous sensor data streaming?
A) Services
B) Actions
C) Topics
D) Parameters

**Answer: C) Topics**

### 3. What type of communication pattern is used for request/response interactions?
A) Topics
B) Services
C) Actions
D) Parameters

**Answer: B) Services**

### 4. In the context of AI integration, what is the typical role of a Python AI agent in ROS 2?
A) Only publishing sensor data
B) Only subscribing to robot commands
C) Subscribing to sensor data and publishing decisions/commands
D) Managing the ROS 2 master

**Answer: C) Subscribing to sensor data and publishing decisions/commands**

### 5. Which message type would be most appropriate for sending a simple robot command?
A) std_msgs/String
B) std_msgs/Float32
C) sensor_msgs/LaserScan
D) geometry_msgs/Twist

**Answer: D) geometry_msgs/Twist**

## True/False Questions

### 6. rclpy allows Python programs to communicate with ROS 2 nodes.
A) True
B) False

**Answer: A) True**

### 7. Services in ROS 2 provide a one-way communication pattern.
A) True
B) False

**Answer: B) False**

### 8. Actions in ROS 2 are suitable for long-running tasks with feedback.
A) True
B) False

**Answer: A) True**

### 9. Multiple subscribers can listen to the same topic in ROS 2.
A) True
B) False

**Answer: A) True**

## Short Answer Questions

### 10. Explain the difference between topics, services, and actions in ROS 2.

**Answer: Topics provide publish/subscribe communication for continuous data streams. Services provide request/response communication for single requests. Actions provide goal-oriented communication with feedback and status for long-running tasks.**

### 11. How can a Python AI agent subscribe to sensor data in ROS 2?

**Answer: A Python AI agent can create a subscription using rclpy, specifying the topic name and message type. The agent defines a callback function that processes incoming sensor messages when they arrive.**

### 12. What are the advantages of using ROS 2 for AI-robot integration?

**Answer: ROS 2 provides: 1) Standardized message formats for sensor data and commands, 2) Built-in communication patterns, 3) Distributed architecture allowing multiple AI agents, 4) Tools for debugging and visualization, 5) Support for multiple programming languages.**

## Scenario-Based Questions

### 13. You're designing an AI system that needs to receive camera images, process them, and send navigation commands. Which ROS 2 communication patterns would you use and why?

**Answer: Use topics for camera images (continuous streaming), topics for navigation commands (continuous control), and potentially services for specific requests like "stop immediately". Topics are ideal for continuous data streams like camera images and navigation commands.**

### 14. How would you implement error handling in a Python AI agent that communicates with ROS 2?

**Answer: Implement error handling by: 1) Checking message validity before processing, 2) Using try/catch blocks around critical operations, 3) Implementing timeouts for service calls, 4) Adding fallback behaviors when communication fails, 5) Logging errors for debugging.**

## Summary

This quiz covers ROS 2 communication patterns and how Python AI agents can integrate with ROS 2 systems. Understanding these concepts is crucial for bridging AI algorithms with physical robot systems.