---
sidebar_position: 3
title: ROS 2 Communication & Python AI
---

# ROS 2 Communication & Python AI

## Nodes, Topics, Services, and Actions

### Nodes

Nodes are the fundamental building blocks of ROS 2 applications. In Python, you create nodes using the `rclpy` library. A node typically represents a single process that performs a specific function, such as sensor processing, control algorithms, or AI decision making.

```python
import rclpy
from rclpy.node import Node

class AINode(Node):
    def __init__(self):
        super().__init__('ai_node')
        # Node initialization code here
```

### Topics

Topics enable publish-subscribe communication in ROS 2. This is ideal for sensor data and other continuous data streams that multiple nodes might need to access.

```python
from std_msgs.msg import String

class AINode(Node):
    def __init__(self):
        super().__init__('ai_node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'ai_commands', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

    def sensor_callback(self, msg):
        # Process sensor data and make AI decisions
        self.get_logger().info(f'Received sensor data: {msg.data}')

        # Make AI decision and publish command
        cmd_msg = String()
        cmd_msg.data = 'move_forward'
        self.publisher.publish(cmd_msg)
```

### Services

Services provide request-response communication. This is useful for AI systems that need to perform complex computations on demand.

```python
from example_interfaces.srv import AddTwoInts

class AINode(Node):
    def __init__(self):
        super().__init__('ai_node')

        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Actions

Actions are used for long-running tasks that require feedback and goal management. This is perfect for AI planning and navigation tasks.

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class AINode(Node):
    def __init__(self):
        super().__init__('ai_node')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Pub/Sub for Sensors and Actuators

### Sensor Integration

Sensors typically publish data to topics that AI nodes can subscribe to. Here's an example of integrating sensor data with AI processing:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SensorAIProcessor(Node):
    def __init__(self):
        super().__init__('sensor_ai_processor')

        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Publish velocity commands to robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription  # prevent unused variable warning

    def laser_callback(self, msg):
        # AI logic to process sensor data
        min_distance = min(msg.ranges)

        # Simple AI decision: avoid obstacles
        cmd = Twist()
        if min_distance > 1.0:  # Safe distance
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn to avoid obstacle

        self.publisher.publish(cmd)
```

### Actuator Control

Actuators receive commands from AI nodes through publishers. The AI system analyzes sensor data and publishes appropriate commands to move the robot.

## Bridging Python AI Agents Using rclpy

### Setting up Python AI Integration

Python is an excellent choice for AI development due to its rich ecosystem of machine learning libraries. Integrating Python AI agents with ROS 2 is straightforward using rclpy.

```python
import rclpy
from rclpy.node import Node
import numpy as np
import tensorflow as tf  # Example AI library

class PythonAIAgent(Node):
    def __init__(self):
        super().__init__('python_ai_agent')

        # Load AI model
        self.model = self.load_model()

        # Setup ROS 2 communication
        self.subscription = self.create_subscription(
            # Define sensor message type
        )
        self.publisher = self.create_publisher(
            # Define command message type
        )

    def load_model(self):
        # Load your trained AI model
        # This could be a TensorFlow model, PyTorch model, etc.
        pass

    def ai_decision_callback(self, sensor_data):
        # Process sensor data through AI model
        processed_data = self.preprocess(sensor_data)
        action = self.model.predict(processed_data)

        # Publish AI decision
        self.publish_action(action)

    def preprocess(self, data):
        # Preprocess sensor data for AI model
        return np.array(data)

    def publish_action(self, action):
        # Convert AI decision to ROS 2 message and publish
        pass
```

### Example: Deep Learning Integration

Here's a more concrete example of integrating a deep learning model with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DeepLearningAIAgent(Node):
    def __init__(self):
        super().__init__('deep_learning_ai_agent')

        # Setup image processing
        self.bridge = CvBridge()

        # Setup AI model (example with placeholder)
        self.model = self.initialize_model()

        # Setup ROS 2 communication
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            # Define appropriate message type for AI output
        )

    def initialize_model(self):
        # Initialize your deep learning model here
        # Example: return tf.keras.models.load_model('path/to/model')
        pass

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image with AI model
        prediction = self.model.predict(np.expand_dims(cv_image, axis=0))

        # Process prediction and publish appropriate commands
        self.process_prediction(prediction)

    def process_prediction(self, prediction):
        # Implement logic to convert AI prediction to robot actions
        pass
```

## Best Practices for AI Integration

### Asynchronous Processing

For computationally intensive AI tasks, consider using separate threads or asynchronous processing:

```python
import threading
import queue

class AsyncAIAgent(Node):
    def __init__(self):
        super().__init__('async_ai_agent')

        self.ai_queue = queue.Queue()
        self.ai_thread = threading.Thread(target=self.ai_worker)
        self.ai_thread.start()

        self.subscription = self.create_subscription(
            # Define message type
        )

    def sensor_callback(self, msg):
        # Add sensor data to AI processing queue
        self.ai_queue.put(msg)

    def ai_worker(self):
        # Process AI tasks in separate thread
        while rclpy.ok():
            try:
                sensor_data = self.ai_queue.get(timeout=0.1)
                # Process with AI model
                result = self.process_with_ai(sensor_data)
                # Publish results
            except queue.Empty:
                continue
```

### Error Handling and Fallbacks

Implement robust error handling for AI systems:

```python
def ai_decision_callback(self, sensor_data):
    try:
        # AI processing
        action = self.model.predict(sensor_data)
        self.publish_action(action)
    except Exception as e:
        self.get_logger().error(f'AI model error: {e}')
        # Fallback behavior
        self.fallback_behavior()
```

## Summary

Integrating Python AI agents with ROS 2 enables powerful robotic applications. Using rclpy, you can create sophisticated AI systems that interact with sensors and actuators through ROS 2's communication primitives. Topics, services, and actions provide the necessary communication patterns for different types of AI-robot interactions. Following best practices for asynchronous processing and error handling ensures robust AI integration.