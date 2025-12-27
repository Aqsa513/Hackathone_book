---
sidebar_position: 3
---

# Introduction to ROS 2 - Hands-on Exercise

## Exercise: Understanding ROS 2 Architecture

In this exercise, you'll create a simple ROS 2 system to understand the basic architecture and communication patterns.

### Prerequisites

- ROS 2 installed (Humble Hawksbill or newer)
- Basic knowledge of command line tools
- Understanding of the ROS 2 concepts covered in the previous section

### Task 1: Creating a Simple Publisher Node

1. Create a new ROS 2 package:
```bash
ros2 pkg create --build-type ament_python ros2_exercise_pkg
```

2. Create a publisher node that publishes a simple message:
```python
# ros2_exercise_pkg/ros2_exercise_pkg/publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2 World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 2: Creating a Simple Subscriber Node

Create a subscriber node that receives messages from the publisher:
```python
# ros2_exercise_pkg/ros2_exercise_pkg/subscriber_node.py
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

if __name__ == '__main__':
    main()
```

### Task 3: Running the Nodes

1. Make sure your publisher and subscriber nodes are executable:
```bash
chmod +x ros2_exercise_pkg/ros2_exercise_pkg/publisher_node.py
chmod +x ros2_exercise_pkg/ros2_exercise_pkg/subscriber_node.py
```

2. Build your package:
```bash
colcon build --packages-select ros2_exercise_pkg
```

3. Source your workspace:
```bash
source install/setup.bash
```

4. Run the publisher in one terminal:
```bash
ros2 run ros2_exercise_pkg publisher_node
```

5. In another terminal, run the subscriber:
```bash
ros2 run ros2_exercise_pkg subscriber_node
```

6. Observe the communication between the nodes.

### Exercise Questions

1. What happens when you run the publisher and subscriber nodes?
2. How does DDS enable the automatic discovery between these nodes?
3. What would happen if you created multiple subscribers for the same topic?
4. What would happen if you created multiple publishers for the same topic?

### Summary

This exercise demonstrates the basic ROS 2 architecture with nodes communicating through topics. You've seen how:
- Nodes can publish messages to topics
- Other nodes can subscribe to those topics
- DDS handles the discovery and communication automatically
- Multiple nodes can communicate without direct connections