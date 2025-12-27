---
sidebar_position: 6
---

# ROS 2 Communication & Python AI - Hands-on Exercise

## Exercise: Connecting Python AI Agents to ROS 2

In this exercise, you'll create a simple Python AI agent that communicates with ROS 2 nodes using rclpy.

### Prerequisites

- ROS 2 installed (Humble Hawksbill or newer)
- Python 3.8 or higher
- Understanding of basic Python programming
- Knowledge of ROS 2 communication patterns

### Task 1: Creating a Python AI Agent with rclpy

Create a Python AI agent that subscribes to sensor data and makes decisions based on that data:

```python
# ai_agent.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import random

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Create a subscription to receive sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Create a publisher to send AI decisions
        self.decision_publisher = self.create_publisher(String, 'ai_decisions', 10)

        # Create a publisher to send commands to the robot
        self.cmd_publisher = self.create_publisher(Float32, 'robot_cmd', 10)

        self.get_logger().info('Simple AI Agent initialized')

    def laser_callback(self, msg):
        """Process laser scan data and make decisions"""
        # Get the minimum distance from the laser scan
        if len(msg.ranges) > 0:
            min_distance = min([r for r in msg.ranges if r > 0 and r < float('inf')])

            # Simple AI decision making
            decision = self.make_decision(min_distance)

            # Publish the decision
            decision_msg = String()
            decision_msg.data = decision
            self.decision_publisher.publish(decision_msg)

            # Publish robot command based on decision
            cmd_msg = Float32()
            cmd_msg.data = self.get_robot_command(decision)
            self.cmd_publisher.publish(cmd_msg)

            self.get_logger().info(f'Distance: {min_distance:.2f}, Decision: {decision}, Command: {cmd_msg.data}')

    def make_decision(self, distance):
        """Simple AI decision making based on distance"""
        if distance < 0.5:  # Obstacle too close
            return "OBSTACLE_AVOIDANCE"
        elif distance < 1.0:  # Obstacle at medium distance
            return "SLOW_DOWN"
        else:  # Clear path
            return "MOVE_FORWARD"

    def get_robot_command(self, decision):
        """Convert AI decision to robot command"""
        if decision == "OBSTACLE_AVOIDANCE":
            return -0.5  # Move backward
        elif decision == "SLOW_DOWN":
            return 0.3   # Move forward slowly
        else:  # MOVE_FORWARD
            return 0.8   # Move forward at normal speed

def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('AI Agent stopped by user')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 2: Creating a Mock Sensor Node

Create a mock sensor node that publishes simulated laser scan data:

```python
# mock_sensor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class MockLaserSensor(Node):
    def __init__(self):
        super().__init__('mock_laser_sensor')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Mock Laser Sensor initialized')

    def timer_callback(self):
        """Publish simulated laser scan data"""
        msg = LaserScan()

        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Set laser scan parameters
        msg.angle_min = -1.57  # -90 degrees in radians
        msg.angle_max = 1.57   # 90 degrees in radians
        msg.angle_increment = 0.1
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Generate random ranges (with occasional obstacles)
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = []

        for i in range(num_ranges):
            # Sometimes add an obstacle
            if random.random() < 0.2:  # 20% chance of obstacle
                msg.ranges.append(random.uniform(0.3, 0.8))
            else:
                msg.ranges.append(random.uniform(1.0, 5.0))

        self.publisher.publish(msg)
        self.get_logger().info(f'Published laser scan with {len(msg.ranges)} ranges')

def main(args=None):
    rclpy.init(args=args)
    mock_sensor = MockLaserSensor()

    try:
        rclpy.spin(mock_sensor)
    except KeyboardInterrupt:
        mock_sensor.get_logger().info('Mock sensor stopped by user')
    finally:
        mock_sensor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 3: Running the AI Agent and Mock Sensor

1. Make sure your Python files are executable:
```bash
chmod +x ai_agent.py mock_sensor.py
```

2. Run the mock sensor in one terminal:
```bash
python3 mock_sensor.py
```

3. In another terminal, run the AI agent:
```bash
python3 ai_agent.py
```

4. Observe how the AI agent processes sensor data and makes decisions.

### Exercise Questions

1. How does the AI agent receive sensor data from ROS 2?
2. What kind of decision-making logic does the AI agent use?
3. How does the AI agent send commands back to the robot?
4. What would happen if multiple AI agents were running simultaneously?
5. How could you modify the decision-making logic to be more sophisticated?

### Advanced Task

Modify the AI agent to:
1. Remember previous sensor readings to detect patterns
2. Implement a more complex decision-making algorithm (e.g., using a simple neural network)
3. Add a service client that can request specific actions from other nodes

### Summary

This exercise demonstrates how Python AI agents can be integrated with ROS 2 systems using rclpy. You've seen how:
- AI agents can subscribe to sensor data topics
- AI logic can process sensor data and make decisions
- AI agents can publish commands back to the robot
- rclpy provides the interface between Python and ROS 2