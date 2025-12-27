---
sidebar_position: 20
title: Simulated Sensors Exercise
---

# Simulated Sensors - Hands-on Exercise

## Exercise: Creating Simulated Sensors in Gazebo

In this exercise, you'll create a robot model with simulated LiDAR, depth camera, and IMU sensors in Gazebo.

### Prerequisites

- Gazebo installed (Gazebo 11 or newer)
- Basic understanding of URDF/SDF
- ROS 2 installed for sensor data processing (optional)

### Task 1: Creating a Robot Model with Sensors

Create a robot model with all three sensor types:

```xml
<?xml version="1.0" ?>
<robot name="sensor_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Robot properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="M_PI_2" value="1.5707963267948966" />

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2" />
    </inertial>

    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0" />
      <geometry>
        <box size="0.8 0.6 0.4" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0" />
      <geometry>
        <box size="0.8 0.6 0.4" />
      </geometry>
    </collision>
  </link>

  <!-- LiDAR Mount -->
  <link name="laser_link">
    <inertial>
      <mass value="0.1" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.05" />
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.05" />
      </geometry>
    </collision>
  </link>

  <!-- LiDAR Joint -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link" />
    <child link="laser_link" />
    <origin xyz="0.3 0 0.3" rpy="0 0 0" />
  </joint>

  <!-- Depth Camera Mount -->
  <link name="camera_link">
    <inertial>
      <mass value="0.1" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.05 0.1 0.03" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.05 0.1 0.03" />
      </geometry>
    </collision>
  </link>

  <!-- Depth Camera Joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link" />
    <child link="camera_link" />
    <origin xyz="0.35 0 0.25" rpy="0 0 0" />
  </joint>

  <!-- IMU Mount -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001" />
    </inertial>
  </link>

  <!-- IMU Joint -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link" />
    <child link="imu_link" />
    <origin xyz="0 0 0.2" rpy="0 0 0" />
  </joint>

  <!-- LiDAR Sensor Definition -->
  <gazebo reference="laser_link">
    <sensor type="ray" name="laser">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-2.356194</min_angle> <!-- -135 degrees -->
            <max_angle>2.356194</max_angle>  <!-- 135 degrees -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
        <topic_name>scan</topic_name>
        <frame_name>laser_link</frame_name>
        <min_intensity>0.1</min_intensity>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Depth Camera Sensor Definition -->
  <gazebo reference="camera_link">
    <sensor type="depth" name="depth_camera">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <baseline>0.2</baseline>
        <alwaysOn>true</alwaysOn>
        <updateRate>1.0</updateRate>
        <cameraName>camera_ir</cameraName>
        <imageTopicName>/camera/image_raw</imageTopicName>
        <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
        <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
        <cameraInfoTopicName>/camera/camera_info</cameraInfoTopicName>
        <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
        <frameName>camera_link</frameName>
        <pointCloudCutoff>0.5</pointCloudCutoff>
        <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Sensor Definition -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <topic>__default_topic__</topic>
      <plugin filename="libgazebo_ros_imu.so" name="imu_plugin">
        <topicName>imu</topicName>
        <bodyName>imu_link</bodyName>
        <updateRateHZ>100.0</updateRateHZ>
        <gaussianNoise>0.01</gaussianNoise>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>imu_link</frameName>
      </plugin>
      <pose>0 0 0 0 0 0</pose>
    </sensor>
  </gazebo>

</robot>
```

### Task 2: Creating a Sensor Processing Node (Optional - with ROS 2)

Create a simple ROS 2 node to process the sensor data:

```python
#!/usr/bin/env python3
# sensor_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers for each sensor
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )

        self.bridge = CvBridge()

        self.get_logger().info('Sensor Processor node initialized')

    def laser_callback(self, msg):
        """Process LiDAR data"""
        # Find minimum range to detect obstacles
        min_range = min([r for r in msg.ranges if r > msg.range_min and r < msg.range_max], default=float('inf'))

        if min_range < 1.0:  # Obstacle within 1 meter
            self.get_logger().info(f'OBSTACLE DETECTED: {min_range:.2f}m')
        else:
            self.get_logger().info(f'Distance to nearest obstacle: {min_range:.2f}m')

    def camera_callback(self, msg):
        """Process camera data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Simple processing: detect edges using Canny
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Display the edge-detected image
            cv2.imshow("Camera View - Edges", edges)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation and angular velocity
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity

        self.get_logger().info(
            f'IMU - Roll: {orientation.x:.2f}, Pitch: {orientation.y:.2f}, Yaw: {orientation.z:.2f}, '
            f'Ang Vel: ({angular_velocity.x:.2f}, {angular_velocity.y:.2f}, {angular_velocity.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        sensor_processor.get_logger().info('Sensor Processor stopped by user')
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 3: Creating a Gazebo World with Obstacles

Create a world file with various obstacles to test the sensors:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="sensor_world">
    <!-- Physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.6 -0.4 -0.8</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles for sensor testing -->
    <model name="wall_1">
      <pose>5 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_1">
      <pose>-2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="cylinder_1">
      <pose>2 -2 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Task 4: Launching the Simulation

1. Save your robot model as `sensor_robot.urdf`
2. Save your world file as `sensor_world.sdf`
3. Launch Gazebo with your world:
```bash
gzserver --verbose sensor_world.sdf
```
4. In another terminal, spawn your robot:
```bash
gz model -f sensor_robot.urdf -m sensor_robot -x 0 -y 0 -z 0.5
```
5. Visualize the sensors in Gazebo:
```bash
gzclient
```

### Task 5: Testing Sensor Data

If using ROS 2:
1. Make sure your sensor processor node is running
2. Check the sensor topics:
```bash
ros2 topic list | grep sensor
```
3. Echo sensor data:
```bash
# LiDAR data
ros2 topic echo /scan sensor_msgs/msg/LaserScan

# Camera data
ros2 topic echo /camera/image_raw sensor_msgs/msg/Image

# IMU data
ros2 topic echo /imu sensor_msgs/msg/Imu
```

### Exercise Questions

1. How does the LiDAR sensor detect obstacles in the environment?
2. What is the difference between the depth image and point cloud data from the camera?
3. How does the IMU sensor provide information about the robot's motion?
4. What happens to sensor accuracy when the robot moves quickly?
5. How would you modify the sensor parameters to detect objects at greater distances?

### Advanced Task

Enhance your sensor simulation by:
1. Adding noise models to make sensor data more realistic
2. Implementing a sensor fusion algorithm that combines data from multiple sensors
3. Creating a simple navigation algorithm that uses sensor data to avoid obstacles
4. Adding more complex sensor types like GPS or thermal cameras
5. Implementing a sensor visualization tool that displays sensor data in real-time

### Summary

This exercise demonstrates the fundamental concepts of simulated sensors in Gazebo:
- Creating robot models with integrated sensors
- Configuring sensor parameters for realistic simulation
- Understanding the different types of sensor data
- Testing sensor performance in various environments
- Processing sensor data for robot decision-making