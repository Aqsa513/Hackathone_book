---
sidebar_position: 4
title: Humanoid Structure with URDF
---

# Humanoid Structure with URDF

## URDF Concepts: Links, Joints, and Frames

### What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its structure, joints, and sensors. URDF is essential for:

- Robot simulation in tools like Gazebo
- Robot visualization in RViz
- Kinematic analysis and planning
- Robot control and calibration

### Links

Links represent the rigid bodies of a robot. Each link has:

- **Visual properties**: How the link appears (geometry, material, color)
- **Collision properties**: How the link interacts in physics simulation
- **Inertial properties**: Mass, center of mass, and inertia matrix

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints

Joints define the connections between links and specify how they can move relative to each other. Common joint types include:

- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint with limits
- **Fixed**: No movement (used to attach links rigidly)
- **Floating**: 6 DOF movement
- **Planar**: Movement in a plane

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.1 0.2 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Frames

Frames in URDF establish coordinate systems for each link. The transformation between frames is defined by the joint constraints. TF (Transform) tree connects all frames in the robot, allowing for:

- Coordinate transformations
- Sensor data interpretation
- Robot state estimation
- Path planning in world coordinates

## Modeling Humanoid Bodies and Sensors

### Humanoid Robot Structure

A typical humanoid robot consists of:

- **Torso**: Main body containing the core systems
- **Head**: Contains cameras, IMU, and other sensors
- **Arms**: With shoulder, elbow, and wrist joints
- **Hands**: With multiple finger joints
- **Legs**: With hip, knee, and ankle joints
- **Feet**: For balance and locomotion

### Example Humanoid URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_shoulder_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Add more joints and links for complete humanoid structure -->
</robot>
```

### Sensors in URDF

Sensors are modeled as additional links attached to the main robot structure. Common sensors include:

- **Cameras**: For vision-based perception
- **LIDAR**: For 3D mapping and obstacle detection
- **IMU**: For orientation and acceleration
- **Force/Torque sensors**: For contact detection
- **GPS**: For outdoor localization

```xml
<!-- Camera sensor -->
<joint name="head_to_camera" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<link name="camera_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## URDF's Role in Simulation and Control

### Simulation

URDF models are crucial for robot simulation in environments like Gazebo:

1. **Physics simulation**: URDF provides inertial properties for realistic physics
2. **Collision detection**: Collision properties define how parts interact
3. **Visualization**: Visual properties determine how the robot appears
4. **Sensor simulation**: Sensors in URDF are simulated with realistic noise models

### Control

URDF enables robot control by:

1. **Kinematic chains**: Joints define how parts move relative to each other
2. **Forward kinematics**: Computing end-effector positions from joint angles
3. **Inverse kinematics**: Computing joint angles for desired end-effector positions
4. **Dynamics**: Inertial properties for dynamic control

### Example: URDF with Control Plugins

```xml
<!-- Joint state publisher -->
<ros2_control name="SystemMultiJointInterface" type="system">
  <hardware>
    <plugin>mock_components/JointSystem</plugin>
  </hardware>
  <joint name="torso_to_head">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <param name="initial_position">0.0</param>
  </joint>
  <joint name="torso_to_left_shoulder">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <param name="initial_position">0.0</param>
  </joint>
</ros2_control>
```

## Best Practices for URDF Modeling

### File Organization

- Keep URDF files modular by separating common components
- Use Xacro for complex robots to avoid repetition
- Include proper documentation in comments

### Scaling and Units

- Use consistent units (meters for distance, kilograms for mass)
- Ensure realistic physical properties
- Scale models appropriately for visualization and simulation

### Validation

- Use `check_urdf` command to validate URDF files
- Visualize in RViz to verify structure
- Test in simulation to ensure proper physics behavior

### Xacro for Complex Models

For complex humanoid robots, use Xacro (XML Macros) to simplify the URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Macro for creating a joint -->
  <xacro:macro name="simple_joint" params="name type parent child origin_xyz axis_xyz">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${origin_xyz}" rpy="0 0 0"/>
      <axis xyz="${axis_xyz}"/>
      <limit lower="-${M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_joint name="shoulder_joint" type="revolute"
                      parent="torso" child="upper_arm"
                      origin_xyz="0.2 0 0" axis_xyz="0 1 0"/>

</robot>
```

## Summary

URDF is fundamental to representing humanoid robots in ROS. Understanding links, joints, and frames is crucial for creating accurate robot models. Properly modeling humanoid structures with appropriate sensors enables effective simulation and control. Following best practices ensures that URDF models are valid, efficient, and suitable for both simulation and real-world robot control.