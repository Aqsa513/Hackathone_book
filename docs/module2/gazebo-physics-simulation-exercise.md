---
sidebar_position: 14
title: Gazebo Physics Simulation Exercise
---

# Gazebo Physics Simulation - Hands-on Exercise

## Exercise: Creating a Physics-Based Robot Simulation

In this exercise, you'll create a simple robot simulation in Gazebo with realistic physics properties.

### Prerequisites

- Gazebo installed (Gazebo 11 or newer)
- Basic understanding of URDF/SDF
- ROS 2 installed for integration (optional)

### Task 1: Creating a Simple Robot Model

Create a basic wheeled robot model with proper physics properties:

```xml
<?xml version="1.0" ?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Robot properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.5 0.3 0.2" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="0.5 0.3 0.2" />
      </geometry>
    </collision>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 ${M_PI/2} 0" />
      <geometry>
        <cylinder radius="0.1" length="0.05" />
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 ${M_PI/2} 0" />
      <geometry>
        <cylinder radius="0.1" length="0.05" />
      </geometry>
    </collision>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 ${M_PI/2} 0" />
      <geometry>
        <cylinder radius="0.1" length="0.05" />
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 ${M_PI/2} 0" />
      <geometry>
        <cylinder radius="0.1" length="0.05" />
      </geometry>
    </collision>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="left_wheel" />
    <origin xyz="0 0.2 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="right_wheel" />
    <origin xyz="0 -0.2 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
  </joint>
</robot>
```

### Task 2: Creating a Gazebo World File

Create a world file with physics properties:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <!-- Physics engine configuration -->
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
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
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
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a simple obstacle -->
    <model name="box_obstacle">
      <pose>-2 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.083</iyy>
            <iyz>0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Task 3: Launching the Simulation

1. Save your robot model as `simple_robot.urdf`
2. Save your world file as `simple_world.sdf`
3. Launch Gazebo with your world:
```bash
gzserver --verbose simple_world.sdf
```
4. In another terminal, spawn your robot:
```bash
gz model -f simple_robot.urdf -m simple_robot -x 0 -y 0 -z 0.5
```

### Task 4: Experimenting with Physics Parameters

1. Change the gravity to simulate different environments (e.g., moon gravity: `0 0 -1.62`)
2. Modify friction parameters to see how they affect robot movement
3. Adjust the robot's mass and inertia properties to see the impact on movement
4. Change the time step size in the physics configuration and observe the effect on simulation stability

### Exercise Questions

1. How does changing the robot's mass affect its movement in the simulation?
2. What happens when you reduce the friction coefficient of the ground plane?
3. How does the time step size affect the stability of the simulation?
4. What would happen if you removed the collision properties from your robot model?
5. How could you add a sensor to your robot to make it more realistic?

### Advanced Task

Modify the robot to include:
1. A simple LiDAR sensor on top of the robot
2. An IMU sensor to measure orientation
3. A camera sensor for visual input
4. Proper controllers to move the wheels using ROS

### Summary

This exercise demonstrates the fundamental concepts of physics simulation in Gazebo, including:
- Proper URDF model creation with physics properties
- World configuration with realistic physics parameters
- The relationship between mass, inertia, and movement
- The impact of environmental parameters on simulation behavior