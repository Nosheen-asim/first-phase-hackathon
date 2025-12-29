---
sidebar_position: 3
---

# Chapter 3: URDF for Humanoids

This chapter provides an introduction to URDF (Unified Robot Description Format) fundamentals and how to model humanoid robots with joints, links, and sensors. URDF is essential for representing robot models in ROS.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand URDF fundamentals and its purpose in robotics
- Model humanoid joints using URDF
- Model humanoid links using URDF
- Model humanoid sensors using URDF
- Create complete humanoid robot descriptions

## Introduction to URDF Fundamentals

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its kinematic structure, inertial properties, visual appearance, and collision properties.

### Key Concepts:
- **Links**: Represent rigid bodies of the robot
- **Joints**: Define connections between links
- **Visual**: Define how the robot appears visually
- **Collision**: Define collision properties for physics simulation
- **Inertial**: Define mass, center of mass, and inertia properties

### Basic URDF Structure:
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Modeling Humanoid Joints

Humanoid robots require specific joint types to mimic human-like movement. URDF supports several joint types that are essential for humanoid modeling.

### Joint Types for Humanoids:
- **Revolute**: Rotational joint with limited range of motion
- **Continuous**: Rotational joint with unlimited range of motion
- **Prismatic**: Linear sliding joint
- **Fixed**: No movement between links

### Humanoid Joint Examples:

#### Shoulder Joint (Revolute):
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0.2 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

#### Elbow Joint (Revolute):
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_link"/>
  <child link="forearm_link"/>
  <origin xyz="0.3 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="3.14" effort="100" velocity="1"/>
</joint>
```

#### Hip Joint (Ball joint simulation with 3 revolute joints):
```xml
<joint name="hip_yaw_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="thigh_link"/>
  <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.78" upper="0.78" effort="200" velocity="1"/>
</joint>
```

## Modeling Humanoid Links

Links represent the rigid bodies of a humanoid robot. Each link contains visual, collision, and inertial properties.

### Basic Link Structure:
```xml
<link name="link_name">
  <!-- Visual properties for display -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://path/to/mesh.dae"/>
      <!-- OR -->
      <box size="0.1 0.1 0.1"/>
      <!-- OR -->
      <cylinder radius="0.1" length="0.5"/>
      <!-- OR -->
      <sphere radius="0.1"/>
    </geometry>
    <material name="color_name">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision properties for physics -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties for physics simulation -->
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

### Humanoid Link Examples:

#### Head Link:
```xml
<link name="head_link">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="head_color">
      <color rgba="0.9 0.9 0.9 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
  </inertial>
</link>
```

#### Torso Link:
```xml
<link name="torso_link">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="torso_color">
      <color rgba="0.2 0.2 0.8 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.25" ixy="0" ixz="0" iyy="0.225" iyz="0" izz="0.025"/>
  </inertial>
</link>
```

## Modeling Humanoid Sensors

Humanoid robots often include various sensors for perception and interaction. In URDF, sensors are typically represented as fixed links with appropriate plugins.

### Sensor Examples:

#### Camera Sensor:
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
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

#### IMU Sensor:
```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="torso_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## Complete Humanoid Example

Here's a simplified example of a humanoid robot with torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.25" ixy="0" ixz="0" iyy="0.225" iyz="0" izz="0.025"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="50" velocity="1"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0075" ixy="0" ixz="0" iyy="0.0075" iyz="0" izz="0.000625"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="left_upper_arm_link"/>
    <origin xyz="0.2 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Summary

URDF is a fundamental component of ROS for describing robot models. For humanoid robots, URDF allows you to define the kinematic structure with appropriate joints and links, as well as specify visual, collision, and inertial properties. Proper URDF modeling is essential for simulation, visualization, and control of humanoid robots in ROS.

## Exercises

1. Create a URDF model for a simple humanoid with a torso, head, two arms, and two legs.
2. Add a camera sensor to the head of your humanoid model.
3. Modify the joint limits in your model to reflect human-like movement constraints.