---
sidebar_position: 6
title: "URDF Basics"
description: "Learn how to describe humanoid robot structure using URDF"
---

# URDF Basics

URDF (Unified Robot Description Format) is an XML format used to describe the physical structure of a robot, including links (rigid bodies), joints (connections between links), and their properties.

## Why URDF for Humanoid Robots?

Humanoid robots have complex structures:
- Multiple links (torso, arms, legs, head)
- Many joints (shoulders, elbows, hips, knees)
- Sensors and actuators attached to links
- Visual and collision geometries

URDF provides a standard way to describe all of this.

## Basic URDF Structure

A minimal URDF file looks like this:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>
  
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Key Elements

### Links

Links represent rigid bodies (parts of the robot):

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### Joints

Joints connect links and define how they move:

```xml
<joint name="left_hip" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="200" velocity="5"/>
</joint>
```

**Joint Types**:
- `fixed`: No movement
- `revolute`: Rotational (like a hinge)
- `prismatic`: Linear (like a slider)
- `continuous`: Unlimited rotation
- `planar`: Movement in a plane

## Humanoid Robot Example

Here's a simplified humanoid robot URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  
  <!-- Base/Torso -->
  <link name="torso">
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
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>
  
  <!-- Left Arm -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="10"/>
  </joint>
  
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.36" effort="30" velocity="10"/>
  </joint>
  
  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.035" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  
  <!-- Add more links and joints for right arm, legs, head, etc. -->
  
</robot>
```

## Using Meshes

For more realistic visualization, use 3D meshes:

```xml
<link name="torso">
  <visual>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/torso.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
</link>
```

## Xacro: URDF Macros

Xacro allows you to create reusable URDF components:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <xacro:macro name="arm" params="side">
    <joint name="${side}_shoulder" type="revolute">
      <parent link="torso"/>
      <child link="${side}_upper_arm"/>
      <!-- ... -->
    </joint>
    <link name="${side}_upper_arm">
      <!-- ... -->
    </link>
  </xacro:macro>
  
  <xacro:arm side="left"/>
  <xacro:arm side="right"/>
  
</robot>
```

## Viewing URDF

Use RViz2 to visualize your robot:

```bash
ros2 run rviz2 rviz2
```

Then add a RobotModel display and set the description parameter.

## Best Practices

1. **Start simple**: Begin with basic shapes, add meshes later
2. **Use Xacro**: For complex robots, use Xacro for reusability
3. **Include collision**: Always define collision geometry
4. **Set realistic limits**: Joint limits should match hardware
5. **Test in simulation**: Verify URDF works in Gazebo

## What's Next?

Now that you can describe robot structure, let's learn how to use launch files to start and configure robot systems.

[Next: Launch Files →](./launch-files.md)

