---
sidebar_position: 3
title: "URDF and SDF Formats"
description: "Understanding robot description formats for simulation"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-2-simulation/urdf-sdf"
  originalContent="sidebar_position: 3"
/>

<TranslationButton 
  chapterPath="/docs/module-2-simulation/urdf-sdf"
  originalContent="sidebar_position: 3"
/>

# URDF and SDF Formats

Both URDF and SDF are XML formats for describing robots, but they serve different purposes in simulation.

## URDF vs SDF

### URDF (Unified Robot Description Format)
- **Purpose**: ROS 2 robot description
- **Focus**: Robot structure (links, joints)
- **Limitations**: Single robot, no world description
- **Use**: ROS 2 applications, RViz2

### SDF (Simulation Description Format)
- **Purpose**: Gazebo simulation description
- **Focus**: Complete simulation (robots, world, physics)
- **Features**: Multiple robots, world elements, physics properties
- **Use**: Gazebo simulations

## Converting URDF to SDF

Gazebo can automatically convert URDF to SDF, but you may need to add simulation-specific properties.

### Using gz sdf

```bash
# Convert URDF to SDF
gz sdf -p robot.urdf > robot.sdf
```

### Manual Conversion

For better control, create an SDF file manually:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="humanoid_robot">
    <!-- Include URDF content, then add Gazebo-specific elements -->
    <include>
      <uri>model://humanoid_robot</uri>
    </include>
    
    <!-- Gazebo-specific properties -->
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find humanoid_controller)/config/controller.yaml</parameters>
    </plugin>
  </model>
</sdf>
```

## Adding Simulation Properties to URDF

You can add Gazebo-specific properties directly in URDF using `<gazebo>` tags:

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </visual>
  
  <!-- Gazebo-specific properties -->
  <gazebo>
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <selfCollide>true</selfCollide>
  </gazebo>
</link>
```

## Physics Properties

Define physics properties for realistic simulation:

```xml
<link name="torso">
  <inertial>
    <mass>10.0</mass>
    <inertia>
      <ixx>0.5</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.5</iyy>
      <iyz>0.0</iyz>
      <izz>0.5</izz>
    </inertia>
  </inertial>
  
  <gazebo>
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>  <!-- Friction coefficient -->
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>100.0</kd>      <!-- Contact damping -->
  </gazebo>
</link>
```

## Joint Properties

Configure joints for simulation:

```xml
<joint name="left_shoulder" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  
  <gazebo>
    <implicitSpringDamper>true</implicitSpringDamper>
    <provideFeedback>true</provideFeedback>
  </gazebo>
</joint>
```

## Complete SDF Example

Here's a complete SDF file for a humanoid robot:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="humanoid_robot">
    <static>false</static>
    <pose>0 0 1 0 0 0</pose>
    
    <!-- Links -->
    <link name="torso">
      <pose>0 0 0.5 0 0 0</pose>
      <visual name="torso_visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
        </material>
      </visual>
      <collision name="torso_collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <izz>0.5</izz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Joints -->
    <joint name="left_shoulder" type="revolute">
      <parent>torso</parent>
      <child>left_upper_arm</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100</effort>
          <velocity>10</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- Add more links and joints... -->
    
    <!-- ROS 2 Control Plugin -->
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find humanoid_controller)/config/controller.yaml</parameters>
    </plugin>
  </model>
</sdf>
```

## Best Practices

1. **Start with URDF**: Create URDF first, then convert to SDF
2. **Add physics properties**: Set realistic masses, inertias, friction
3. **Test in simulation**: Verify robot behaves correctly
4. **Use Xacro**: For complex robots, use Xacro for both URDF and SDF
5. **Validate**: Check files for errors before simulation

## What's Next?

Now that you understand robot description formats, let's learn about physics simulation.

[Next: Physics Simulation →](./physics-simulation.md)