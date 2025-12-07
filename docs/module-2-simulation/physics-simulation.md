---
sidebar_position: 4
title: "Physics Simulation"
description: "Understanding gravity, collisions, and dynamics in Gazebo"
---

# Physics Simulation

Physics simulation is what makes Gazebo realistic. Understanding how physics works in simulation helps you create accurate robot models and behaviors.

## Physics Engines

Gazebo supports multiple physics engines:

- **ODE (Open Dynamics Engine)**: Default, good general-purpose engine
- **Bullet**: Better for complex collisions
- **Simbody**: Good for articulated bodies
- **DART**: Advanced dynamics and constraints

## Gravity

Gravity is enabled by default in Gazebo. Configure it in world files:

```xml
<world name="my_world">
  <physics type="ode">
    <gravity>0 0 -9.81</gravity>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
</world>
```

### Testing Gravity

Spawn a box and watch it fall:

```bash
gz model --spawn-file box.sdf --model-name test_box -x 0 -y 0 -z 5
```

The box will fall due to gravity.

## Collisions

Collision detection prevents objects from passing through each other.

### Collision Geometry

Define collision geometry (can be simpler than visual):

```xml
<link name="torso">
  <visual>
    <geometry>
      <mesh filename="torso.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
</link>
```

### Collision Properties

Configure collision behavior:

```xml
<gazebo>
  <collision name="torso_collision">
    <surface>
      <friction>
        <ode>
          <mu>0.2</mu>
          <mu2>0.2</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>
      </bounce>
    </surface>
  </collision>
</gazebo>
```

## Dynamics

Dynamics control how objects move and interact.

### Mass and Inertia

Accurate mass and inertia are crucial:

```xml
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
  <pose>0 0 0 0 0 0</pose>
</inertial>
```

### Joint Dynamics

Control joint behavior:

```xml
<joint name="left_shoulder" type="revolute">
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>10</velocity>
    </limit>
    <dynamics>
      <damping>0.1</damping>
      <friction>0.01</friction>
    </dynamics>
  </axis>
</joint>
```

## Friction

Friction affects how objects slide:

```xml
<gazebo>
  <mu1>0.2</mu1>  <!-- Static friction -->
  <mu2>0.2</mu2>  <!-- Dynamic friction -->
  <fdir1>1 0 0</fdir1>  <!-- Friction direction -->
</gazebo>
```

## Contact Forces

Monitor contact forces for balance control:

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState

class ContactMonitor(Node):
    def __init__(self):
        super().__init__('contact_monitor')
        self.subscription = self.create_subscription(
            ContactsState,
            '/gazebo/contacts',
            self.contact_callback,
            10
        )
    
    def contact_callback(self, msg):
        for contact in msg.states:
            self.get_logger().info(
                f'Contact: {contact.collision1_name} <-> {contact.collision2_name}'
            )
            # Process contact forces for balance control
```

## Balancing a Humanoid

For humanoid balance, you need:

1. **Accurate mass distribution**: Center of mass matters
2. **Contact forces**: Monitor foot contacts
3. **IMU simulation**: Sense orientation
4. **Control loop**: Adjust based on physics

Example balance controller:

```python
class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        # Subscribe to IMU and contact forces
        # Publish joint commands to maintain balance
        
    def balance_loop(self):
        # Calculate desired joint positions based on:
        # - Current orientation (from IMU)
        # - Contact forces (from contact monitor)
        # - Desired center of mass position
        pass
```

## Physics Parameters

Tune physics for your needs:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller = more accurate -->
  <real_time_factor>1.0</real_time_factor>  <!-- 1.0 = real-time -->
  <solver>
    <type>quick</type>
    <iters>10</iters>
    <sor>1.3</sor>
  </solver>
</physics>
```

## Best Practices

1. **Start simple**: Test with simple shapes first
2. **Verify masses**: Use realistic values from CAD models
3. **Test collisions**: Ensure collision geometries are correct
4. **Monitor performance**: Adjust physics parameters for speed vs accuracy
5. **Validate**: Compare simulation behavior with real robot

## What's Next?

Now that you understand physics, let's learn about Unity for high-fidelity rendering.

[Next: Unity Rendering →](./unity-rendering.md)

