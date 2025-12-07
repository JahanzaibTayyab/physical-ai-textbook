---
sidebar_position: 6
title: "Sensor Simulation"
description: "Simulating LiDAR, depth cameras, and IMUs in Gazebo"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-2-simulation/sensor-simulation"
  originalContent="sidebar_position: 6"
/>

<TranslationButton 
  chapterPath="/docs/module-2-simulation/sensor-simulation"
  originalContent="sidebar_position: 6"
/>

# Sensor Simulation

Accurate sensor simulation is crucial for developing perception algorithms. Gazebo can simulate various sensors with realistic noise and characteristics.

## Camera Simulation

### Adding a Camera

Add a camera sensor to your robot:

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
  </visual>
  
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.05</near>
        <far>3.0</far>
      </clip>
    </camera>
    
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
      <min_depth>0.05</min_depth>
      <max_depth>3.0</max_depth>
    </plugin>
  </sensor>
</link>
```

### Accessing Camera Data in ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)
```

## Depth Camera

### Adding Depth Camera

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  
  <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/depth_camera</namespace>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_camera_link</frame_name>
    <min_depth>0.05</min_depth>
    <max_depth>5.0</max_depth>
  </plugin>
</sensor>
```

### Processing Depth Data

```python
from sensor_msgs.msg import Image
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.bridge = CvBridge()
    
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        # Process depth data
        # Find obstacles, calculate distances, etc.
```

## LiDAR Simulation

### Adding LiDAR

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  
  <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/lidar</namespace>
    </ros>
    <frame_name>lidar_link</frame_name>
    <topic_name>scan</topic_name>
  </plugin>
</sensor>
```

### Processing LiDAR Data

```python
from sensor_msgs.msg import LaserScan

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10
        )
    
    def scan_callback(self, msg):
        # Process laser scan
        ranges = msg.ranges
        min_range = min(ranges)
        min_index = ranges.index(min_range)
        angle = msg.angle_min + min_index * msg.angle_increment
        
        self.get_logger().info(
            f'Closest obstacle at {min_range}m, angle {angle}rad'
        )
```

## IMU Simulation

### Adding IMU

```xml
<sensor name="imu" type="imu">
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
      <!-- Similar for y and z -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
      <!-- Similar for y and z -->
    </linear_acceleration>
  </imu>
  
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/imu</namespace>
    </ros>
    <frame_name>imu_link</frame_name>
    <topic_name>data</topic_name>
  </plugin>
</sensor>
```

### Using IMU Data

```python
from sensor_msgs.msg import Imu

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
    
    def imu_callback(self, msg):
        # Extract orientation
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        
        # Use for balance control
        self.calculate_balance_correction(orientation, angular_velocity)
```

## Sensor Noise

Add realistic noise to sensors:

```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>
  <bias_mean>0.0</bias_mean>
  <bias_stddev>0.0</bias_stddev>
</noise>
```

## Sensor Fusion Example

Combine multiple sensors:

```python
class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        # Subscribe to multiple sensors
        self.camera_sub = self.create_subscription(...)
        self.lidar_sub = self.create_subscription(...)
        self.imu_sub = self.create_subscription(...)
    
    def fuse_data(self, camera_data, lidar_data, imu_data):
        # Combine sensor data for better perception
        # E.g., use camera for object detection, LiDAR for distance
        pass
```

## Best Practices

1. **Match real sensors**: Configure simulation to match hardware specs
2. **Add noise**: Realistic noise improves algorithm robustness
3. **Test algorithms**: Use simulated sensors to develop perception
4. **Calibrate**: Match sensor characteristics to real hardware
5. **Monitor performance**: Sensor simulation can be computationally expensive

## Module 2 Summary

Congratulations! You've learned:
- ✅ Setting up Gazebo for simulation
- ✅ Working with URDF and SDF formats
- ✅ Physics simulation (gravity, collisions, dynamics)
- ✅ Unity for high-fidelity rendering
- ✅ Simulating sensors (cameras, LiDAR, IMU)

You're ready for Module 3: NVIDIA Isaac!

[Next Module: The AI-Robot Brain (NVIDIA Isaac) →](../module-3-isaac/intro.md)