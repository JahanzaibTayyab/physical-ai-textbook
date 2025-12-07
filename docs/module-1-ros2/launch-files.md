---
sidebar_position: 7
title: "Launch Files"
description: "Learn how to start and configure ROS 2 systems using launch files"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-1-ros2/launch-files"
  originalContent="sidebar_position: 7"
/>

<TranslationButton 
  chapterPath="/docs/module-1-ros2/launch-files"
  originalContent="sidebar_position: 7"
/>

# Launch Files

Launch files allow you to start multiple nodes, set parameters, and configure your robot system with a single command. This is essential for complex humanoid robots with many components.

## Why Launch Files?

A humanoid robot system might need:
- Sensor nodes (cameras, LiDAR, IMU)
- Control nodes (balance, locomotion, manipulation)
- Planning nodes (path planning, task planning)
- Visualization (RViz2)

Starting all of these manually would be tedious. Launch files automate this.

## Basic Launch File

Create a file `my_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen'
        ),
    ])
```

## Starting Multiple Nodes

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_package',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='sensor_package',
            executable='lidar_node',
            name='lidar_node'
        ),
        Node(
            package='control_package',
            executable='balance_controller',
            name='balance_controller'
        ),
    ])
```

## Setting Parameters

Parameters can be set in launch files:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_controller',
            executable='balance_controller',
            name='balance_controller',
            parameters=[{
                'kp': 100.0,
                'kd': 10.0,
                'max_torque': 50.0,
            }]
        ),
    ])
```

## Loading Parameters from YAML

For complex configurations, use YAML files:

```yaml
# config/balance_params.yaml
balance_controller:
  ros__parameters:
    kp: 100.0
    kd: 10.0
    max_torque: 50.0
    control_frequency: 100.0
```

Load in launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('humanoid_controller'),
        'config',
        'balance_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='humanoid_controller',
            executable='balance_controller',
            name='balance_controller',
            parameters=[config_file]
        ),
    ])
```

## Including Other Launch Files

Reuse launch files by including them:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sensor_package'),
                'launch',
                'sensors.launch.py'
            )
        ])
    )
    
    return LaunchDescription([
        sensor_launch,
        Node(
            package='humanoid_controller',
            executable='balance_controller',
            name='balance_controller'
        ),
    ])
```

## Complete Humanoid Robot Launch File

Here's a comprehensive example:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_description_file = LaunchConfiguration('robot_description_file')
    
    # Get package directories
    humanoid_description_dir = get_package_share_directory('humanoid_description')
    humanoid_controller_dir = get_package_share_directory('humanoid_controller')
    
    # Robot description
    robot_description_path = os.path.join(
        humanoid_description_dir,
        'urdf',
        'humanoid.urdf'
    )
    
    # Parameter files
    controller_params = os.path.join(
        humanoid_controller_dir,
        'config',
        'controller_params.yaml'
    )
    
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(robot_description_path).read(),
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Joint state publisher (for simulation/testing)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        
        # Balance controller
        Node(
            package='humanoid_controller',
            executable='balance_controller',
            name='balance_controller',
            parameters=[controller_params]
        ),
        
        # Locomotion controller
        Node(
            package='humanoid_controller',
            executable='locomotion_controller',
            name='locomotion_controller',
            parameters=[controller_params]
        ),
        
        # RViz2 (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(humanoid_description_dir, 'rviz', 'humanoid.rviz')]
        ),
    ])
```

## Running Launch Files

```bash
# Basic usage
ros2 launch my_package my_launch_file.launch.py

# With arguments
ros2 launch my_package my_launch_file.launch.py use_sim_time:=true

# From package
ros2 launch humanoid_bringup humanoid.launch.py
```

## Best Practices

1. **Organize by function**: Separate launch files for sensors, control, planning
2. **Use parameters**: Make values configurable via parameters
3. **Include conditionally**: Use conditions to include/exclude nodes
4. **Document**: Add comments explaining what each node does
5. **Test incrementally**: Test launch files as you build them

## What's Next?

Now that you understand ROS 2 basics, let's learn how to bridge Python AI agents to ROS 2 controllers.

[Next: Python Agents →](./python-agents.md)