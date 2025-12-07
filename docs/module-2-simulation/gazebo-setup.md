---
sidebar_position: 2
title: "Gazebo Setup"
description: "Install and configure Gazebo for robot simulation"
---

# Gazebo Setup

Gazebo is a powerful physics-based simulator that integrates seamlessly with ROS 2. Let's set it up and run your first simulation.

## Installing Gazebo

### On Ubuntu (ROS 2 Humble)

Gazebo comes with ROS 2 installation:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Standalone Installation

If you need the latest Gazebo version:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo11
```

## Starting Gazebo

Launch Gazebo with an empty world:

```bash
gazebo
```

Or launch with a specific world:

```bash
gazebo worlds/empty.world
```

## ROS 2 Integration

To use Gazebo with ROS 2, install the bridge:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

Launch Gazebo with ROS 2:

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

## Your First Simulation

Let's spawn a simple box in Gazebo:

1. **Start Gazebo**:
```bash
gazebo
```

2. **Insert a box**:
   - Click "Insert" tab
   - Select "Box"
   - Click in the world to place it

3. **Add physics**:
   - The box will fall due to gravity
   - You can push it around with your mouse

## Spawning a Robot

To spawn a robot from URDF:

```bash
ros2 run gazebo_ros spawn_entity.py \
    -topic robot_description \
    -entity my_robot \
    -x 0 -y 0 -z 0.5
```

This requires:
- A `robot_state_publisher` node publishing the robot description
- The robot description on the `/robot_description` topic

## Example: Spawning Humanoid Robot

Create a launch file to spawn your humanoid:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get robot description
    robot_description_path = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'humanoid.urdf'
    )
    
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(robot_description_path).read()
            }]
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'humanoid_robot',
                '-x', '0', '-y', '0', '-z', '1.0'
            ]
        ),
    ])
```

Run it:

```bash
ros2 launch humanoid_bringup gazebo.launch.py
```

## Gazebo World Files

World files define the simulation environment. Create `worlds/my_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add your models here -->
  </world>
</sdf>
```

Launch with custom world:

```bash
gazebo worlds/my_world.world
```

## Useful Gazebo Tools

### GUI Controls

- **Left click + drag**: Rotate view
- **Right click + drag**: Pan view
- **Scroll wheel**: Zoom
- **Insert tab**: Add models
- **Play/Pause**: Control simulation

### Command Line Tools

```bash
# List models in world
gz model --list

# Get model info
gz model --info -m my_robot

# Control model
gz topic -t "/cmd_vel" -m geometry_msgs.msgs.Twist -p "linear: {x: 0.5}"
```

## Troubleshooting

**Gazebo won't start**:
- Check graphics drivers are installed
- Try: `export LIBGL_ALWAYS_SOFTWARE=1` (for software rendering)

**Robot doesn't appear**:
- Check URDF is valid
- Verify robot_state_publisher is running
- Check spawn entity output for errors

**Physics issues**:
- Verify collision geometries in URDF
- Check joint limits are reasonable
- Ensure masses and inertias are set

## What's Next?

Now that Gazebo is set up, let's learn about URDF and SDF formats for describing robots in simulation.

[Next: URDF and SDF →](./urdf-sdf.md)

