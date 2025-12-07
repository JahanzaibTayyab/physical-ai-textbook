---
sidebar_position: 2
title: "Isaac Sim"
description: "Photorealistic simulation and synthetic data generation"
---

# Isaac Sim

Isaac Sim is NVIDIA's photorealistic robotics simulator built on Omniverse. It provides advanced graphics, physics, and AI capabilities for training and testing robots.

## Installing Isaac Sim

### System Requirements

- NVIDIA GPU with RTX support (recommended)
- CUDA-capable GPU
- 16GB+ RAM
- Ubuntu 20.04/22.04 or Windows

### Installation

```bash
# Download Isaac Sim
# Visit: https://developer.nvidia.com/isaac-sim

# Extract and run
./isaac-sim.sh
```

## Basic Usage

### Starting Isaac Sim

```bash
./isaac-sim.sh
```

### Creating a Scene

1. **Add Ground**: Insert → Ground Plane
2. **Add Robot**: Import your URDF/SDF
3. **Add Objects**: Insert various objects
4. **Configure Lighting**: Set up realistic lighting

## ROS 2 Integration

### Isaac Sim ROS Bridge

Isaac Sim can communicate with ROS 2:

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import omni.isaac.core.utils.nucleus as nucleus_utils

# Create world
world = World()

# Load robot
robot_prim_path = "/humanoid_robot"
robot = world.scene.add(
    Robot(
        prim_path=robot_prim_path,
        name="humanoid",
        position=np.array([0, 0, 1.0])
    )
)

# Enable ROS 2 bridge
from omni.isaac.core_nodes.scripts.utils import set_target_prims
set_target_prims(primPath=robot_prim_path, targetPrims=[robot_prim_path])
```

## Synthetic Data Generation

Isaac Sim excels at generating synthetic training data:

### Camera Data

```python
import omni.isaac.sensor as sensor

# Add camera
camera = sensor.Camera(
    prim_path="/camera",
    name="camera",
    position=np.array([0, 0, 2]),
    frequency=30
)

# Capture images
rgb_image = camera.get_rgba()
depth_image = camera.get_depth()
```

### Dataset Generation

```python
def generate_dataset(num_samples=1000):
    dataset = []
    for i in range(num_samples):
        # Randomize scene
        randomize_objects()
        randomize_lighting()
        
        # Capture data
        rgb = camera.get_rgba()
        depth = camera.get_depth()
        annotations = get_annotations()
        
        dataset.append({
            'rgb': rgb,
            'depth': depth,
            'annotations': annotations
        })
    
    return dataset
```

## Physics Simulation

Isaac Sim uses PhysX for physics:

```python
from omni.isaac.core import World

world = World(physics_dt=1.0/60.0)  # 60 Hz physics

# Enable gravity
world.scene.set_default_ground_plane_prim_path("/ground")
world.scene.add_default_ground_plane()

# Add physics objects
world.scene.add(
    DynamicCuboid(
        prim_path="/cube",
        name="cube",
        position=np.array([0, 0, 1]),
        size=np.array([0.1, 0.1, 0.1])
    )
)
```

## Humanoid Robot Simulation

### Loading Humanoid

```python
from omni.isaac.core.robots import Robot

# Load humanoid from URDF
robot = world.scene.add(
    Robot(
        prim_path="/humanoid",
        name="humanoid",
        usd_path="path/to/humanoid.usd"
    )
)

# Control joints
robot.set_joint_positions(
    positions=np.array([0.5, -0.3, 0.0, ...])  # Joint angles
)
```

### Balance Simulation

```python
class BalanceSimulator:
    def __init__(self, robot):
        self.robot = robot
        self.imu = robot.get_imu()
    
    def simulate_balance(self):
        while True:
            # Get current state
            orientation = self.imu.get_orientation()
            joint_states = self.robot.get_joint_states()
            
            # Calculate balance correction
            correction = self.balance_controller(orientation)
            
            # Apply correction
            self.robot.set_joint_velocities(correction)
            
            world.step(render=True)
```

## Best Practices

1. **Use USD format**: Isaac Sim works best with USD files
2. **Optimize scenes**: Reduce complexity for real-time performance
3. **Generate diverse data**: Vary lighting, objects, poses
4. **Use GPU acceleration**: Leverage RTX features
5. **Test incrementally**: Start simple, add complexity

## What's Next?

Now let's learn about Isaac ROS for hardware-accelerated ROS 2 packages.

[Next: Isaac ROS →](./isaac-ros.md)

