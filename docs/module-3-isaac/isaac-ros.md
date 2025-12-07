---
sidebar_position: 3
title: "Isaac ROS"
description: "Hardware-accelerated VSLAM and navigation with Isaac ROS"
---

# Isaac ROS

Isaac ROS provides GPU-accelerated ROS 2 packages for high-performance robotics applications, including Visual SLAM (VSLAM), navigation, and perception.

## Installing Isaac ROS

```bash
# Install Isaac ROS
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
./scripts/run_dev.sh
```

## Visual SLAM (VSLAM)

Isaac ROS provides hardware-accelerated VSLAM:

### Using Isaac ROS VSLAM

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### VSLAM Node

```python
from isaac_ros_visual_slam import VisualSlamNode

# Create VSLAM node
vslam_node = VisualSlamNode()
vslam_node.set_parameters({
    'enable_rectified_pose': True,
    'enable_imu_fusion': True
})
```

## Navigation Stack

Isaac ROS integrates with Nav2 for navigation:

### Nav2 Configuration

```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    use_map_topic: true
    first_map_only: false

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
```

### Running Nav2

```bash
ros2 launch nav2_bringup navigation_launch.py
```

## GPU-Accelerated Perception

Isaac ROS provides GPU-accelerated perception:

### Object Detection

```python
from isaac_ros_tensor_list_interfaces.msg import TensorList

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
    
    def image_callback(self, msg):
        # GPU-accelerated object detection
        detections = self.detect_objects(msg)
        # Process detections...
```

## Best Practices

1. **Use GPU acceleration**: Leverage CUDA for performance
2. **Optimize data flow**: Minimize CPU-GPU transfers
3. **Monitor performance**: Use profiling tools
4. **Test on hardware**: Verify GPU acceleration works

[Next: Nav2 Path Planning →](./nav2-path-planning.md)

