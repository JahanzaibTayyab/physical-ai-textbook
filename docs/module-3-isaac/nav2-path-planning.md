---
sidebar_position: 4
title: "Nav2 Path Planning"
description: "Path planning for bipedal humanoid movement"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-3-isaac/nav2-path-planning"
  originalContent="sidebar_position: 4"
/>

<TranslationButton 
  chapterPath="/docs/module-3-isaac/nav2-path-planning"
  originalContent="sidebar_position: 4"
/>

# Nav2 Path Planning

Nav2 provides path planning and navigation for mobile robots, adapted for humanoid bipedal locomotion.

## Nav2 Architecture

Nav2 consists of:
- **Planner**: Generates paths
- **Controller**: Follows paths
- **Recovery**: Handles failures
- **BT Navigator**: Behavior tree coordination

## Configuration for Humanoids

### Costmap Configuration

```yaml
local_costmap:
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  global_frame: odom
  resolution: 0.05
  robot_radius: 0.3  # Humanoid footprint radius
```

### Planner Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
```

## Path Planning for Bipedal Locomotion

### Custom Planner

```python
from nav2_simple_commander import BasicNavigator

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')
        self.navigator = BasicNavigator()
    
    def navigate_to_goal(self, x, y, theta):
        # Set initial pose
        initial_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.navigator.setInitialPose(initial_pose)
        
        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        
        # Navigate to goal
        goal_pose = self.create_pose_stamped(x, y, theta)
        self.navigator.goToPose(goal_pose)
        
        # Wait for result
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # Process feedback...
```

## Obstacle Avoidance

Nav2 automatically handles obstacles:

```python
# Nav2 handles dynamic obstacles via costmaps
# Update costmap with sensor data
# Nav2 replans automatically
```

## Best Practices

1. **Tune costmaps**: Adjust for humanoid footprint
2. **Set realistic speeds**: Account for bipedal constraints
3. **Handle failures**: Implement recovery behaviors
4. **Test in simulation**: Validate before real robot

[Next: Perception and Manipulation →](./perception-manipulation.md)