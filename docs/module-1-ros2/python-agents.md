---
sidebar_position: 8
title: "Python Agents with rclpy"
description: "Learn how to bridge Python AI agents to ROS 2 controllers"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-1-ros2/python-agents"
  originalContent="sidebar_position: 8"
/>

<TranslationButton 
  chapterPath="/docs/module-1-ros2/python-agents"
  originalContent="sidebar_position: 8"
/>

# Python Agents with rclpy

One of the powerful aspects of ROS 2 is its Python support through `rclpy`. This allows you to integrate AI agents, machine learning models, and high-level planning with low-level robot control.

## Why Python Agents?

Python is ideal for:
- **AI/ML integration**: Easy to use libraries like TensorFlow, PyTorch
- **Rapid prototyping**: Fast development and testing
- **High-level planning**: Natural language processing, task planning
- **Data processing**: NumPy, Pandas for sensor data analysis

ROS 2's `rclpy` lets you combine Python's AI capabilities with ROS 2's robot control.

## Basic Python Agent Structure

Here's a template for a Python agent that interfaces with ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')
        
        # Subscribers - receive robot state
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers - send commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Agent initialization
        self.initialize_agent()
    
    def initialize_agent(self):
        """Initialize AI/ML models"""
        # Load your AI models here
        # self.model = load_model('path/to/model')
        self.get_logger().info('AI Agent initialized')
    
    def joint_state_callback(self, msg):
        """Process sensor data and make decisions"""
        # Process joint states
        # Make AI-based decisions
        # Publish commands
        pass

def main(args=None):
    rclpy.init(args=args)
    agent = AIAgent()
    rclpy.spin(agent)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example: Vision-Based Navigation Agent

Here's a more complete example that processes camera data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNavigationAgent(Node):
    def __init__(self):
        super().__init__('vision_navigation_agent')
        
        # Image subscriber
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Command publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info('Vision Navigation Agent started')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process image (simplified example)
            # In practice, you'd use your AI model here
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            
            # Simple navigation logic (replace with AI)
            # Find center of detected features
            M = cv2.moments(edges)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                image_center = cv_image.shape[1] / 2
                
                # Calculate steering
                error = cx - image_center
                angular_z = -0.01 * error  # Proportional control
                
                # Publish command
                cmd = Twist()
                cmd.linear.x = 0.2
                cmd.angular.z = angular_z
                self.cmd_pub.publish(cmd)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigationAgent()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating Machine Learning Models

Here's how to integrate a trained ML model:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
from PIL import Image as PILImage

class MLControlAgent(Node):
    def __init__(self):
        super().__init__('ml_control_agent')
        
        # Load your trained model
        self.model = torch.load('path/to/model.pth')
        self.model.eval()
        
        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])
        
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        try:
            # Convert and preprocess
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            pil_image = PILImage.fromarray(cv_image)
            input_tensor = self.transform(pil_image).unsqueeze(0)
            
            # Run inference
            with torch.no_grad():
                output = self.model(input_tensor)
                # Process output (example: output is [linear_x, angular_z])
                linear_x = float(output[0][0])
                angular_z = float(output[0][1])
            
            # Publish command
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            self.cmd_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'ML inference error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MLControlAgent()
    rclpy.spin(node)
    rclpy.shutdown()
```

## High-Level Planning Agent

For task-level planning:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import ExecuteTask

class TaskPlanningAgent(Node):
    def __init__(self):
        super().__init__('task_planning_agent')
        
        # Service client for executing tasks
        self.execute_client = self.create_client(
            ExecuteTask,
            'execute_task'
        )
        
        # Command subscriber (from LLM or user)
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
    
    def command_callback(self, msg):
        """Process natural language command and plan actions"""
        command = msg.data
        
        # Parse command (simplified - in practice use NLP)
        if "pick up" in command.lower():
            # Plan pick-up sequence
            self.plan_pickup_sequence()
        elif "move to" in command.lower():
            # Plan navigation
            self.plan_navigation(command)
        # ... more command parsing
    
    def plan_pickup_sequence(self):
        """Plan sequence of actions for picking up object"""
        sequence = [
            "navigate_to_object",
            "grasp_object",
            "lift_object"
        ]
        
        for action in sequence:
            self.execute_action(action)
    
    def execute_action(self, action_name):
        """Execute a single action via service call"""
        request = ExecuteTask.Request()
        request.action = action_name
        future = self.execute_client.call_async(request)
        # Handle response...

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanningAgent()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Best Practices

1. **Separate concerns**: Keep AI logic separate from ROS 2 communication
2. **Handle errors**: AI models can fail - handle gracefully
3. **Use async**: For long-running AI operations, use async callbacks
4. **Monitor performance**: Log inference times and resource usage
5. **Test offline**: Test AI models before integrating with ROS 2

## Module 1 Summary

Congratulations! You've learned:
- ✅ ROS 2 architecture and concepts
- ✅ Creating nodes, publishers, and subscribers
- ✅ Using services for request/response
- ✅ Organizing code into packages
- ✅ Describing robots with URDF
- ✅ Starting systems with launch files
- ✅ Bridging Python AI agents to ROS 2

You're now ready to move on to Module 2: Simulation!

[Next Module: The Digital Twin (Gazebo & Unity) →](../module-2-simulation/intro.md)