---
sidebar_position: 3
title: "Cognitive Planning with LLMs"
description: "Using LLMs to translate natural language to ROS 2 actions"
---

# Cognitive Planning with LLMs

Large Language Models can understand natural language commands and generate sequences of robot actions.

## Using Gemini for Planning

```python
import google.generativeai as genai

genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel('gemini-1.5-flash')

class CognitivePlanner(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        self.model = model
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.plan_callback,
            10
        )
        self.action_pub = self.create_publisher(
            String,
            '/robot_action',
            10
        )
    
    def plan_callback(self, msg):
        command = msg.data
        
        # Generate plan using LLM
        prompt = f"""
        Translate this command to robot actions: "{command}"
        
        Available actions:
        - move_forward(distance)
        - turn_left(angle)
        - turn_right(angle)
        - pick_up(object)
        - place(object, location)
        - navigate_to(x, y)
        
        Return a JSON list of actions.
        """
        
        response = self.model.generate_content(prompt)
        plan = self.parse_plan(response.text)
        
        # Execute plan
        self.execute_plan(plan)
    
    def parse_plan(self, llm_output):
        # Parse LLM output to action sequence
        import json
        return json.loads(llm_output)
    
    def execute_plan(self, plan):
        for action in plan:
            msg = String()
            msg.data = json.dumps(action)
            self.action_pub.publish(msg)
            self.get_logger().info(f'Executing: {action}')
```

## Example: "Clean the Room"

```python
# User says: "Clean the room"

# LLM generates plan:
plan = [
    {"action": "navigate_to", "x": 2.0, "y": 1.0},
    {"action": "pick_up", "object": "trash"},
    {"action": "navigate_to", "x": 0.0, "y": 0.0},
    {"action": "place", "object": "trash", "location": "bin"},
    # ... more actions
]

# Execute plan step by step
```

## Context-Aware Planning

```python
class ContextAwarePlanner(CognitivePlanner):
    def get_context(self):
        # Get current robot state
        joint_states = self.get_joint_states()
        camera_image = self.get_camera_image()
        object_detections = self.detect_objects(camera_image)
        
        return {
            'position': self.get_position(),
            'objects': object_detections,
            'joint_states': joint_states
        }
    
    def plan_with_context(self, command):
        context = self.get_context()
        
        prompt = f"""
        Current state: {context}
        Command: "{command}"
        
        Generate a plan considering current state.
        """
        
        return self.model.generate_content(prompt)
```

## Best Practices

1. **Use structured prompts**: Clear format for LLM output
2. **Validate plans**: Check safety before execution
3. **Handle failures**: Re-plan if action fails
4. **Use context**: Include robot state in planning

[Next: Multi-Modal Interaction →](./multi-modal-interaction.md)

