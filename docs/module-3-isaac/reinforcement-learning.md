---
sidebar_position: 6
title: "Reinforcement Learning for Robot Control"
description: "Training RL policies for humanoid robot control"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-3-isaac/reinforcement-learning"
  originalContent="sidebar_position: 6"
/>

<TranslationButton 
  chapterPath="/docs/module-3-isaac/reinforcement-learning"
  originalContent="sidebar_position: 6"
/>

# Reinforcement Learning for Robot Control

Reinforcement Learning (RL) enables robots to learn complex behaviors through trial and error in simulation.

## RL Basics

RL involves:
- **Agent**: The robot/controller
- **Environment**: The simulation
- **State**: Current robot state
- **Action**: Control commands
- **Reward**: Success/failure signal

## Training in Isaac Sim

### Setting Up RL Environment

```python
from omni.isaac.gym import gymapi, gymutil

class HumanoidRLEnv:
    def __init__(self):
        self.gym = gymapi.acquire_gym()
        self.sim = self.create_sim()
        self.envs = self.create_envs()
    
    def reset(self):
        # Reset robot to initial state
        return self.get_observation()
    
    def step(self, action):
        # Apply action
        self.apply_action(action)
        self.sim.step()
        
        # Get new state and reward
        obs = self.get_observation()
        reward = self.calculate_reward()
        done = self.is_done()
        
        return obs, reward, done, {}
```

## Training a Policy

### Using Stable-Baselines3

```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Create environment
env = make_vec_env(HumanoidRLEnv, n_envs=4)

# Train policy
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1_000_000)

# Save model
model.save("humanoid_policy")
```

## Balance Control Example

```python
class BalanceRLEnv(HumanoidRLEnv):
    def calculate_reward(self):
        # Reward for staying upright
        orientation = self.get_orientation()
        tilt_penalty = abs(orientation[0]) + abs(orientation[1])
        
        # Reward for maintaining position
        position_reward = -abs(self.get_position()[0])
        
        # Penalty for falling
        fall_penalty = -100 if self.is_fallen() else 0
        
        return -tilt_penalty + position_reward + fall_penalty
```

## Deploying Trained Policy

```python
# Load trained model
model = PPO.load("humanoid_policy")

# Use in ROS 2 node
class RLController(Node):
    def __init__(self):
        super().__init__('rl_controller')
        self.model = PPO.load("humanoid_policy")
        self.joint_sub = self.create_subscription(...)
    
    def control_callback(self, msg):
        # Get observation from joint states
        obs = self.get_observation(msg)
        
        # Get action from policy
        action, _ = self.model.predict(obs)
        
        # Apply action
        self.apply_action(action)
```

## Best Practices

1. **Start simple**: Begin with simple tasks
2. **Use domain randomization**: Vary simulation parameters
3. **Monitor training**: Use TensorBoard
4. **Validate policies**: Test in simulation before real robot

[Next: Sim-to-Real Transfer →](./sim-to-real.md)