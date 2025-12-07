---
sidebar_position: 7
title: "Sim-to-Real Transfer"
description: "Transferring learned behaviors from simulation to real robots"
---

# Sim-to-Real Transfer

Sim-to-real transfer is the process of deploying policies and behaviors learned in simulation to real robots.

## The Sim-to-Real Gap

Differences between simulation and reality:
- **Physics**: Friction, dynamics, contact
- **Sensors**: Noise, latency, calibration
- **Actuators**: Backlash, delays, limits
- **Environment**: Lighting, textures, objects

## Domain Randomization

Randomize simulation to improve transfer:

```python
class RandomizedEnv(HumanoidRLEnv):
    def randomize_domain(self):
        # Randomize physics
        self.set_friction(
            uniform(0.1, 0.5)
        )
        
        # Randomize sensor noise
        self.set_sensor_noise(
            normal(0, 0.01)
        )
        
        # Randomize lighting
        self.set_lighting(
            uniform(0.5, 1.5)
        )
```

## Progressive Transfer

Gradually increase realism:

1. **Simple simulation**: Basic physics
2. **Add noise**: Sensor and actuator noise
3. **Realistic physics**: Match real robot parameters
4. **Real robot**: Deploy and fine-tune

## Calibration

Match simulation to real robot:

```python
def calibrate_simulation(real_robot_data):
    # Measure real robot parameters
    real_mass = measure_mass()
    real_inertia = measure_inertia()
    real_friction = measure_friction()
    
    # Update simulation
    sim_robot.set_mass(real_mass)
    sim_robot.set_inertia(real_inertia)
    sim_robot.set_friction(real_friction)
```

## Deployment Strategy

### 1. Validate in Simulation

```python
# Test policy in simulation first
test_results = evaluate_policy(model, sim_env, n_episodes=100)
assert test_results['success_rate'] > 0.9
```

### 2. Safe Deployment

```python
class SafeDeployment(Node):
    def __init__(self):
        super().__init__('safe_deployment')
        self.model = PPO.load("humanoid_policy")
        self.safety_monitor = SafetyMonitor()
    
    def control_loop(self):
        obs = self.get_observation()
        action = self.model.predict(obs)
        
        # Safety checks
        if self.safety_monitor.is_safe(action):
            self.apply_action(action)
        else:
            self.apply_safe_action()
```

### 3. Fine-Tuning

```python
# Continue training on real robot with small learning rate
real_env = RealRobotEnv()
model.set_env(real_env)
model.learn(total_timesteps=10_000, reset_num_timesteps=False)
```

## Best Practices

1. **Extensive simulation training**: Train thoroughly in sim
2. **Domain randomization**: Expose to varied conditions
3. **Calibrate carefully**: Match sim to real parameters
4. **Deploy safely**: Use safety monitors
5. **Fine-tune gradually**: Small adjustments on real robot

## Module 3 Summary

Congratulations! You've learned:
- ✅ Isaac Sim for photorealistic simulation
- ✅ Isaac ROS for GPU-accelerated ROS 2
- ✅ Nav2 for humanoid navigation
- ✅ AI-powered perception and manipulation
- ✅ Reinforcement learning for control
- ✅ Sim-to-real transfer techniques

Ready for Module 4: Vision-Language-Action!

[Next Module: Vision-Language-Action (VLA) →](../module-4-vla/intro.md)

