---
sidebar_position: 5
title: "Unity Rendering"
description: "High-fidelity rendering and human-robot interaction in Unity"
---

# Unity Rendering

Unity provides photorealistic rendering capabilities that complement Gazebo's physics simulation. Use Unity for visualization, VR/AR, and human-robot interaction studies.

## Why Unity?

Unity offers:

- **Photorealistic graphics**: Advanced lighting and materials
- **VR/AR support**: Test human-robot interaction in virtual reality
- **Performance**: Optimized rendering pipeline
- **Asset store**: Rich library of 3D models and tools
- **Cross-platform**: Windows, macOS, Linux, mobile

## Unity ROS 2 Integration

### ROS-TCP-Connector

Use ROS-TCP-Connector to connect Unity with ROS 2:

1. **Install Unity** (2021.3 LTS or newer)
2. **Import ROS-TCP-Connector** package
3. **Configure connection** to ROS 2

### Basic Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROS2Connection : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect("localhost", 10000);
    }
}
```

## Importing Robot Models

### From URDF

Use URDF Importer to load your robot:

1. Install **URDF Importer** package
2. Import URDF file
3. Unity automatically creates GameObjects

### Manual Import

For more control, import meshes manually:

```csharp
// Load robot mesh
GameObject robot = Instantiate(Resources.Load<GameObject>("HumanoidRobot"));
robot.transform.position = new Vector3(0, 0, 0);
```

## ROS 2 Communication in Unity

### Publishing Joint States

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStatePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/joint_states";
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    void Update()
    {
        JointStateMsg jointState = new JointStateMsg();
        jointState.name = new string[] {"left_shoulder", "left_elbow"};
        jointState.position = new double[] {0.5, -0.3};
        
        ros.Publish(topicName, jointState);
    }
}
```

### Subscribing to Commands

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CommandSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/cmd_vel";
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, MoveRobot);
    }
    
    void MoveRobot(TwistMsg message)
    {
        // Apply velocity to robot
        transform.Translate(
            new Vector3(
                (float)message.linear.x,
                0,
                0
            ) * Time.deltaTime
        );
    }
}
```

## Human-Robot Interaction

Unity excels at HRI studies:

### VR Integration

```csharp
using UnityEngine.XR;

public class VRHumanController : MonoBehaviour
{
    void Update()
    {
        // Get VR controller position
        Vector3 handPosition = InputTracking.GetLocalPosition(
            XRNode.RightHand
        );
        
        // Send to ROS 2 for robot to follow
        PublishHandPosition(handPosition);
    }
}
```

### Gesture Recognition

Use Unity's ML-Agents or integrate external libraries:

```csharp
public class GestureRecognizer : MonoBehaviour
{
    public void OnGestureDetected(string gesture)
    {
        // Send gesture command to ROS 2
        PublishGestureCommand(gesture);
    }
}
```

## Visual Effects

### Lighting

Configure realistic lighting:

```csharp
Light mainLight = GameObject.Find("Directional Light").GetComponent<Light>();
mainLight.type = LightType.Directional;
mainLight.intensity = 1.0f;
mainLight.shadows = LightShadows.Soft;
```

### Materials

Create realistic materials:

```csharp
Material robotMaterial = new Material(Shader.Find("Standard"));
robotMaterial.SetColor("_Color", Color.blue);
robotMaterial.SetFloat("_Metallic", 0.5f);
robotMaterial.SetFloat("_Glossiness", 0.8f);
```

## Performance Optimization

### Level of Detail (LOD)

Use LOD for complex models:

```csharp
LODGroup lodGroup = robot.AddComponent<LODGroup>();
LOD[] lods = new LOD[3];
// Configure LOD levels based on distance
lodGroup.SetLODs(lods);
```

### Occlusion Culling

Enable occlusion culling for better performance:

```csharp
Camera.main.useOcclusionCulling = true;
```

## Best Practices

1. **Separate physics and rendering**: Use Gazebo for physics, Unity for visuals
2. **Optimize models**: Reduce polygon count for real-time performance
3. **Use LOD**: Different detail levels based on distance
4. **Batch rendering**: Combine similar objects
5. **Profile performance**: Use Unity Profiler to identify bottlenecks

## What's Next?

Now let's learn how to simulate sensors like LiDAR, cameras, and IMUs.

[Next: Sensor Simulation →](./sensor-simulation.md)

