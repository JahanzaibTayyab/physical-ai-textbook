---
sidebar_position: 2
title: "ROS 2 Architecture"
description: "Understanding the core architecture and concepts of ROS 2"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-1-ros2/architecture"
  originalContent="sidebar_position: 2"
/>

<TranslationButton 
  chapterPath="/docs/module-1-ros2/architecture"
  originalContent="sidebar_position: 2"
/>

# ROS 2 Architecture

ROS 2 follows a distributed, node-based architecture where different components of a robot system communicate through a middleware layer. Understanding this architecture is fundamental to working with ROS 2.

## Core Concepts

### Nodes

A **node** is a process that performs computation. Each node is responsible for a specific task:

- A sensor node might read data from a camera
- A control node might process sensor data and send commands
- An actuator node might receive commands and move a motor

Nodes are independent processes that can run on the same machine or different machines across a network.

### Topics

**Topics** are named buses over which nodes exchange messages. Topics implement a publish/subscribe communication pattern:

- **Publishers** send messages to topics
- **Subscribers** receive messages from topics
- Multiple nodes can publish to the same topic
- Multiple nodes can subscribe to the same topic

Topics are asynchronous and one-to-many—one publisher can send messages to many subscribers.

### Services

**Services** provide a request/response communication pattern:

- A **client** sends a request
- A **server** processes the request and sends a response
- Unlike topics, services are synchronous and one-to-one

Services are useful for operations that require a response, like asking a robot to move to a specific position and waiting for confirmation.

## ROS 2 Middleware (DDS)

ROS 2 uses **Data Distribution Service (DDS)** as its middleware layer. DDS provides:

- **Discovery**: Nodes automatically find each other
- **Quality of Service (QoS)**: Control over message delivery guarantees
- **Security**: Built-in encryption and authentication
- **Performance**: Efficient message passing

You typically don't interact with DDS directly—ROS 2 handles it for you.

## Communication Patterns

### Publisher/Subscriber (Topics)

```
[Camera Node] --publishes--> /camera/image --> [Vision Node]
                                    |
                                    +--> [Recording Node]
```

This pattern is ideal for:

- Sensor data streams
- Continuous state updates
- Broadcasting information

### Request/Response (Services)

```
[Control Node] --request--> /move_arm --> [Arm Controller]
                    <--response--
```

This pattern is ideal for:

- One-time commands
- Operations that need confirmation
- Actions that return results

## ROS 2 Graph

The **ROS 2 graph** is a network of nodes, topics, and services. You can visualize it using:

```bash
ros2 run rqt_graph rqt_graph
```

This shows all active nodes and how they're connected via topics and services.

## Quality of Service (QoS)

QoS policies control how messages are delivered:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Transient vs. volatile
- **History**: Keep last N messages vs. keep all
- **Deadline**: Maximum time between messages

For humanoid robots, you often need reliable, real-time communication for critical systems like balance control.

## Example: Simple ROS 2 System

Let's look at a simple example of a ROS 2 system with two nodes:

```
[Talker Node] --publishes--> /chatter --> [Listener Node]
```

The talker node publishes messages, and the listener node receives them. This is the "Hello World" of ROS 2.

## ROS 2 vs ROS 1

If you're familiar with ROS 1, here are key differences:

- **DDS middleware**: More reliable and performant than ROS 1's custom middleware
- **Real-time support**: Better support for real-time applications
- **Cross-platform**: Works on Windows, macOS, and Linux
- **Improved security**: Built-in security features
- **Better Python support**: Python 3 from the start

## What's Next?

Now that you understand the architecture, let's create your first ROS 2 nodes and see how topics work in practice.

[Next: Nodes and Topics →](./nodes-and-topics.md)