---
sidebar_position: 3
title: "Nodes and Topics"
description: "Learn how to create ROS 2 nodes and communicate using topics"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-1-ros2/nodes-and-topics"
  originalContent="sidebar_position: 3"
/>

<TranslationButton 
  chapterPath="/docs/module-1-ros2/nodes-and-topics"
  originalContent="sidebar_position: 3"
/>

# Nodes and Topics

In this lesson, you'll learn how to create ROS 2 nodes and use topics for communication. This is the foundation of ROS 2 programming.

## Creating Your First Node

Let's create a simple ROS 2 node in Python. First, create a new file called `simple_node.py`:

```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Understanding the Code

- `rclpy.init()`: Initializes ROS 2
- `Node`: Base class for all ROS 2 nodes
- `get_logger()`: Provides logging functionality
- `rclpy.spin()`: Keeps the node alive and processes callbacks
- `rclpy.shutdown()`: Cleans up ROS 2 resources

## Creating a Publisher Node

A publisher node sends messages to a topic. Here's an example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components

- `create_publisher()`: Creates a publisher for a topic
  - First argument: Message type (`String`)
  - Second argument: Topic name (`'chatter'`)
  - Third argument: Queue size (`10`)
- `create_timer()`: Creates a timer that calls a function periodically
- `publish()`: Sends a message to the topic

## Creating a Subscriber Node

A subscriber node receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components

- `create_subscription()`: Creates a subscriber for a topic
  - First argument: Message type
  - Second argument: Topic name
  - Third argument: Callback function
  - Fourth argument: Queue size
- The callback function is called whenever a message is received

## Running the Nodes

1. **Terminal 1** - Run the publisher:

```bash
python3 publisher_node.py
```

2. **Terminal 2** - Run the subscriber:

```bash
python3 subscriber_node.py
```

You should see the subscriber receiving messages from the publisher!

## Custom Message Types

While ROS 2 provides many built-in message types, you can create custom ones. For example, for a humanoid robot, you might create:

```python
# JointState.msg
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
```

Custom messages are defined in `.msg` files and compiled into Python/C++ code.

## Topic Command-Line Tools

ROS 2 provides useful command-line tools:

```bash
# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /chatter

# Get topic information
ros2 topic info /chatter

# Publish a message manually
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from command line'"
```

## Best Practices

1. **Use descriptive topic names**: `/robot/left_arm/joint_states` is better than `/topic1`
2. **Choose appropriate queue sizes**: Larger queues for high-frequency data
3. **Handle errors**: Check if publishers/subscribers are created successfully
4. **Use namespaces**: Organize topics with namespaces like `/robot/sensors/`

## Example: Sensor Node for Humanoid Robot

Here's a more realistic example—a sensor node that publishes joint positions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_shoulder', 'left_elbow', 'right_shoulder', 'right_elbow']
        msg.position = [0.5, -0.3, -0.5, 0.3]  # Example positions
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

## What's Next?

Now that you understand nodes and topics, let's learn about services for request/response communication.

[Next: Services →](./services.md)