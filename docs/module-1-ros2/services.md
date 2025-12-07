---
sidebar_position: 4
title: "Services"
description: "Learn how to use ROS 2 services for request/response communication"
---

# Services

While topics provide one-way, asynchronous communication, services offer synchronous request/response communication. This is essential for operations that need confirmation or return values.

## Understanding Services

Services in ROS 2 follow a client-server model:

- **Service Server**: Provides a service and responds to requests
- **Service Client**: Sends requests and waits for responses

Unlike topics, services are:

- **Synchronous**: Client waits for response
- **One-to-one**: One client, one server per request
- **Request/Response**: Each call has a request and response

## Creating a Service Server

Let's create a service that calculates the sum of two numbers:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts service server ready')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Client

Now let's create a client that calls this service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    response = client.send_request(5, 3)
    client.get_logger().info(f'Result: {response.sum}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Service Definitions

For humanoid robots, you might create custom services. For example, a service to move a joint:

```python
# MoveJoint.srv
float64 joint_name
float64 target_position
float64 max_velocity
---
bool success
string message
float64 final_position
```

The `---` separates the request (above) from the response (below).

## Service Command-Line Tools

```bash
# List all services
ros2 service list

# Get service information
ros2 service info /add_two_ints

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

## Example: Robot Control Service

Here's a more realistic example for controlling a humanoid robot arm:

```python
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MoveArm

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.srv = self.create_service(MoveArm, 'move_arm', self.move_arm_callback)
        self.current_position = 0.0

    def move_arm_callback(self, request, response):
        self.get_logger().info(f'Moving arm to position: {request.target_position}')

        # Simulate arm movement
        if abs(request.target_position) <= 1.57:  # 90 degrees
            self.current_position = request.target_position
            response.success = True
            response.message = "Arm moved successfully"
            response.final_position = self.current_position
        else:
            response.success = False
            response.message = "Target position out of range"
            response.final_position = self.current_position

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    rclpy.shutdown()
```

## When to Use Services vs Topics

**Use Topics when**:

- Broadcasting sensor data continuously
- One-to-many communication
- Asynchronous updates
- High-frequency data streams

**Use Services when**:

- Need a response/confirmation
- One-time commands
- Synchronous operations
- Requesting specific information

## Best Practices

1. **Keep service callbacks fast**: Blocking operations can delay other requests
2. **Handle errors gracefully**: Return appropriate error messages
3. **Use descriptive service names**: `/robot/arm/move` is better than `/service1`
4. **Consider timeouts**: Services should respond within reasonable time

## What's Next?

Now that you understand services, let's learn how to organize your code into ROS 2 packages.

[Next: ROS 2 Packages →](./packages.md)
