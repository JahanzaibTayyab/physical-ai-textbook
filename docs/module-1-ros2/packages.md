---
sidebar_position: 5
title: "ROS 2 Packages"
description: "Learn how to organize your code into ROS 2 packages"
---

import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

<PersonalizationButton 
  chapterPath="/docs/module-1-ros2/packages"
  originalContent="sidebar_position: 5"
/>

<TranslationButton 
  chapterPath="/docs/module-1-ros2/packages"
  originalContent="sidebar_position: 5"
/>

# ROS 2 Packages

A ROS 2 package is the fundamental unit for organizing and distributing your ROS 2 code. Packages contain related nodes, libraries, data files, and launch files.

## Package Structure

A typical ROS 2 package has this structure:

```
my_robot_package/
├── package.xml          # Package metadata
├── setup.py             # Python package setup (for Python packages)
├── setup.cfg            # Python package config
├── CMakeLists.txt       # Build configuration (for C++ packages)
├── my_robot_package/    # Python package directory
│   ├── __init__.py
│   └── nodes/
│       ├── __init__.py
│       └── my_node.py
├── launch/              # Launch files
│   └── my_launch_file.launch.py
└── config/              # Configuration files
    └── params.yaml
```

## Creating a Python Package

Let's create a package for a humanoid robot controller:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_controller
```

This creates the basic package structure. Now let's add a node:

```python
# humanoid_controller/humanoid_controller/nodes/balance_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def joint_states_callback(self, msg):
        # Balance control logic here
        cmd = Twist()
        # ... calculate balance commands ...
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Package.xml

The `package.xml` file defines package metadata:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_controller</name>
  <version>0.1.0</version>
  <description>Controller package for humanoid robot</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Setup.py

The `setup.py` file configures the Python package:

```python
from setuptools import setup

package_name = 'humanoid_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Controller package for humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_controller = humanoid_controller.nodes.balance_controller:main',
        ],
    },
)
```

## Building and Installing

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_controller
source install/setup.bash
```

## Running Your Package

After building and sourcing:

```bash
ros2 run humanoid_controller balance_controller
```

## Package Dependencies

Dependencies are declared in `package.xml`:

- **Build dependencies**: Needed to build the package
- **Runtime dependencies**: Needed to run the package
- **Test dependencies**: Needed only for testing

Common dependencies:
- `rclpy`: ROS 2 Python client library
- `sensor_msgs`: Standard sensor message types
- `geometry_msgs`: Geometry message types
- `std_msgs`: Standard message types

## Organizing Large Projects

For complex humanoid robot projects, organize into multiple packages:

```
humanoid_robot_ws/
├── humanoid_controller/      # Control algorithms
├── humanoid_perception/      # Vision and sensing
├── humanoid_planning/        # Motion planning
├── humanoid_description/     # URDF and meshes
└── humanoid_bringup/         # Launch files
```

## Best Practices

1. **One package, one purpose**: Keep packages focused
2. **Use namespaces**: Organize nodes with namespaces
3. **Document dependencies**: Clearly list all dependencies
4. **Version control**: Use Git for package management
5. **Follow naming conventions**: Use lowercase with underscores

## What's Next?

Now that you can create packages, let's learn about URDF files for describing robot structure.

[Next: URDF Basics →](./urdf-basics.md)