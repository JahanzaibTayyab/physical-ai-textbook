import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  courseSidebar: [
    "intro",
    {
      type: "category",
      label: "Module 1: The Robotic Nervous System (ROS 2)",
      items: [
        "module-1-ros2/intro",
        "module-1-ros2/architecture",
        "module-1-ros2/nodes-and-topics",
        "module-1-ros2/services",
        "module-1-ros2/packages",
        "module-1-ros2/urdf-basics",
        "module-1-ros2/launch-files",
        "module-1-ros2/python-agents",
      ],
    },
    {
      type: "category",
      label: "Module 2: The Digital Twin (Gazebo & Unity)",
      items: [
        "module-2-simulation/intro",
        "module-2-simulation/gazebo-setup",
        "module-2-simulation/urdf-sdf",
        "module-2-simulation/physics-simulation",
        "module-2-simulation/unity-rendering",
        "module-2-simulation/sensor-simulation",
      ],
    },
    {
      type: "category",
      label: "Module 3: The AI-Robot Brain (NVIDIA Isaac)",
      items: [
        "module-3-isaac/intro",
        "module-3-isaac/isaac-sim",
        "module-3-isaac/isaac-ros",
        "module-3-isaac/nav2-path-planning",
        "module-3-isaac/perception-manipulation",
        "module-3-isaac/reinforcement-learning",
        "module-3-isaac/sim-to-real",
      ],
    },
    {
      type: "category",
      label: "Module 4: Vision-Language-Action (VLA)",
      items: [
        "module-4-vla/intro",
        "module-4-vla/voice-to-action",
        "module-4-vla/cognitive-planning",
        "module-4-vla/multi-modal-interaction",
        "module-4-vla/capstone-project",
      ],
    },
  ],
};

export default sidebars;
