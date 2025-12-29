// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Module',
      items: [
        'ros2-module/ros2-architecture',
        'ros2-module/python-agents-rclpy',
        'ros2-module/urdf-humanoids'
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Module',
      items: [
        'digital-twin/gazebo-physics-simulation',
        'digital-twin/unity-interaction',
        'digital-twin/sensor-simulation'
      ],
    },
    {
      type: 'category',
      label: 'Isaac AI Brain Module',
      items: [
        'isaac-ai-brain/isaac-sim-synthetic-data',
        'isaac-ai-brain/isaac-ros-vslam',
        'isaac-ai-brain/nav2-humanoid-navigation',
        'isaac-ai-brain/perception-to-navigation'
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action Module',
      items: [
        'vla-integration/voice-to-action-pipelines',
        'vla-integration/llm-cognitive-planning',
        'vla-integration/vla-execution-ros2'
      ],
    },
  ],
};

export default sidebars;