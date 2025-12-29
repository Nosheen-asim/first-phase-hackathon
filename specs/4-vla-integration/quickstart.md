# Quickstart Guide: Setting up the Vision-Language-Action Educational Module

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Basic command-line knowledge
- Understanding of previous modules (ROS 2, Digital Twin, Isaac AI Brain)

## Installation Steps

### 1. Navigate to the Book Frontend Directory
```bash
cd book_frontend
```

### 2. Verify Dependencies
```bash
npm install
# or
yarn install
```

### 3. Set up the Vision-Language-Action Module Content
Create the directory structure for the VLA module:
```bash
mkdir -p docs/vla-integration
```

### 4. Add Chapter Files
Create the three main chapters for the VLA module:

**Chapter 1: Voice-to-Action Pipelines**
```bash
touch docs/vla-integration/01-voice-to-action-pipelines.md
```

**Chapter 2: LLM-Based Cognitive Planning**
```bash
touch docs/vla-integration/02-llm-cognitive-planning.md
```

**Chapter 3: VLA Execution with ROS 2**
```bash
touch docs/vla-integration/03-vla-execution-ros2.md
```

### 5. Configure Sidebar Navigation
Update `sidebars.js` to include the VLA module:
```javascript
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Module',
      items: [
        'ros2-module/01-ros2-architecture',
        'ros2-module/02-python-agents-rclpy',
        'ros2-module/03-urdf-humanoids'
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Module',
      items: [
        'digital-twin/01-gazebo-physics-simulation',
        'digital-twin/02-unity-interaction',
        'digital-twin/03-sensor-simulation'
      ],
    },
    {
      type: 'category',
      label: 'Isaac AI Brain Module',
      items: [
        'isaac-ai-brain/01-isaac-sim-synthetic-data',
        'isaac-ai-brain/02-isaac-ros-vslam',
        'isaac-ai-brain/03-nav2-humanoid-navigation',
        'isaac-ai-brain/04-perception-to-navigation'
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action Module',
      items: [
        'vla-integration/01-voice-to-action-pipelines',
        'vla-integration/02-llm-cognitive-planning',
        'vla-integration/03-vla-execution-ros2'
      ],
    },
  ],
};
```

### 6. Run the Development Server
```bash
npm run start
# or
yarn start
```

The development server will start at `http://localhost:3000`.

### 7. Build for Production
```bash
npm run build
# or
yarn build
```

## Adding Content to Chapters

### Chapter 1: Voice-to-Action Pipelines
Create content explaining:
- OpenAI Whisper integration for speech input
- Converting voice commands to structured intents
- Voice processing concepts and techniques
- Examples of voice-to-action mapping

### Chapter 2: LLM-Based Cognitive Planning
Create content explaining:
- Natural language processing for task understanding
- Translating natural language tasks into action plans
- High-level reasoning for robotic workflows
- LLM integration with robotics systems

### Chapter 3: VLA Execution with ROS 2
Create content explaining:
- Mapping plans to ROS 2 actions and services
- Coordinating perception, navigation, and manipulation
- VLA execution patterns with ROS 2
- Integration examples and best practices

## Development Workflow

1. Create or edit Markdown files in the `docs/vla-integration/` directory
2. Use the development server to preview changes
3. Commit changes to Git
4. Push to GitHub for automatic deployment (if configured)

## Content Guidelines

- Write in Markdown format
- Use clear, instructional tone appropriate for students with ROS 2 knowledge
- Include minimal code examples to illustrate concepts
- Focus on conceptual understanding rather than hands-on implementation
- Structure content with clear headings and subheadings
- Reference concepts from previous modules where appropriate

## Troubleshooting

### Common Issues

**Issue:** Content not appearing in sidebar
**Solution:** Verify the file paths in `sidebars.js` match your Markdown files

**Issue:** Images or assets not loading
**Solution:** Place assets in the `static/` directory and reference them with `/img/` paths

**Issue:** Links to previous modules not working
**Solution:** Verify that the previous module content exists and paths are correct