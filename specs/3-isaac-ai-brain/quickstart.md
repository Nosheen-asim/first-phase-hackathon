# Quickstart Guide: Setting up the Isaac AI Brain Educational Module

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Basic command-line knowledge
- Understanding of Module 1 (ROS 2) and Module 2 (Digital Twin) concepts

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

### 3. Set up the Isaac AI Brain Module Content
Create the directory structure for the Isaac AI Brain module:
```bash
mkdir -p docs/isaac-ai-brain
```

### 4. Add Chapter Files
Create the three main chapters for the Isaac AI Brain module:

**Chapter 1: Isaac Sim & Synthetic Data**
```bash
touch docs/isaac-ai-brain/01-isaac-sim-synthetic-data.md
```

**Chapter 2: Isaac ROS & Visual SLAM**
```bash
touch docs/isaac-ai-brain/02-isaac-ros-vslam.md
```

**Chapter 3: Nav2 for Humanoid Navigation**
```bash
touch docs/isaac-ai-brain/03-nav2-humanoid-navigation.md
```

### 5. Configure Sidebar Navigation
Update `sidebars.js` to include the Isaac AI Brain module:
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
        'isaac-ai-brain/03-nav2-humanoid-navigation'
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

### Chapter 1: Isaac Sim & Synthetic Data
Create content explaining:
- NVIDIA Isaac Sim concepts and capabilities
- Photorealistic simulation for perception training
- Synthetic data generation for vision models
- Integration with Isaac tools ecosystem

### Chapter 2: Isaac ROS & Visual SLAM
Create content explaining:
- Isaac ROS and Visual SLAM concepts
- Hardware-accelerated VSLAM concepts
- Sensor fusion for localization and mapping
- GPU acceleration in perception tasks

### Chapter 3: Nav2 for Humanoid Navigation
Create content explaining:
- Nav2 for humanoid robot navigation
- Path planning fundamentals
- Navigation pipelines for bipedal humanoids
- Humanoid-specific navigation challenges

## Development Workflow

1. Create or edit Markdown files in the `docs/isaac-ai-brain/` directory
2. Use the development server to preview changes
3. Commit changes to Git
4. Push to GitHub for automatic deployment (if configured)

## Content Guidelines

- Write in Markdown format
- Use clear, instructional tone appropriate for students with ROS 2 knowledge
- Include minimal code examples to illustrate concepts
- Focus on conceptual understanding rather than hands-on implementation
- Structure content with clear headings and subheadings
- Reference concepts from Module 1 (ROS 2) and Module 2 (Digital Twin) where appropriate

## Troubleshooting

### Common Issues

**Issue:** Content not appearing in sidebar
**Solution:** Verify the file paths in `sidebars.js` match your Markdown files

**Issue:** Images or assets not loading
**Solution:** Place assets in the `static/` directory and reference them with `/img/` paths

**Issue:** Links to previous modules not working
**Solution:** Verify that the ROS 2 and Digital Twin module content exists and paths are correct