# Quickstart Guide: Setting up the Digital Twin Educational Module

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Basic command-line knowledge
- Understanding of Module 1 (ROS 2 concepts)

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

### 3. Set up the Digital Twin Module Content
Create the directory structure for the Digital Twin module:
```bash
mkdir -p docs/digital-twin
```

### 4. Add Chapter Files
Create the three main chapters for the Digital Twin module:

**Chapter 1: Gazebo Physics Simulation**
```bash
touch docs/digital-twin/01-gazebo-physics-simulation.md
```

**Chapter 2: Unity-Based Interaction**
```bash
touch docs/digital-twin/02-unity-interaction.md
```

**Chapter 3: Sensor Simulation**
```bash
touch docs/digital-twin/03-sensor-simulation.md
```

### 5. Configure Sidebar Navigation
Update `sidebars.js` to include the Digital Twin module:
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

### Chapter 1: Gazebo Physics Simulation
Create content explaining:
- Digital twin concepts and their importance in robotics
- Gazebo physics simulation fundamentals
- Physics engines (ODE, Bullet, Simbody)
- Collision detection and response
- Dynamics simulation principles

### Chapter 2: Unity-Based Interaction
Create content explaining:
- Unity interface for robotics visualization
- 3D visualization principles
- User interaction systems for robot control
- VR/AR capabilities for immersive simulation

### Chapter 3: Sensor Simulation
Create content explaining:
- Sensor simulation principles (LiDAR, depth cameras, IMUs)
- How different sensors are simulated in digital environments
- Noise modeling and realistic sensor data generation
- Integration with physics engines for realistic simulation

## Development Workflow

1. Create or edit Markdown files in the `docs/digital-twin/` directory
2. Use the development server to preview changes
3. Commit changes to Git
4. Push to GitHub for automatic deployment (if configured)

## Content Guidelines

- Write in Markdown format
- Use clear, instructional tone appropriate for students
- Include minimal code examples to illustrate concepts
- Focus on conceptual understanding rather than hands-on implementation
- Structure content with clear headings and subheadings
- Reference concepts from Module 1 (ROS 2) where appropriate

## Troubleshooting

### Common Issues

**Issue:** Content not appearing in sidebar
**Solution:** Verify the file paths in `sidebars.js` match your Markdown files

**Issue:** Images or assets not loading
**Solution:** Place assets in the `static/` directory and reference them with `/img/` paths

**Issue:** Links to Module 1 content not working
**Solution:** Verify that the ROS 2 module content exists and paths are correct