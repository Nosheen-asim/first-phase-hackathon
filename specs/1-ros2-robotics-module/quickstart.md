# Quickstart Guide: Setting up the ROS 2 Educational Book

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Basic command-line knowledge

## Installation Steps

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Initialize Docusaurus Project
If starting fresh, create a new Docusaurus project:
```bash
npx create-docusaurus@latest my-website classic
```

### 4. Set up the ROS 2 Module Content
Create the directory structure for the ROS 2 module:
```bash
mkdir -p docs/ros2-module
```

### 5. Add Chapter Files
Create the three main chapters for the ROS 2 module:

**Chapter 1: ROS 2 Architecture**
```bash
touch docs/ros2-module/01-ros2-architecture.md
```

**Chapter 2: Python Agents with rclpy**
```bash
touch docs/ros2-module/02-python-agents-rclpy.md
```

**Chapter 3: URDF for Humanoids**
```bash
touch docs/ros2-module/03-urdf-humanoids.md
```

### 6. Configure Sidebar Navigation
Update `sidebars.js` to include the ROS 2 module:
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
  ],
};
```

### 7. Run the Development Server
```bash
npm run start
# or
yarn start
```

The development server will start at `http://localhost:3000`.

### 8. Build for Production
```bash
npm run build
# or
yarn build
```

## Adding Content to Chapters

### Chapter 1: ROS 2 Architecture
Create content explaining:
- Purpose of ROS 2 as robotic middleware
- Nodes, Topics, Services, and communication model
- How these components interact

### Chapter 2: Python Agents with rclpy
Create content explaining:
- Creating ROS 2 nodes using Python
- Bridging AI logic to robot controllers via rclpy
- Practical examples with minimal code

### Chapter 3: URDF for Humanoids
Create content explaining:
- Understanding URDF fundamentals
- Modeling humanoid joints, links, and sensors
- Practical examples with minimal code

## Deployment

### GitHub Pages Deployment
1. Configure `docusaurus.config.js` with your GitHub repository details
2. Run the deployment command:
```bash
npm run deploy
# or
yarn deploy
```

### Manual Deployment
Build the static files and serve them with any static hosting service:
```bash
npm run build
# The static files will be in the build/ directory
```

## Development Workflow

1. Create or edit Markdown files in the `docs/` directory
2. Use the development server to preview changes
3. Commit changes to Git
4. Push to GitHub for automatic deployment (if configured)

## Content Guidelines

- Write in Markdown format
- Use clear, instructional tone appropriate for students
- Include minimal code examples to illustrate concepts
- Focus on conceptual understanding rather than hands-on implementation
- Structure content with clear headings and subheadings

## Troubleshooting

### Common Issues

**Issue:** Development server won't start
**Solution:** Ensure Node.js and npm are properly installed and in your PATH

**Issue:** Content not appearing in sidebar
**Solution:** Verify the file paths in `sidebars.js` match your Markdown files

**Issue:** Images or assets not loading
**Solution:** Place assets in the `static/` directory and reference them with `/img/` paths