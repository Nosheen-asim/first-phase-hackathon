# Data Model for Digital Twin Educational Module

## Content Structure

### Chapter Entity
- **Fields:**
  - id: string (unique identifier for the chapter)
  - title: string (chapter title)
  - description: string (brief description of the chapter)
  - content: string (main content in Markdown format)
  - learningObjectives: array of strings (what students will learn)
  - prerequisites: array of strings (required knowledge)
  - duration: number (estimated time to complete in minutes)
  - difficulty: string (beginner, intermediate, advanced)
  - order: number (sequence in the module)
  - relatedChapters: array of strings (IDs of related chapters in other modules)

### Topic Entity
- **Fields:**
  - id: string (unique identifier for the topic)
  - title: string (topic title)
  - content: string (topic content in Markdown)
  - chapterId: string (reference to parent chapter)
  - learningObjectives: array of strings
  - examples: array of Example entities
  - order: number (sequence within the chapter)

### Example Entity
- **Fields:**
  - id: string (unique identifier for the example)
  - title: string (example title)
  - description: string (what the example demonstrates)
  - code: string (minimal code example in Markdown)
  - explanation: string (explanation of the example)
  - topicId: string (reference to parent topic)

### Concept Entity
- **Fields:**
  - id: string (unique identifier for the concept)
  - name: string (name of the concept)
  - definition: string (clear definition of the concept)
  - explanation: string (detailed explanation)
  - relatedConcepts: array of strings (IDs of related concepts)
  - examples: array of Example entities
  - chapterId: string (reference to parent chapter)

### SimulationEntity
- **Fields:**
  - id: string (unique identifier for the simulation component)
  - name: string (name of the simulation component)
  - type: string (physics, sensor, interaction, visualization)
  - description: string (what the component does)
  - parameters: object (configuration parameters for the simulation)
  - useCases: array of strings (scenarios where this component is used)
  - chapterId: string (reference to parent chapter)

## Relationships
- Chapter contains multiple Topics
- Topic contains multiple Examples
- Topic contains multiple Concepts
- Chapter contains multiple SimulationEntities
- Concept may reference other Concepts (relatedConcepts)
- SimulationEntity may reference Concepts (for explanation)

## Validation Rules
- Chapter title must be unique within the module
- Chapter content must be in Markdown format
- Learning objectives must be specific and measurable
- Difficulty level must be one of: beginner, intermediate, advanced
- Duration must be a positive number
- Order values must be sequential within each parent container
- SimulationEntity type must be one of: physics, sensor, interaction, visualization

## State Transitions
- Draft → Review (when content is ready for review)
- Review → Approved (when content passes educational review)
- Approved → Published (when content is ready for students)

## Specific Entities for Digital Twin Module

### Chapter 1: Gazebo Physics Simulation
- Learning Objectives: Understand physics simulation principles, configure Gazebo environments, simulate robot dynamics
- Topics: Physics Engines, Collision Detection, Dynamics Simulation, Gazebo Integration
- SimulationEntities: Physics Engine (ODE/Bullet), Collision Models, Joint Dynamics, Contact Simulation

### Chapter 2: Unity-Based Interaction
- Learning Objectives: Create interactive 3D environments, implement user controls, visualize robot data
- Topics: Unity Interface, 3D Visualization, User Interaction, VR/AR Integration
- SimulationEntities: 3D Models, Camera Systems, Interaction Controllers, Rendering Pipelines

### Chapter 3: Sensor Simulation
- Learning Objectives: Simulate LiDAR, depth cameras, and IMUs, understand sensor noise models, validate perception algorithms
- Topics: LiDAR Simulation, Depth Camera Simulation, IMU Simulation, Noise Modeling
- SimulationEntities: LiDAR Sensors, Depth Cameras, IMU Sensors, Noise Generators