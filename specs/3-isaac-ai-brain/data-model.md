# Data Model for Isaac AI Brain Educational Module

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

### IsaacComponent
- **Fields:**
  - id: string (unique identifier for the Isaac component)
  - name: string (name of the Isaac component)
  - type: string (simulation, perception, navigation, training)
  - description: string (what the component does)
  - parameters: object (configuration parameters for the component)
  - useCases: array of strings (scenarios where this component is used)
  - chapterId: string (reference to parent chapter)

## Relationships
- Chapter contains multiple Topics
- Topic contains multiple Examples
- Topic contains multiple Concepts
- Chapter contains multiple IsaacComponents
- Concept may reference other Concepts (relatedConcepts)
- IsaacComponent may reference Concepts (for explanation)

## Validation Rules
- Chapter title must be unique within the module
- Chapter content must be in Markdown format
- Learning objectives must be specific and measurable
- Difficulty level must be one of: beginner, intermediate, advanced
- Duration must be a positive number
- Order values must be sequential within each parent container
- IsaacComponent type must be one of: simulation, perception, navigation, training

## State Transitions
- Draft → Review (when content is ready for review)
- Review → Approved (when content passes educational review)
- Approved → Published (when content is ready for students)

## Specific Entities for Isaac AI Brain Module

### Chapter 1: Isaac Sim & Synthetic Data
- Learning Objectives: Understand Isaac Sim, photorealistic simulation, synthetic data generation
- Topics: Isaac Sim, Photorealistic Rendering, Synthetic Data Pipelines
- IsaacComponents: Isaac Sim Environment, Synthetic Data Generator, Rendering Pipeline

### Chapter 2: Isaac ROS & Visual SLAM
- Learning Objectives: Understand Isaac ROS, VSLAM concepts, sensor fusion
- Topics: Isaac ROS, Hardware-accelerated SLAM, Sensor Fusion
- IsaacComponents: VSLAM Module, Sensor Fusion Pipeline, GPU Acceleration Layer

### Chapter 3: Nav2 for Humanoid Navigation
- Learning Objectives: Understand Nav2 for humanoid robots, path planning, navigation pipelines
- Topics: Humanoid Navigation, Path Planning, Bipedal Navigation
- IsaacComponents: Path Planner, Navigation Pipeline, Gait-aware Navigation