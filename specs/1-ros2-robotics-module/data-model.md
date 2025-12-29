# Data Model for ROS 2 Educational Module

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

## Relationships
- Chapter contains multiple Topics
- Topic contains multiple Examples
- Topic contains multiple Concepts
- Concept may reference other Concepts (relatedConcepts)

## Validation Rules
- Chapter title must be unique within the module
- Chapter content must be in Markdown format
- Learning objectives must be specific and measurable
- Difficulty level must be one of: beginner, intermediate, advanced
- Duration must be a positive number
- Order values must be sequential within each parent container

## State Transitions
- Draft → Review (when content is ready for review)
- Review → Approved (when content passes educational review)
- Approved → Published (when content is ready for students)

## Specific Entities for ROS 2 Module

### Chapter 1: ROS 2 Architecture
- Learning Objectives: Understand ROS 2 as middleware, explain nodes/topics/services, describe communication model
- Topics: ROS 2 Purpose, Nodes, Topics, Services, Communication Model

### Chapter 2: Python Agents with rclpy
- Learning Objectives: Create ROS 2 nodes in Python, bridge AI logic to controllers, use rclpy effectively
- Topics: rclpy Introduction, Node Creation, Publisher/Subscriber Pattern, AI Integration

### Chapter 3: URDF for Humanoids
- Learning Objectives: Explain URDF fundamentals, model joints/links/sensors, understand humanoid structure
- Topics: URDF Basics, Joints and Links, Sensors Definition, Humanoid Modeling