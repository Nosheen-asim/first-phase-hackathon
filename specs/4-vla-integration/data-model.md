# Data Model for Vision-Language-Action Educational Module

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

### VLAComponent
- **Fields:**
  - id: string (unique identifier for the VLA component)
  - name: string (name of the VLA component)
  - type: string (voice-processing, language-modeling, action-planning, ros-execution)
  - description: string (what the component does)
  - parameters: object (configuration parameters for the component)
  - useCases: array of strings (scenarios where this component is used)
  - chapterId: string (reference to parent chapter)

## Relationships
- Chapter contains multiple Topics
- Topic contains multiple Examples
- Topic contains multiple Concepts
- Chapter contains multiple VLAComponents
- Concept may reference other Concepts (relatedConcepts)
- VLAComponent may reference Concepts (for explanation)

## Validation Rules
- Chapter title must be unique within the module
- Chapter content must be in Markdown format
- Learning objectives must be specific and measurable
- Difficulty level must be one of: beginner, intermediate, advanced
- Duration must be a positive number
- Order values must be sequential within each parent container
- VLAComponent type must be one of: voice-processing, language-modeling, action-planning, ros-execution

## State Transitions
- Draft → Review (when content is ready for review)
- Review → Approved (when content passes educational review)
- Approved → Published (when content is ready for students)

## Specific Entities for Vision-Language-Action Module

### Chapter 1: Voice-to-Action Pipelines
- Learning Objectives: Understand voice processing, OpenAI Whisper integration, intent extraction
- Topics: Speech Recognition, Intent Processing, Voice Command Mapping
- VLAComponents: Whisper Integration, Intent Classifier, Command Parser

### Chapter 2: LLM-Based Cognitive Planning
- Learning Objectives: Understand LLM integration, task decomposition, action planning
- Topics: Natural Language Understanding, Plan Generation, Workflow Reasoning
- VLAComponents: LLM Interface, Task Decomposer, Plan Generator

### Chapter 3: VLA Execution with ROS 2
- Learning Objectives: Understand ROS 2 integration, action mapping, system coordination
- Topics: ROS 2 Service Mapping, Action Coordination, Feedback Integration
- VLAComponents: ROS 2 Bridge, Action Executor, Feedback Handler