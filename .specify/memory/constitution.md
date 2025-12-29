<!-- Sync Impact Report:
Version change: N/A → 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections added
Removed sections: N/A
Templates requiring updates: N/A (new file)
Follow-up TODOs: None
-->

# Project Constitution

**Project:** AI-Spec–Driven Book with Embedded RAG Chatbot
**Version:** 1.0.0
**Ratification Date:** 2025-12-29
**Last Amended:** 2025-12-29

## Governance

This constitution governs the development of the AI-Spec–Driven Book with Embedded RAG Chatbot project. All contributors must adhere to these principles. Amendments require consensus from project maintainers and must be documented with clear rationale.

### Amendment Procedure
Changes to this constitution must be proposed with a clear rationale, reviewed by project maintainers, and documented with before/after analysis.

### Versioning Policy
- MAJOR: Backward incompatible governance/principle removals or redefinitions
- MINOR: New principle/section added or materially expanded guidance
- PATCH: Clarifications, wording, typo fixes, non-semantic refinements

### Compliance Review
Quarterly reviews ensure continued alignment with project goals and principles.

## Principles

### Technical Accuracy and Spec-Driven Execution
All implementation work MUST follow the established specifications. Code changes without corresponding spec updates are prohibited. Technical decisions MUST be based on accurate, verified information and validated through testing.

### Clear, Developer-Focused Writing
Documentation and code comments MUST be clear, concise, and targeted at developers who will maintain the system. All APIs MUST be documented with examples and clear use cases.

### Fully Reproducible Setup and Deployment
All development, testing, and production environments MUST be reproducible through automated scripts. Deployment processes MUST be documented and version-controlled. Any developer MUST be able to set up a complete working environment with minimal manual steps.

### Modular Architecture
The system MUST be built with clear separation of concerns between book content, backend services, and RAG functionality. Components MUST be independently deployable and testable. Dependencies between modules SHOULD be minimized and clearly defined.

## Standards

### Docusaurus Book
- All book content MUST be in Markdown format
- Navigation MUST be clear and hierarchical
- Content MUST be structured for both reading and API consumption

### GitHub + GitHub Pages Deployment
- All code MUST be version-controlled in GitHub
- Documentation MUST be automatically deployed to GitHub Pages
- Pull requests MUST follow established review processes

### RAG Stack
- OpenAI Agents/ChatKit MUST be used for conversation handling
- FastAPI MUST be used for backend API services
- Neon Postgres MUST be used for metadata storage
- Qdrant Cloud MUST be used for vector storage and retrieval
- Chatbot responses MUST be grounded in book content or user-provided text

## Constraints

### No Placeholder Code
All code MUST be complete and functional. Placeholder code or TODO comments MUST be resolved before merging.

### Runnable, Complete Implementation
All features MUST be fully implemented and tested. Partial implementations are not acceptable in main branch.

### Clean Folder Structure
Project structure MUST be organized logically with clear separation between book, backend, and RAG components.

### Minimal Verbosity
Code and documentation SHOULD be concise and focused. Unnecessary complexity or over-engineering is prohibited.

## Success Criteria

### Book Builds and Deploys Successfully
The Docusaurus book MUST build without errors and deploy to GitHub Pages.

### RAG Chatbot Works End-to-End
The chatbot MUST be able to ingest book content, process queries, and provide accurate responses.

### Accurate, Context-Bounded Answers
The chatbot MUST only provide answers based on the book content or user-provided context.

### Spec-Kit Plus Compliance
All project artifacts MUST comply with Spec-Kit Plus standards and templates.