<!-- SYNC IMPACT REPORT
Version change: N/A → 1.0.0
Added sections: All principles and sections as specified
Removed sections: None (new constitution)
Modified principles: None (new constitution)
Templates requiring updates:
- ✅ .specify/templates/plan-template.md - to be checked
- ✅ .specify/templates/spec-template.md - to be checked
- ✅ .specify/templates/tasks-template.md - to be checked
- ✅ .specify/templates/commands/*.md - to be checked
Follow-up TODOs: None
-->

# AI-Spec–Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Specification-First Development
All features and functionality must be specified before implementation; Specifications serve as contracts that guide development and testing; Changes to behavior require specification updates first

### Technical Accuracy and Documentation
Use official documentation and verified sources; No hallucinated APIs or undocumented features; All technical claims must be verifiable and accurate

### Clarity for Professional Software Engineers
Documentation and code must be clear and understandable to professional developers; Examples should be practical and production-ready; Avoid oversimplification that masks real-world complexity

### Full Reproducibility
All components of the book, deployment, and chatbot must be fully reproducible from source; Include complete setup, build, and deployment instructions; Environment variables for all secrets, no hardcoded credentials

### Disciplined AI-Native Authoring
Use Claude Code and Spec-Kit Plus for AI-assisted development; Maintain human oversight and validation; No hallucinated APIs or features that don't exist

### Open Source and Free-Tier Compliance
Use only open-source or free-tier tools; No proprietary or undocumented APIs; Maintain cost-effective operation on free tiers where possible

## Technology Stack and Architecture
Book framework: Docusaurus; Deployment: GitHub Pages; Backend: FastAPI; LLM: OpenAI Agents/ChatKit SDKs; Vector DB: Qdrant Cloud (Free Tier); Relational DB: Neon Serverless Postgres; Chatbot embedded in Docusaurus site; Single monorepo for book + backend

## Development Workflow and Quality Standards
Specification-driven development using Spec-Kit Plus; Prompt History Records (PHRs) for all user inputs; Architectural Decision Records (ADRs) for significant decisions; Test-first approach where applicable; Code reviews and quality gates; Compliance verification for all PRs

## Governance
Constitution supersedes all other practices; Amendments require documentation and approval; All PRs/reviews must verify compliance with these principles; Complexity must be justified with clear benefits; Use project templates and standards for consistency

**Version**: 1.0.0 | **Ratified**: 2025-12-26 | **Last Amended**: 2025-12-26