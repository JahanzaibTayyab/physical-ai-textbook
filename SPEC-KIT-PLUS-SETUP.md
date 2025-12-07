# Spec-Kit Plus Integration Guide

This project uses [Spec-Kit Plus](https://github.com/panaversity/spec-kit-plus/) for spec-driven development, following the methodology outlined in the repository.

## What is Spec-Kit Plus?

Spec-Kit Plus is a practical fork of GitHub's spec-kit with patterns & templates for building scalable multi-agent AI systems. It enables **Spec-Driven Development** where:

- **Specifications** are treated as first-class artifacts
- **Architecture history** is preserved
- **Prompt history** is tracked
- **Tests** and **evaluations** are automated

## Project Structure

```
physical-ai-textbook/
├── .specify/                    # Spec-driven development files
│   ├── memory/
│   │   └── constitution.md     # Project constitution
│   ├── specs/                  # Feature specifications
│   ├── plans/                  # Implementation plans
│   ├── tasks/                  # Task breakdowns
│   ├── phr/                    # Prompt History Records
│   └── adr/                    # Architecture Decision Records
├── docs/                       # Course content (Docusaurus)
├── src/                        # React components
└── static/                      # Static assets
```

## Using Spec-Kit Plus Commands

### Installation

```bash
# Install specifyplus CLI using UV (recommended)
uv tool install specifyplus

# Or using pip
pip install specifyplus
```

### Available Commands

#### 1. Create a Specification (`/sp.spec`)

Create a new feature specification:

```bash
sp.spec "RAG Chatbot Integration"
```

This creates:
- `.specify/specs/rag-chatbot/spec.md` - Feature specification
- Follows the spec template with user stories, requirements, success criteria

#### 2. Create Implementation Plan (`/sp.plan`)

Generate an implementation plan from a spec:

```bash
sp.plan rag-chatbot
```

This creates:
- `.specify/specs/rag-chatbot/plan.md` - Implementation plan
- `.specify/specs/rag-chatbot/research.md` - Technical research
- `.specify/specs/rag-chatbot/data-model.md` - Data models
- `.specify/specs/rag-chatbot/quickstart.md` - Quick start guide

#### 3. Create Tasks (`/sp.tasks`)

Break down plan into executable tasks:

```bash
sp.tasks rag-chatbot
```

This creates:
- `.specify/specs/rag-chatbot/tasks.md` - Task breakdown

#### 4. Implement (`/sp.implement`)

Execute the implementation plan:

```bash
sp.implement rag-chatbot
```

This will:
- Execute tasks in order
- Follow TDD approach
- Create checkpoints for validation

#### 5. Other Commands

- `/sp.analyze` - Analyze code and provide feedback
- `/sp.clarify` - Clarify requirements
- `/sp.adr` - Create Architecture Decision Record
- `/sp.phr` - Create Prompt History Record

## Workflow for This Project

### Step 1: Create Specification for RAG Chatbot

```bash
cd physical-ai-textbook
sp.spec "RAG Chatbot for Textbook Q&A"
```

### Step 2: Create Implementation Plan

```bash
sp.plan rag-chatbot
```

### Step 3: Break Down into Tasks

```bash
sp.tasks rag-chatbot
```

### Step 4: Implement

```bash
sp.implement rag-chatbot
```

## Features to Implement Using Spec-Kit Plus

1. **RAG Chatbot** (Core Requirement)
   - FastAPI backend
   - Qdrant vector database
   - OpenAI integration
   - Text selection support

2. **Authentication** (Bonus Feature)
   - Better Auth integration
   - User background collection
   - Profile management

3. **Content Personalization** (Bonus Feature)
   - User preference storage
   - Dynamic content rendering
   - Background-based customization

4. **Urdu Translation** (Bonus Feature)
   - Translation API integration
   - Content localization
   - Language switching

## Constitution

The project constitution is defined in `.specify/memory/constitution.md`. It establishes:

- Core principles (Spec-Driven, AI-Native, Test-First)
- Technology stack constraints
- Development workflow
- Quality gates

All development must comply with the constitution.

## Resources

- [Spec-Kit Plus Repository](https://github.com/panaversity/spec-kit-plus/)
- [Spec-Driven Development Guide](https://github.com/panaversity/spec-kit-plus/blob/main/docs/quickstart.md)
- [Constitution Template](https://github.com/panaversity/spec-kit-plus/blob/main/memory/constitution.md)
- [Spec Template](https://github.com/panaversity/spec-kit-plus/blob/main/templates/spec-template.md)

## Next Steps

1. Install `specifyplus` CLI using UV:
   ```bash
   uv tool install specifyplus
   ```

2. Create specifications for each feature
3. Generate implementation plans
4. Execute implementations following the spec-driven workflow

## Python Package Management with UV

This project uses **UV** as the Python package manager. When working with Python dependencies:

```bash
# Add a new package
uv add fastapi
uv add openai

# Add dev dependencies
uv add --dev pytest
uv add --dev black

# Install tools globally
uv tool install specifyplus
uv tool install ruff

# Sync dependencies
uv sync

# Run commands in the project environment
uv run python main.py
uv run pytest
```

For more information, see: https://github.com/astral-sh/uv

