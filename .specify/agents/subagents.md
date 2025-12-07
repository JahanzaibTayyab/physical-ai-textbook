# Claude Code Subagents

This document defines specialized subagents for the Physical AI & Humanoid Robotics textbook project.

## Subagent 1: Content Creation Subagent

**Name**: `content-creator`

**Purpose**: Specialized for generating educational content for the textbook

**Capabilities**:
- Generate module introductions
- Create lesson content with examples
- Write code examples with explanations
- Generate exercises and assessments
- Ensure content follows Docusaurus format

**Specialization**:
- Focuses on educational content structure
- Maintains consistent tone and style
- Includes practical, working examples
- Follows module learning outcomes

**Usage**:
```
@content-creator Generate an introduction for Module 2: The Digital Twin
```

**Context Provided**:
- Module requirements
- Learning outcomes
- Previous module content
- Target audience level

---

## Subagent 2: Testing Subagent

**Name**: `testing-agent`

**Purpose**: Specialized for creating and running tests

**Capabilities**:
- Generate unit tests for code examples
- Create integration tests for APIs
- Write end-to-end tests for features
- Verify test coverage
- Run test suites

**Specialization**:
- Python testing (pytest)
- FastAPI testing
- React component testing
- Integration test scenarios

**Usage**:
```
@testing-agent Create tests for the RAG chatbot API endpoints
```

**Context Provided**:
- Code to test
- Test requirements
- Expected behavior
- Test framework preferences

---

## Subagent 3: Documentation Subagent

**Name**: `docs-agent`

**Purpose**: Specialized for generating and maintaining documentation

**Capabilities**:
- Generate API documentation
- Create user guides
- Write technical specifications
- Maintain README files
- Generate architecture diagrams (text-based)

**Specialization**:
- OpenAPI/Swagger documentation
- Markdown formatting
- Technical writing
- Code documentation

**Usage**:
```
@docs-agent Generate API documentation for the personalization endpoint
```

**Context Provided**:
- API endpoints
- Request/response models
- Authentication requirements
- Usage examples

---

## How to Use Subagents

1. **Invoke in Claude Code**: Use `@subagent-name` followed by task description
2. **Provide Context**: Give relevant context about the task
3. **Review Output**: Review and refine subagent output
4. **Iterate**: Use subagents iteratively for complex tasks

## Subagent Registry

| Subagent ID | Name | Purpose | Status |
|-------------|------|---------|--------|
| SA-001 | content-creator | Educational content generation | ✅ Defined |
| SA-002 | testing-agent | Test creation and execution | ✅ Defined |
| SA-003 | docs-agent | Documentation generation | ✅ Defined |

## Integration with Project

These subagents are designed to work with:
- Spec-Kit Plus workflow
- Docusaurus content structure
- FastAPI backend
- React frontend
- Python testing frameworks

