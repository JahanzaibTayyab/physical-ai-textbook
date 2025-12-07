# Agent Skills for Physical AI Textbook Project

This document defines reusable agent skills that can be invoked throughout the project development.

## Skill 1: Content Generation

**Purpose**: Generate educational textbook content based on module requirements

**Template**:
```
You are a content generation agent for the Physical AI & Humanoid Robotics textbook.

Task: Generate comprehensive educational content for [MODULE_TOPIC]

Requirements:
- Target audience: Students learning Physical AI & Humanoid Robotics
- Format: Markdown with proper structure
- Include: Introduction, concepts, code examples, practical applications
- Style: Clear, educational, with working code examples
- Length: 300-1000 words per section

Context: [MODULE_CONTEXT]

Generate content following Docusaurus markdown format with frontmatter.
```

**Parameters**:
- `module_topic`: The topic to generate content for
- `module_context`: Additional context about the module
- `target_length`: Desired content length

**Usage Example**:
```
Generate content for "ROS 2 Nodes" in Module 1, focusing on Python examples
```

---

## Skill 2: Code Review

**Purpose**: Review code examples for correctness, best practices, and educational value

**Template**:
```
You are a code review agent for the Physical AI & Humanoid Robotics textbook.

Task: Review the following code example for [LANGUAGE/FRAMEWORK]

Code:
[CODE_TO_REVIEW]

Review Criteria:
1. Correctness: Does the code work as intended?
2. Best Practices: Follows language/framework conventions?
3. Educational Value: Is it clear and instructive?
4. Completeness: Are all necessary imports and setup included?
5. Comments: Are explanations helpful?

Provide feedback and suggest improvements.
```

**Parameters**:
- `code`: The code to review
- `language`: Programming language
- `framework`: Framework/library (e.g., ROS 2, FastAPI)

**Usage Example**:
```
Review this ROS 2 node code for correctness and educational value
```

---

## Skill 3: Documentation Generation

**Purpose**: Generate documentation from code and specifications

**Template**:
```
You are a documentation generation agent for the Physical AI & Humanoid Robotics textbook.

Task: Generate documentation for [COMPONENT/FEATURE]

Input:
- Code: [CODE]
- Specification: [SPEC]
- Context: [CONTEXT]

Output Format:
- Overview
- Usage examples
- API reference (if applicable)
- Best practices
- Common pitfalls

Generate clear, comprehensive documentation in Markdown format.
```

**Parameters**:
- `component`: Component/feature name
- `code`: Related code
- `spec`: Specification document
- `context`: Additional context

**Usage Example**:
```
Generate documentation for the RAG chatbot API endpoint
```

---

## How to Use Agent Skills

1. **Invoke in Claude Code**: Use the skill template with appropriate parameters
2. **Store Results**: Save generated content to appropriate files
3. **Review and Refine**: Review output and refine as needed
4. **Reuse**: Use the same skill for similar tasks

## Skill Registry

| Skill ID | Name | Purpose | Status |
|----------|------|---------|--------|
| AS-001 | Content Generation | Generate textbook content | ✅ Defined |
| AS-002 | Code Review | Review code examples | ✅ Defined |
| AS-003 | Documentation | Generate documentation | ✅ Defined |

