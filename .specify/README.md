# Spec-Driven Development Structure

This directory follows the [Spec-Kit Plus](https://github.com/panaversity/spec-kit-plus/) methodology for spec-driven development.

## Directory Structure

```
.specify/
├── memory/
│   └── constitution.md          # Project constitution and principles
├── specs/                       # Feature specifications
│   ├── rag-chatbot/
│   ├── authentication/
│   ├── personalization/
│   └── translation/
├── plans/                       # Implementation plans
├── tasks/                       # Task breakdowns
├── phr/                         # Prompt History Records
└── adr/                         # Architecture Decision Records
```

## Commands

Use the following commands (if specifyplus CLI is installed):

- `/sp.spec` - Create a new feature specification
- `/sp.plan` - Create an implementation plan
- `/sp.tasks` - Break down plan into tasks
- `/sp.implement` - Execute implementation
- `/sp.analyze` - Analyze code and provide feedback
- `/sp.clarify` - Clarify requirements

## Workflow

1. **Specification**: Define feature requirements in `specs/[feature-name]/spec.md`
2. **Planning**: Create implementation plan in `specs/[feature-name]/plan.md`
3. **Tasks**: Break down into tasks in `specs/[feature-name]/tasks.md`
4. **Implementation**: Execute tasks following the plan
5. **Documentation**: Update PHR and ADR as needed

## Resources

- [Spec-Kit Plus Repository](https://github.com/panaversity/spec-kit-plus/)
- [Spec-Driven Development Guide](https://github.com/panaversity/spec-kit-plus/blob/main/docs/quickstart.md)
