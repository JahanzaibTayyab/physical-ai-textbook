# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Spec-Driven Development
Every feature must start with a clear specification before implementation. Specifications are treated as first-class artifacts and must include:
- User scenarios with acceptance criteria
- Functional requirements
- Success criteria
- Edge cases

### II. AI-Native Development
Leverage AI tools (Claude Code, Spec-Kit Plus) throughout the development lifecycle:
- Use spec-driven templates for all features
- Document all AI interactions in PHR (Prompt History Records)
- Create reusable Agent Skills and Subagents where applicable

### III. Test-First Development
- Write tests before implementation
- All features must have acceptance criteria that are testable
- Integration tests required for RAG chatbot, authentication, and API endpoints

### IV. Documentation as Code
- All course content in Markdown/MDX
- Code examples must be executable and tested
- API documentation must be up-to-date

### V. User-Centric Design
- Personalization based on user background (software/hardware experience)
- Multi-language support (English/Urdu)
- Accessible and responsive design

## Technology Stack Constraints

### Frontend
- **Framework**: Docusaurus (required)
- **Language**: TypeScript/JavaScript
- **Package Manager**: pnpm
- **Styling**: CSS Modules, custom CSS

### Backend
- **Framework**: FastAPI (Python)
- **Package Manager**: UV (required)
- **Database**: Neon Serverless Postgres
- **Vector DB**: Qdrant Cloud Free Tier
- **AI Services**: Google Gemini API (for chatbot, embeddings, and agents)
- **OpenAI Whisper** (for voice-to-action in Module 4 - VLA, if needed)

### Authentication
- **Provider**: Better Auth

### Deployment
- **Platform**: GitHub Pages or Vercel
- **CI/CD**: GitHub Actions

## Development Workflow

1. **Specification Phase**: Create spec using `/sp.spec` command
2. **Planning Phase**: Create implementation plan using `/sp.plan` command
3. **Task Breakdown**: Create tasks using `/sp.tasks` command
4. **Implementation**: Execute using `/sp.implement` command
5. **Documentation**: Update PHR and ADR as needed

## Package Management

### Python Packages
- **Always use UV** for Python package management
- Install packages: `uv add <package-name>`
- Install dev dependencies: `uv add --dev <package-name>`
- Install tools: `uv tool install <tool-name>`

### Node.js Packages
- **Always use pnpm** for Node.js package management
- Install packages: `pnpm add <package-name>`
- Install dev dependencies: `pnpm add -D <package-name>`

## Quality Gates

- All code must pass linting
- All tests must pass before merge
- Documentation must be updated
- PHR must be maintained for AI interactions
- ADR required for architectural decisions

## Governance

This constitution supersedes all other practices. Amendments require:
- Documentation of the change
- Approval from project maintainers
- Migration plan if breaking changes

**Version**: 1.0.0 | **Ratified**: 2025-01-XX | **Last Amended**: 2025-01-XX
