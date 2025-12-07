# Physical AI & Humanoid Robotics Textbook

A comprehensive textbook for teaching Physical AI & Humanoid Robotics course, built with Docusaurus and integrated with a RAG chatbot.

## 🚀 Quick Start

### Prerequisites

- Node.js 20+ and pnpm
- Python 3.11+ and UV
- Git

### Installation

1. **Clone the repository**
   ```bash
   git clone <your-repo-url>
   cd physical-ai-textbook
   ```

2. **Install Node.js dependencies**
   ```bash
   pnpm install
   ```

3. **Install Python dependencies** (when backend is added)
   ```bash
   uv sync
   ```

4. **Start development server**
   ```bash
   pnpm start
   ```

5. **Build for production**
   ```bash
   pnpm build
   ```

## 📦 Package Management

### Frontend (Node.js)
- **Package Manager**: pnpm
- Install: `pnpm add <package>`
- Install dev: `pnpm add -D <package>`

### Backend (Python)
- **Package Manager**: UV (required)
- Install: `uv add <package>`
- Install dev: `uv add --dev <package>`
- Install tools: `uv tool install <tool>`

## 🛠️ Development Tools

### Spec-Kit Plus

This project uses [Spec-Kit Plus](https://github.com/panaversity/spec-kit-plus/) for spec-driven development.

**Install specifyplus CLI:**
```bash
uv tool install specifyplus
```

**Create a specification:**
```bash
sp.spec "Feature Name"
```

See [SPEC-KIT-PLUS-SETUP.md](./SPEC-KIT-PLUS-SETUP.md) for detailed usage.

### Claude Code

Use Claude Code for AI-assisted development following the spec-driven workflow.

## 📚 Project Structure

```
physical-ai-textbook/
├── .specify/              # Spec-driven development files
│   ├── memory/            # Constitution and rules
│   ├── specs/             # Feature specifications
│   ├── plans/             # Implementation plans
│   ├── tasks/             # Task breakdowns
│   ├── phr/               # Prompt History Records
│   └── adr/               # Architecture Decision Records
├── docs/                  # Course content
│   ├── intro.md
│   ├── module-1-ros2/
│   ├── module-2-simulation/
│   ├── module-3-isaac/
│   └── module-4-vla/
├── src/                   # React components
├── static/                # Static assets
└── docusaurus.config.ts   # Docusaurus configuration
```

## 🎯 Course Modules

1. **Module 1: The Robotic Nervous System (ROS 2)**
   - ROS 2 architecture and core concepts
   - Nodes, Topics, and Services
   - Python Agents to ROS controllers

2. **Module 2: The Digital Twin (Gazebo & Unity)**
   - Physics simulation
   - Sensor simulation
   - High-fidelity rendering

3. **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
   - Isaac Sim and Isaac ROS
   - VSLAM and navigation
   - Reinforcement learning

4. **Module 4: Vision-Language-Action (VLA)**
   - Voice-to-Action with Whisper
   - Cognitive Planning with LLMs
   - Capstone Project

## 🚀 Deployment

### GitHub Pages

```bash
pnpm deploy
```

### Vercel

Connect your GitHub repository to Vercel for automatic deployments.

## 📖 Documentation

- [Requirements](./REQUIREMENTS.md) - Project requirements
- [Spec-Kit Plus Setup](./SPEC-KIT-PLUS-SETUP.md) - Spec-driven development guide
- [Constitution](./.specify/memory/constitution.md) - Project principles and rules

## 🤝 Contributing

This project follows spec-driven development methodology. See [SPEC-KIT-PLUS-SETUP.md](./SPEC-KIT-PLUS-SETUP.md) for details.

## 📄 License

MIT License

## 🔗 Resources

- [Panaversity](https://panaversity.org)
- [Spec-Kit Plus](https://github.com/panaversity/spec-kit-plus/)
- [Claude Code](https://www.claude.com/product/claude-code)
- [Better Auth](https://www.better-auth.com/)
