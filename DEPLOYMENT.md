# Deployment Guide

This guide covers deploying the Physical AI & Humanoid Robotics textbook to GitHub Pages or Vercel.

---

## 🚀 GitHub Pages Deployment

### Option 1: GitHub Actions (Recommended)

1. **Enable GitHub Pages**:
   - Go to your repository Settings → Pages
   - Source: Deploy from a branch
   - Branch: `gh-pages` (or use GitHub Actions)

2. **The workflow is already configured**:
   - File: `.github/workflows/deploy.yml`
   - Automatically builds and deploys on push to `main`

3. **Update `docusaurus.config.ts`**:
   ```typescript
   url: "https://your-username.github.io",
   baseUrl: "/physical-ai-textbook/", // or "/" if using custom domain
   organizationName: "your-username",
   projectName: "physical-ai-textbook",
   ```

4. **Push to GitHub**:
   ```bash
   git push origin main
   ```

5. **Access your site**:
   - `https://your-username.github.io/physical-ai-textbook/`

### Option 2: Manual Deployment

```bash
# Build the site
pnpm run build

# Deploy to GitHub Pages
GIT_USER=your-username pnpm run deploy
```

---

## ☁️ Vercel Deployment

### Quick Deploy

1. **Connect Repository**:
   - Go to [vercel.com](https://vercel.com)
   - Import your GitHub repository
   - Vercel will auto-detect Docusaurus

2. **Configure Build**:
   - Framework Preset: Docusaurus
   - Build Command: `pnpm run build`
   - Output Directory: `build`
   - Install Command: `pnpm install`

3. **Environment Variables** (if needed):
   - Add any required environment variables
   - Backend API URL (if different from localhost)

4. **Deploy**:
   - Click "Deploy"
   - Your site will be live at `your-project.vercel.app`

---

## 🔧 Backend Deployment

The FastAPI backend needs to be deployed separately.

### Option 1: Railway

1. **Create Railway Project**:
   - Go to [railway.app](https://railway.app)
   - New Project → Deploy from GitHub

2. **Configure**:
   - Root Directory: `backend`
   - Start Command: `uv run uvicorn rag_chatbot_backend.api.main:app --host 0.0.0.0 --port $PORT`

3. **Environment Variables**:
   - `GEMINI_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `NEON_DATABASE_URL`

### Option 2: Render

1. **Create Web Service**:
   - Go to [render.com](https://render.com)
   - New Web Service → Connect GitHub

2. **Configure**:
   - Build Command: `cd backend && uv sync`
   - Start Command: `cd backend && uv run uvicorn rag_chatbot_backend.api.main:app --host 0.0.0.0 --port $PORT`

3. **Environment Variables**: Same as Railway

### Option 3: Fly.io

```bash
cd backend
fly launch
fly secrets set GEMINI_API_KEY=your-key
fly secrets set QDRANT_URL=your-url
fly secrets set QDRANT_API_KEY=your-key
fly secrets set NEON_DATABASE_URL=your-url
fly deploy
```

---

## 🔗 Update Frontend API URL

After deploying the backend, update the frontend to use the production API:

1. **Create environment file**:
   ```bash
   # .env.production
   REACT_APP_API_URL=https://your-backend.railway.app
   ```

2. **Update components**:
   - `src/components/Chatbot/ChatWidget.tsx`
   - `src/components/TextSelectionMenu/index.tsx`
   - `src/components/PersonalizationButton/index.tsx`
   - `src/components/TranslationButton/index.tsx`
   - `src/pages/signup.tsx`
   - `src/pages/signin.tsx`

   Change `http://localhost:8000` to use environment variable:
   ```typescript
   const apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';
   ```

---

## ✅ Pre-Deployment Checklist

- [ ] Update `docusaurus.config.ts` with correct URLs
- [ ] Test build locally: `pnpm run build`
- [ ] Verify all pages render correctly
- [ ] Check chatbot widget appears on all pages
- [ ] Test authentication flow
- [ ] Test personalization and translation
- [ ] Deploy backend and get production URL
- [ ] Update frontend API URLs
- [ ] Test end-to-end in production

---

## 🐛 Troubleshooting

### Build Fails

- Check Node.js version (requires >= 20)
- Clear cache: `pnpm run clear`
- Delete `node_modules` and reinstall

### Pages Not Loading

- Check `baseUrl` in `docusaurus.config.ts`
- Verify GitHub Pages settings
- Check browser console for errors

### Chatbot Not Working

- Verify backend is deployed and accessible
- Check CORS settings in backend
- Verify API URL in frontend

---

**Last Updated**: 2025-01-07

