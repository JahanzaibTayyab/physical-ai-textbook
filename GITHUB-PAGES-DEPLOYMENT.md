# GitHub Pages Deployment Guide

## 🚀 Quick Deployment Steps

### 1. Update Configuration

The `docusaurus.config.ts` has been configured with:
- `url: "https://zaib.github.io"`
- `baseUrl: "/hackathon-1/"`
- `organizationName: "zaib"`
- `projectName: "hackathon-1"`

**If your GitHub username/repo is different**, update these values in `docusaurus.config.ts`.

### 2. Enable GitHub Pages

1. Go to your GitHub repository: `https://github.com/zaib/hackathon-1`
2. Navigate to **Settings** → **Pages**
3. Under **Source**, select:
   - **Source**: `GitHub Actions`
4. Save the settings

### 3. Push to GitHub

```bash
cd /Users/zaib/Panaverse/hackathon-1/physical-ai-textbook

# Add all files
git add .

# Commit changes
git commit -m "Add all features and prepare for deployment"

# Push to main branch
git push origin main
```

### 4. Automatic Deployment

Once you push to `main`, the GitHub Actions workflow will:
1. ✅ Build the Docusaurus site
2. ✅ Deploy to GitHub Pages
3. ✅ Make it available at: `https://zaib.github.io/hackathon-1/`

### 5. Verify Deployment

- Check Actions tab: `https://github.com/zaib/hackathon-1/actions`
- Wait for workflow to complete (usually 2-3 minutes)
- Visit your site: `https://zaib.github.io/hackathon-1/`

---

## 🔧 Manual Deployment (Alternative)

If you prefer manual deployment:

```bash
cd physical-ai-textbook

# Build the site
pnpm run build

# Deploy (requires GIT_USER environment variable)
GIT_USER=zaib pnpm run deploy
```

---

## ⚙️ Configuration Details

### Base URL

The `baseUrl` is set to `/hackathon-1/` because the repository is `hackathon-1`.

**If your repo name is different**, update:
- `baseUrl` in `docusaurus.config.ts`
- `projectName` in `docusaurus.config.ts`

### Custom Domain (Optional)

If you want to use a custom domain:

1. Update `baseUrl: "/"` in `docusaurus.config.ts`
2. Add `CNAME` file in `static/` folder with your domain
3. Configure DNS settings

---

## 🐛 Troubleshooting

### Build Fails

- Check Node.js version (requires >= 20)
- Verify all dependencies are installed
- Check for TypeScript errors: `pnpm run typecheck`

### Pages Not Loading

- Verify `baseUrl` matches your repository structure
- Check GitHub Pages settings (Source should be GitHub Actions)
- Wait a few minutes for DNS propagation

### 404 Errors

- Ensure `baseUrl` is correct
- Check that files are in `build/` directory
- Verify GitHub Pages is enabled

### Chatbot Not Working

- Backend needs to be deployed separately
- Update API URLs in frontend components
- Check CORS settings in backend

---

## 📝 Post-Deployment Checklist

- [ ] Site is accessible at GitHub Pages URL
- [ ] All pages load correctly
- [ ] Navigation works
- [ ] Chatbot widget appears
- [ ] Authentication pages work
- [ ] Personalization buttons visible
- [ ] Translation buttons visible
- [ ] Text selection menu works

---

## 🔗 Important URLs

After deployment, your site will be available at:
- **Main Site**: `https://zaib.github.io/hackathon-1/`
- **Documentation**: `https://zaib.github.io/hackathon-1/docs/intro`
- **Sign Up**: `https://zaib.github.io/hackathon-1/signup`
- **Sign In**: `https://zaib.github.io/hackathon-1/signin`

---

## 🎯 Next Steps

1. **Deploy Backend**: Deploy FastAPI backend to Railway/Render/Fly.io
2. **Update API URLs**: Change `http://localhost:8000` to production URL
3. **Test Everything**: Verify all features work in production
4. **Create Demo Video**: Record < 90 seconds demo
5. **Submit**: Fill out hackathon submission form

---

**Ready to Deploy!** 🚀

Just push to GitHub and the workflow will handle the rest.

