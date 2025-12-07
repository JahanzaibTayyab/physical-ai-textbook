# GitHub Pages Deployment Instructions

## 🚀 Quick Deploy

Your repository is configured for GitHub Pages deployment at:
**https://github.com/JahanzaibTayyab/physical-ai-textbook**

### Step 1: Enable GitHub Pages (REQUIRED)

**⚠️ CRITICAL**: You **MUST** enable GitHub Pages manually before the workflow can deploy!

The workflow cannot automatically enable Pages due to GitHub's security restrictions. You need to do this manually:

1. Go to your repository settings:

   - **https://github.com/JahanzaibTayyab/physical-ai-textbook/settings/pages**

2. Under **"Build and deployment"** → **"Source"**:

   - Select: **`GitHub Actions`**
   - Click **Save**

3. **Important**: After enabling, wait a few seconds, then push your code again or manually trigger the workflow.

**Why?** GitHub requires repository owner/admin permissions to enable Pages, which GitHub Actions doesn't have by default.

### Step 2: Push to GitHub

```bash
cd /Users/zaib/Panaverse/hackathon-1/physical-ai-textbook

# Add all changes
git add .

# Commit
git commit -m "Complete implementation: All features, buttons, and deployment config"

# Push to main branch
git push origin main
```

### Step 3: Wait for Deployment

1. Go to: https://github.com/JahanzaibTayyab/physical-ai-textbook/actions
2. Wait for the "Deploy to GitHub Pages" workflow to complete (2-3 minutes)
3. Your site will be live at: **https://jahanzaibtayyab.github.io/physical-ai-textbook/**

---

## ✅ Configuration Summary

**Current Settings**:

- **URL**: `https://jahanzaibtayyab.github.io`
- **Base URL**: `/physical-ai-textbook/`
- **Organization**: `JahanzaibTayyab`
- **Project**: `physical-ai-textbook`

**GitHub Actions Workflow**: `.github/workflows/deploy.yml` ✅

---

## 🔍 Verify Deployment

After deployment, check:

1. **Homepage**: https://jahanzaibtayyab.github.io/physical-ai-textbook/
2. **Documentation**: https://jahanzaibtayyab.github.io/physical-ai-textbook/docs/intro
3. **Sign Up**: https://jahanzaibtayyab.github.io/physical-ai-textbook/signup
4. **Sign In**: https://jahanzaibtayyab.github.io/physical-ai-textbook/signin
5. **Chatbot**: Should appear on all pages (bottom-right corner)
6. **Text Selection**: Select any text to see menu

---

## 🐛 Troubleshooting

### Build Fails in GitHub Actions

- Check Actions tab for error details
- Verify Node.js version (should be 20)
- Check for TypeScript errors locally first

### Pages Not Loading

- Wait 5-10 minutes for DNS propagation
- Clear browser cache
- Check GitHub Pages settings (Source should be GitHub Actions)

### 404 Errors

- Verify `baseUrl: "/physical-ai-textbook/"` in `docusaurus.config.ts`
- All internal links should use `/docs/` prefix

---

## 📝 Post-Deployment

After successful deployment:

1. ✅ Test all pages load correctly
2. ✅ Verify chatbot widget appears
3. ✅ Test authentication (signup/signin)
4. ✅ Test personalization buttons
5. ✅ Test translation buttons
6. ✅ Test text selection menu
7. ✅ Update backend API URLs (when backend is deployed)

---

## 🔗 Your Site URLs

- **Main Site**: https://jahanzaibtayyab.github.io/physical-ai-textbook/
- **Documentation**: https://jahanzaibtayyab.github.io/physical-ai-textbook/docs/intro
- **Module 1**: https://jahanzaibtayyab.github.io/physical-ai-textbook/docs/module-1-ros2/intro
- **Sign Up**: https://jahanzaibtayyab.github.io/physical-ai-textbook/signup
- **Sign In**: https://jahanzaibtayyab.github.io/physical-ai-textbook/signin

---

**Ready to deploy!** Just push to GitHub and the workflow will handle everything! 🚀
