/**
 * Script to add PersonalizationButton and TranslationButton to all chapter pages.
 *
 * Usage: node scripts/add-buttons-to-chapters.js
 */

const fs = require("fs");
const path = require("path");

const docsDir = path.join(__dirname, "..", "docs");
const modules = [
  "module-1-ros2",
  "module-2-simulation",
  "module-3-isaac",
  "module-4-vla",
];

const buttonImports = `import PersonalizationButton from '@site/src/components/PersonalizationButton';
import TranslationButton from '@site/src/components/TranslationButton';

`;

const buttonComponent = (
  chapterPath,
  contentSummary
) => `<PersonalizationButton 
  chapterPath="${chapterPath}"
  originalContent="${contentSummary}"
/>

<TranslationButton 
  chapterPath="${chapterPath}"
  originalContent="${contentSummary}"
/>

`;

function getContentSummary(filePath) {
  const content = fs.readFileSync(filePath, "utf-8");
  // Extract first paragraph or heading
  const lines = content.split("\n");
  let summary = "";
  for (const line of lines) {
    if (
      line.trim() &&
      !line.startsWith("#") &&
      !line.startsWith("---") &&
      !line.startsWith("import")
    ) {
      summary = line.substring(0, 200).replace(/"/g, '\\"');
      break;
    }
  }
  return summary || "Chapter content";
}

function addButtonsToFile(filePath) {
  const content = fs.readFileSync(filePath, "utf-8");

  // Skip if already has buttons
  if (content.includes("PersonalizationButton")) {
    console.log(`⏭️  Skipping ${filePath} (already has buttons)`);
    return;
  }

  // Extract frontmatter
  const frontmatterMatch = content.match(/^---\n([\s\S]*?)\n---/);
  if (!frontmatterMatch) {
    console.log(`⚠️  No frontmatter found in ${filePath}`);
    return;
  }

  const frontmatter = frontmatterMatch[0];
  const restOfContent = content.substring(frontmatterMatch[0].length).trim();

  // Get chapter path
  const relativePath = path.relative(
    path.join(__dirname, "..", "docs"),
    filePath
  );
  const chapterPath = `/docs/${relativePath
    .replace(/\\/g, "/")
    .replace(".md", "")}`;

  // Get content summary
  const contentSummary = getContentSummary(filePath);

  // Build new content
  const newContent = `${frontmatter}

${buttonImports}${buttonComponent(
    chapterPath,
    contentSummary
  )}${restOfContent}`;

  fs.writeFileSync(filePath, newContent, "utf-8");
  console.log(`✅ Added buttons to ${filePath}`);
}

// Process all module directories
modules.forEach((module) => {
  const moduleDir = path.join(docsDir, module);
  if (!fs.existsSync(moduleDir)) {
    console.log(`⚠️  Module directory not found: ${moduleDir}`);
    return;
  }

  const files = fs.readdirSync(moduleDir);
  files.forEach((file) => {
    if (file.endsWith(".md")) {
      const filePath = path.join(moduleDir, file);
      addButtonsToFile(filePath);
    }
  });
});

console.log("\n✨ Done! All chapters have been updated.");
