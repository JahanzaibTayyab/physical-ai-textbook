/**
 * Chapter buttons component.
 *
 * Combines personalization and translation buttons for chapter pages.
 * This component can be imported in MDX files.
 *
 * Usage in MDX:
 * ```mdx
 * import ChapterButtons from '@site/src/components/ChapterButtons';
 *
 * <ChapterButtons chapterPath="/docs/module-1-ros2/intro" />
 * ```
 */

import PersonalizationButton from "../PersonalizationButton";
import React from "react";
import TranslationButton from "../TranslationButton";

interface ChapterButtonsProps {
  chapterPath: string;
  getContent?: () => string; // Function to get current page content
}

export default function ChapterButtons({
  chapterPath,
  getContent,
}: ChapterButtonsProps): React.JSX.Element {
  // Get content from the page if function provided, otherwise use empty string
  // In MDX, we'll pass the content directly
  const originalContent = getContent ? getContent() : "";

  return (
    <div style={{ marginBottom: "2rem" }}>
      <PersonalizationButton
        chapterPath={chapterPath}
        originalContent={originalContent}
      />
      <TranslationButton
        chapterPath={chapterPath}
        originalContent={originalContent}
      />
    </div>
  );
}
