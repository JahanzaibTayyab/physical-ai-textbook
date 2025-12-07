/**
 * Chapter buttons component.
 * 
 * Combines personalization and translation buttons for chapter pages.
 * This component can be imported in MDX files.
 */

import React from "react";
import PersonalizationButton from "../PersonalizationButton";
import TranslationButton from "../TranslationButton";

interface ChapterButtonsProps {
  chapterPath: string;
  originalContent: string;
}

export default function ChapterButtons({
  chapterPath,
  originalContent,
}: ChapterButtonsProps): React.JSX.Element {
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

