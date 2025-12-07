"""
Markdown parsing utilities for intelligent chunking.

Respects markdown structure (headers, paragraphs, code blocks) when chunking.
"""

import re
from typing import List, Tuple


def split_markdown_by_structure(
    content: str, max_chunk_size: int = 800, overlap: int = 150
) -> List[Tuple[str, dict]]:
    """
    Split markdown content respecting structure.
    
    Preserves code blocks intact and respects paragraph boundaries.
    
    Args:
        content: Markdown content to chunk
        max_chunk_size: Maximum characters per chunk
        overlap: Character overlap between chunks
        
    Returns:
        List of tuples (chunk_text, metadata)
    """
    chunks = []
    
    # Split by code blocks first (preserve them intact)
    code_block_pattern = r'(```[\s\S]*?```)'
    parts = re.split(code_block_pattern, content)
    
    current_chunk = ""
    current_metadata = {"has_code": False, "headers": []}
    
    for part in parts:
        if part.strip().startswith("```"):
            # This is a code block - preserve it intact
            if len(current_chunk) + len(part) > max_chunk_size and current_chunk:
                # Save current chunk
                chunks.append((current_chunk.strip(), current_metadata.copy()))
                current_chunk = part
                current_metadata = {"has_code": True, "headers": current_metadata["headers"]}
            else:
                current_chunk += part
                current_metadata["has_code"] = True
        else:
            # Regular text - split by paragraphs
            paragraphs = part.split("\n\n")
            
            for para in paragraphs:
                para = para.strip()
                if not para:
                    continue
                
                # Extract headers for metadata
                header_match = re.match(r'^(#{1,6})\s+(.+)$', para)
                if header_match:
                    level = len(header_match.group(1))
                    header_text = header_match.group(2)
                    current_metadata["headers"].append((level, header_text))
                
                # Check if adding this paragraph would exceed chunk size
                if len(current_chunk) + len(para) + 2 > max_chunk_size and current_chunk:
                    # Save current chunk with overlap
                    if overlap > 0 and len(current_chunk) > overlap:
                        overlap_text = current_chunk[-overlap:]
                        chunks.append((current_chunk.strip(), current_metadata.copy()))
                        current_chunk = overlap_text + "\n\n" + para
                    else:
                        chunks.append((current_chunk.strip(), current_metadata.copy()))
                        current_chunk = para
                else:
                    if current_chunk:
                        current_chunk += "\n\n" + para
                    else:
                        current_chunk = para
    
    # Add final chunk
    if current_chunk.strip():
        chunks.append((current_chunk.strip(), current_metadata))
    
    return chunks


def extract_code_blocks(content: str) -> List[Tuple[str, str]]:
    """
    Extract code blocks from markdown.
    
    Returns:
        List of tuples (language, code)
    """
    pattern = r'```(\w+)?\n([\s\S]*?)```'
    matches = re.findall(pattern, content)
    return [(lang or "text", code) for lang, code in matches]

