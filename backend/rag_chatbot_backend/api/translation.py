"""
Urdu translation API.

Translates chapter content to Urdu while preserving structure.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
import hashlib
import re

from ..database.connection import get_session
from ..database.models import TranslationModel
from google import genai
import os

router = APIRouter(prefix="/api/translate", tags=["translation"])


class TranslateRequest(BaseModel):
    chapter_path: str
    content: str
    target_language: str = "ur"  # Urdu


class TranslateResponse(BaseModel):
    translated_content: str
    cached: bool


@router.post("/", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate chapter content to Urdu (or other languages).
    """
    async for session in get_session():
        try:
            # Check cache
            content_hash = hashlib.sha256(
                f"{request.chapter_path}:{request.content}:{request.target_language}".encode()
            ).hexdigest()
            
            stmt = select(TranslationModel).where(
                TranslationModel.content_hash == content_hash
            )
            result = await session.execute(stmt)
            cached = result.scalar_one_or_none()
            
            if cached:
                return TranslateResponse(
                    translated_content=cached.translated_content,
                    cached=True,
                )
            
            # Extract code blocks to preserve them
            code_blocks = []
            code_pattern = r'```(\w+)?\n(.*?)```'
            
            def replace_code(match):
                lang = match.group(1) or ""
                code = match.group(2)
                placeholder = f"__CODE_BLOCK_{len(code_blocks)}__"
                code_blocks.append((lang, code))
                return placeholder
            
            content_without_code = re.sub(code_pattern, replace_code, request.content, flags=re.DOTALL)
            
            # Generate translation using Gemini
            GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
            if not GEMINI_API_KEY:
                raise HTTPException(status_code=500, detail="GEMINI_API_KEY not configured")
            
            client = genai.Client(api_key=GEMINI_API_KEY)
            
            prompt = f"""Translate the following educational content to Urdu. Preserve all markdown structure, headers, and formatting. Do NOT translate code blocks - keep placeholders as __CODE_BLOCK_N__.

Content:
{content_without_code}

Translate to Urdu while preserving markdown structure:"""
            
            response = client.models.generate_content(
                model="gemini-2.5-flash",
                contents=prompt,
            )
            
            translated = response.text
            
            # Restore code blocks
            for i, (lang, code) in enumerate(code_blocks):
                placeholder = f"__CODE_BLOCK_{i}__"
                code_block = f"```{lang}\n{code}\n```" if lang else f"```\n{code}\n```"
                translated = translated.replace(placeholder, code_block, 1)
            
            # Cache the result
            cached_translation = TranslationModel(
                chapter_path=request.chapter_path,
                language=request.target_language,
                original_content=request.content,
                translated_content=translated,
                content_hash=content_hash,
            )
            session.add(cached_translation)
            await session.commit()
            
            return TranslateResponse(
                translated_content=translated,
                cached=False,
            )
        
        except HTTPException:
            await session.rollback()
            raise
        except Exception as e:
            await session.rollback()
            raise HTTPException(status_code=500, detail=f"Error translating content: {str(e)}")
        finally:
            break

