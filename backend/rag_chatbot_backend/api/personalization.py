"""
Content personalization API.

Generates personalized content based on user background.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from datetime import datetime
import hashlib

from ..database.connection import get_session
from ..database.models import UserProfileModel, PersonalizedContentModel
from ..services.gemini_model_provider import get_gemini_provider
from google import genai
import os

router = APIRouter(prefix="/api/personalize", tags=["personalization"])


class PersonalizeRequest(BaseModel):
    user_id: str
    chapter_path: str
    original_content: str
    background_type: str  # "software" or "hardware"


class PersonalizeResponse(BaseModel):
    personalized_content: str
    cached: bool


@router.post("/", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """
    Personalize chapter content based on user background.
    """
    async for session in get_session():
        try:
            # Get user profile
            stmt = select(UserProfileModel).where(UserProfileModel.user_id == int(request.user_id))
            result = await session.execute(stmt)
            profile = result.scalar_one_or_none()
            
            if not profile:
                raise HTTPException(status_code=404, detail="User profile not found")
            
            # Get background information
            background_text = ""
            if request.background_type == "software":
                background_text = profile.software_background or ""
            elif request.background_type == "hardware":
                background_text = profile.hardware_background or ""
            
            if not background_text:
                raise HTTPException(status_code=400, detail=f"No {request.background_type} background information available")
            
            # Check cache
            content_hash = hashlib.sha256(
                f"{request.chapter_path}:{request.original_content}:{background_text}".encode()
            ).hexdigest()
            
            stmt = select(PersonalizedContentModel).where(
                PersonalizedContentModel.content_hash == content_hash
            )
            result = await session.execute(stmt)
            cached = result.scalar_one_or_none()
            
            if cached:
                return PersonalizeResponse(
                    personalized_content=cached.personalized_content,
                    cached=True,
                )
            
            # Generate personalized content using Gemini
            GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
            if not GEMINI_API_KEY:
                raise HTTPException(status_code=500, detail="GEMINI_API_KEY not configured")
            
            client = genai.Client(api_key=GEMINI_API_KEY)
            
            prompt = f"""Personalize the following educational content for a student with {request.background_type} background: {background_text}

Original content:
{request.original_content}

Instructions:
- Adapt the content to emphasize aspects relevant to {request.background_type} background
- Keep all code examples unchanged
- Preserve markdown structure
- Maintain educational value
- Make it more relatable to someone with {request.background_type} experience

Generate personalized content:"""
            
            model = client.models.generate_content(
                model="gemini-2.5-flash",
                contents=prompt,
            )
            
            personalized_content = model.text
            
            # Cache the result
            cached_content = PersonalizedContentModel(
                user_id=int(request.user_id),
                chapter_path=request.chapter_path,
                original_content=request.original_content,
                personalized_content=personalized_content,
                background_type=request.background_type,
                content_hash=content_hash,
            )
            session.add(cached_content)
            await session.commit()
            
            return PersonalizeResponse(
                personalized_content=personalized_content,
                cached=False,
            )
        
        except HTTPException:
            await session.rollback()
            raise
        except Exception as e:
            await session.rollback()
            raise HTTPException(status_code=500, detail=f"Error personalizing content: {str(e)}")
        finally:
            break

