"""
Authentication API routes.

Custom authentication system compatible with Better Auth patterns.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, EmailStr
from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
import bcrypt
import secrets
from datetime import datetime, timedelta

from ..database.connection import get_session
from ..database.models import UserModel, UserProfileModel

router = APIRouter(prefix="/api/auth", tags=["auth"])


# Request/Response models
class SignupRequest(BaseModel):
    email: EmailStr
    password: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class SigninRequest(BaseModel):
    email: EmailStr
    password: str


class AuthResponse(BaseModel):
    user_id: str
    email: str
    session_token: str
    expires_at: datetime


class UserProfileResponse(BaseModel):
    user_id: str
    email: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


@router.post("/signup", response_model=AuthResponse)
async def signup(request: SignupRequest):
    """Sign up a new user with background information."""
    async for session in get_session():
        try:
            # Check if user already exists
            stmt = select(UserModel).where(UserModel.email == request.email)
            result = await session.execute(stmt)
            existing_user = result.scalar_one_or_none()
            
            if existing_user:
                raise HTTPException(status_code=400, detail="User with this email already exists")
            
            # Hash password
            password_hash = bcrypt.hashpw(request.password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')
            
            # Create user
            user = UserModel(
                email=request.email,
                password_hash=password_hash,
            )
            session.add(user)
            await session.flush()
            await session.refresh(user)
            
            # Create user profile with background information
            profile = UserProfileModel(
                user_id=user.id,
                software_background=request.software_background,
                hardware_background=request.hardware_background,
            )
            session.add(profile)
            
            # Generate session token
            session_token = secrets.token_urlsafe(32)
            expires_at = datetime.utcnow() + timedelta(days=7)
            
            await session.commit()
            
            return AuthResponse(
                user_id=str(user.id),
                email=user.email,
                session_token=session_token,
                expires_at=expires_at,
            )
        
        except HTTPException:
            await session.rollback()
            raise
        except Exception as e:
            await session.rollback()
            raise HTTPException(status_code=500, detail=f"Error creating user: {str(e)}")
        finally:
            break


@router.post("/signin", response_model=AuthResponse)
async def signin(request: SigninRequest):
    """Sign in an existing user."""
    async for session in get_session():
        try:
            # Find user
            stmt = select(UserModel).where(UserModel.email == request.email)
            result = await session.execute(stmt)
            user = result.scalar_one_or_none()
            
            if not user:
                raise HTTPException(status_code=401, detail="Invalid email or password")
            
            # Verify password
            if not bcrypt.checkpw(request.password.encode('utf-8'), user.password_hash.encode('utf-8')):
                raise HTTPException(status_code=401, detail="Invalid email or password")
            
            # Generate session token
            session_token = secrets.token_urlsafe(32)
            expires_at = datetime.utcnow() + timedelta(days=7)
            
            return AuthResponse(
                user_id=str(user.id),
                email=user.email,
                session_token=session_token,
                expires_at=expires_at,
            )
        
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Error signing in: {str(e)}")
        finally:
            break


@router.get("/profile", response_model=UserProfileResponse)
async def get_profile(user_id: str):
    """Get user profile information."""
    async for session in get_session():
        try:
            stmt = select(UserModel, UserProfileModel).join(
                UserProfileModel, UserModel.id == UserProfileModel.user_id
            ).where(UserModel.id == int(user_id))
            
            result = await session.execute(stmt)
            row = result.first()
            
            if not row:
                raise HTTPException(status_code=404, detail="User not found")
            
            user, profile = row
            
            return UserProfileResponse(
                user_id=str(user.id),
                email=user.email,
                software_background=profile.software_background,
                hardware_background=profile.hardware_background,
            )
        
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Error fetching profile: {str(e)}")
        finally:
            break


@router.post("/signout")
async def signout():
    """Sign out user (invalidate session)."""
    return {"message": "Signed out successfully"}
