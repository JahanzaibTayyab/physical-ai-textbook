#!/usr/bin/env python3
"""
Database initialization script.

Creates all database tables.
"""

import asyncio
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from rag_chatbot_backend.database.connection import init_db

# Load environment variables
load_dotenv()


async def main():
    """Initialize database."""
    print("📊 Initializing database...")
    try:
        await init_db()
        print("✅ Database initialized successfully!")
    except Exception as e:
        print(f"❌ Error initializing database: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

