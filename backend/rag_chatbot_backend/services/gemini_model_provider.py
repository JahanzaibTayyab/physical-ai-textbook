"""
Gemini Model Provider for OpenAI Agents SDK

This module implements a custom model provider that uses Gemini API
with OpenAI Agents SDK, using Gemini's OpenAI-compatible endpoint.

Gemini Base URL: https://generativelanguage.googleapis.com/v1beta/openai

Reference: https://github.com/openai/openai-agents-python/blob/main/examples/model_providers/custom_example_provider.py
"""

from __future__ import annotations
import os
from typing import Optional
from openai import AsyncOpenAI
from agents import Model, ModelProvider, OpenAIChatCompletionsModel

# Gemini OpenAI-compatible endpoint
GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai"

def _get_gemini_client():
    """Get or create Gemini client (lazy initialization)."""
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    if not GEMINI_API_KEY:
        raise ValueError("GEMINI_API_KEY environment variable is required")
    
    return AsyncOpenAI(
        base_url=GEMINI_BASE_URL,
        api_key=GEMINI_API_KEY
    )


class GeminiModelProvider(ModelProvider):
    """
    Model provider that uses Gemini's OpenAI-compatible endpoint.
    
    This uses OpenAIChatCompletionsModel with an AsyncOpenAI client
    pointing to Gemini's endpoint, making integration simple.
    
    Usage:
        provider = GeminiModelProvider(model_name="gemini-2.5-flash")
        model = provider.get_model()
    """
    
    def __init__(self, model_name: str = "gemini-2.5-flash"):
        """
        Initialize Gemini model provider.
        
        Args:
            model_name: Gemini model to use (default: "gemini-2.5-flash")
                      Options: "gemini-2.5-flash", "gemini-2.5-pro", etc.
        """
        self.model_name = model_name
        self._client = None
    
    @property
    def client(self):
        """Lazy-load client."""
        if self._client is None:
            self._client = _get_gemini_client()
        return self._client
    
    def get_model(self, model_name: Optional[str] = None) -> Model:
        """
        Get a Gemini model instance using OpenAI-compatible interface.
        
        Args:
            model_name: Optional model name override
            
        Returns:
            OpenAIChatCompletionsModel instance configured for Gemini
        """
        return OpenAIChatCompletionsModel(
            model=model_name or self.model_name,
            openai_client=self.client
        )


# Global provider instance (lazy initialization)
_GEMINI_PROVIDER = None

def get_gemini_provider():
    """Get or create global Gemini provider."""
    global _GEMINI_PROVIDER
    if _GEMINI_PROVIDER is None:
        _GEMINI_PROVIDER = GeminiModelProvider(model_name="gemini-2.5-flash")
    return _GEMINI_PROVIDER

