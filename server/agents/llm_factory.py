"""
LLM Factory - Provider-agnostic LLM instantiation

Supports multiple AI providers with a unified interface:
- Anthropic (Claude)
- OpenAI (GPT)
- Google (Gemini)
- Groq (Llama, Mixtral)

Usage:
    from server.agents.llm_factory import get_llm, get_available_providers

    # Get LLM with specific provider/model
    llm = get_llm(provider="google", model="gemini-1.5-pro", api_key="xxx")

    # Get available providers and models
    providers = get_available_providers()
"""

import os
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field


# =============================================================================
# Provider Configuration
# =============================================================================

@dataclass
class ModelInfo:
    """Information about a specific model."""
    id: str
    name: str
    description: str
    context_window: int = 128000
    supports_tools: bool = True


@dataclass
class ProviderInfo:
    """Information about an AI provider."""
    id: str
    name: str
    env_key: str  # Environment variable name for API key
    models: List[ModelInfo] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "env_key": self.env_key,
            "models": [
                {
                    "id": m.id,
                    "name": m.name,
                    "description": m.description,
                    "context_window": m.context_window,
                    "supports_tools": m.supports_tools
                }
                for m in self.models
            ]
        }


# Define supported providers and their models
PROVIDERS: Dict[str, ProviderInfo] = {
    "anthropic": ProviderInfo(
        id="anthropic",
        name="Anthropic",
        env_key="ANTHROPIC_API_KEY",
        models=[
            ModelInfo("claude-sonnet-4-20250514", "Claude Sonnet 4", "Best balance of intelligence and speed"),
            ModelInfo("claude-opus-4-20250514", "Claude Opus 4", "Most capable, best for complex tasks"),
            ModelInfo("claude-3-5-haiku-20241022", "Claude 3.5 Haiku", "Fastest, most economical"),
        ]
    ),
    "openai": ProviderInfo(
        id="openai",
        name="OpenAI",
        env_key="OPENAI_API_KEY",
        models=[
            ModelInfo("gpt-4o", "GPT-4o", "Most capable GPT model"),
            ModelInfo("gpt-4o-mini", "GPT-4o Mini", "Fast and affordable"),
            ModelInfo("gpt-4-turbo", "GPT-4 Turbo", "Previous generation, 128K context"),
            ModelInfo("o1", "o1", "Reasoning model for complex problems"),
            ModelInfo("o1-mini", "o1 Mini", "Faster reasoning model"),
        ]
    ),
    "google": ProviderInfo(
        id="google",
        name="Google",
        env_key="GOOGLE_API_KEY",
        models=[
            ModelInfo("gemini-1.5-pro", "Gemini 1.5 Pro", "Best for complex tasks, 1M context"),
            ModelInfo("gemini-1.5-flash", "Gemini 1.5 Flash", "Fast and versatile"),
            ModelInfo("gemini-2.0-flash", "Gemini 2.0 Flash", "Latest generation, multimodal"),
        ]
    ),
    "groq": ProviderInfo(
        id="groq",
        name="Groq",
        env_key="GROQ_API_KEY",
        models=[
            ModelInfo("llama-3.3-70b-versatile", "Llama 3.3 70B", "Most capable open model"),
            ModelInfo("llama-3.1-8b-instant", "Llama 3.1 8B", "Ultra-fast responses"),
            ModelInfo("mixtral-8x7b-32768", "Mixtral 8x7B", "32K context, fast"),
        ]
    ),
}


# =============================================================================
# LLM Factory Functions
# =============================================================================

def get_available_providers() -> List[Dict[str, Any]]:
    """
    Get list of all available providers and their models.

    Returns:
        List of provider info dicts with models
    """
    return [p.to_dict() for p in PROVIDERS.values()]


def get_provider_info(provider_id: str) -> Optional[ProviderInfo]:
    """Get info for a specific provider."""
    return PROVIDERS.get(provider_id)


def get_default_model(provider_id: str) -> Optional[str]:
    """Get the default (first) model for a provider."""
    provider = PROVIDERS.get(provider_id)
    if provider and provider.models:
        return provider.models[0].id
    return None


def validate_provider_model(provider_id: str, model_id: str) -> Tuple[bool, str]:
    """
    Validate that a provider/model combination is supported.

    Returns:
        (is_valid, error_message)
    """
    if provider_id not in PROVIDERS:
        return False, f"Unknown provider: {provider_id}. Supported: {list(PROVIDERS.keys())}"

    provider = PROVIDERS[provider_id]
    model_ids = [m.id for m in provider.models]

    if model_id not in model_ids:
        return False, f"Unknown model '{model_id}' for {provider.name}. Supported: {model_ids}"

    return True, ""


def get_llm(
    provider: str = None,
    model: str = None,
    api_key: str = None,
    temperature: float = 0,
    **kwargs
) -> Any:
    """
    Get a LangChain chat model for the specified provider.

    Args:
        provider: Provider ID (anthropic, openai, google, groq)
        model: Model ID (e.g., "claude-sonnet-4-20250514", "gpt-4o")
        api_key: API key (if not provided, reads from environment)
        temperature: Model temperature (default 0 for deterministic)
        **kwargs: Additional model-specific arguments

    Returns:
        LangChain BaseChatModel instance

    Raises:
        ValueError: If provider/model invalid or API key missing
        ImportError: If required package not installed
    """
    # Default to anthropic if not specified
    provider = provider or os.getenv("LLM_PROVIDER", "anthropic")

    # Validate provider
    if provider not in PROVIDERS:
        raise ValueError(f"Unknown provider: {provider}. Supported: {list(PROVIDERS.keys())}")

    provider_info = PROVIDERS[provider]

    # Default to first model if not specified
    model = model or os.getenv("LLM_MODEL") or get_default_model(provider)

    # Validate model
    is_valid, error = validate_provider_model(provider, model)
    if not is_valid:
        raise ValueError(error)

    # Get API key from parameter, environment, or raise error
    if not api_key:
        api_key = os.getenv(provider_info.env_key)

    if not api_key:
        raise ValueError(
            f"No API key for {provider_info.name}. "
            f"Set {provider_info.env_key} environment variable or pass api_key parameter."
        )

    # Create the appropriate LLM
    if provider == "anthropic":
        return _create_anthropic_llm(model, api_key, temperature, **kwargs)
    elif provider == "openai":
        return _create_openai_llm(model, api_key, temperature, **kwargs)
    elif provider == "google":
        return _create_google_llm(model, api_key, temperature, **kwargs)
    elif provider == "groq":
        return _create_groq_llm(model, api_key, temperature, **kwargs)
    else:
        raise ValueError(f"Provider {provider} not implemented")


def _create_anthropic_llm(model: str, api_key: str, temperature: float, **kwargs):
    """Create Anthropic (Claude) LLM."""
    try:
        from langchain_anthropic import ChatAnthropic
    except ImportError:
        raise ImportError("Install langchain-anthropic: pip install langchain-anthropic")

    # Handle Anthropic-specific kwargs
    model_kwargs = kwargs.pop("model_kwargs", {})
    if "thinking" not in model_kwargs:
        model_kwargs["thinking"] = {"type": "disabled"}

    return ChatAnthropic(
        model=model,
        temperature=temperature,
        api_key=api_key,
        model_kwargs=model_kwargs,
        **kwargs
    )


def _create_openai_llm(model: str, api_key: str, temperature: float, **kwargs):
    """Create OpenAI (GPT) LLM."""
    try:
        from langchain_openai import ChatOpenAI
    except ImportError:
        raise ImportError("Install langchain-openai: pip install langchain-openai")

    return ChatOpenAI(
        model=model,
        temperature=temperature,
        api_key=api_key,
        **kwargs
    )


def _create_google_llm(model: str, api_key: str, temperature: float, **kwargs):
    """Create Google (Gemini) LLM."""
    try:
        from langchain_google_genai import ChatGoogleGenerativeAI
    except ImportError:
        raise ImportError("Install langchain-google-genai: pip install langchain-google-genai")

    return ChatGoogleGenerativeAI(
        model=model,
        temperature=temperature,
        google_api_key=api_key,
        **kwargs
    )


def _create_groq_llm(model: str, api_key: str, temperature: float, **kwargs):
    """Create Groq LLM."""
    try:
        from langchain_groq import ChatGroq
    except ImportError:
        raise ImportError("Install langchain-groq: pip install langchain-groq")

    return ChatGroq(
        model=model,
        temperature=temperature,
        api_key=api_key,
        **kwargs
    )


# =============================================================================
# Connection Testing
# =============================================================================

def test_connection(
    provider: str,
    model: str,
    api_key: str
) -> Tuple[bool, str]:
    """
    Test if we can connect to a provider with given credentials.

    Args:
        provider: Provider ID
        model: Model ID
        api_key: API key to test

    Returns:
        (success, message) tuple
    """
    try:
        # Validate inputs
        is_valid, error = validate_provider_model(provider, model)
        if not is_valid:
            return False, error

        if not api_key or len(api_key.strip()) < 10:
            return False, "API key is too short or empty"

        # Create LLM and make a simple test call
        llm = get_llm(provider=provider, model=model, api_key=api_key)

        # Simple test message
        response = llm.invoke("Say 'Connection successful' and nothing else.")

        # Check we got a response
        if response and hasattr(response, 'content') and response.content:
            return True, f"Connected to {PROVIDERS[provider].name} ({model})"
        else:
            return False, "Received empty response from model"

    except ImportError as e:
        return False, f"Missing package: {str(e)}"
    except Exception as e:
        error_msg = str(e)
        # Clean up common error messages
        if "401" in error_msg or "unauthorized" in error_msg.lower():
            return False, "Invalid API key"
        elif "404" in error_msg:
            return False, f"Model '{model}' not found or not accessible"
        elif "rate" in error_msg.lower():
            return False, "Rate limited - try again later"
        else:
            return False, f"Connection failed: {error_msg[:100]}"


# =============================================================================
# Utility Functions
# =============================================================================

def get_env_key_for_provider(provider_id: str) -> Optional[str]:
    """Get the environment variable name for a provider's API key."""
    provider = PROVIDERS.get(provider_id)
    return provider.env_key if provider else None


def check_provider_configured(provider_id: str) -> bool:
    """Check if a provider has an API key configured in environment."""
    provider = PROVIDERS.get(provider_id)
    if not provider:
        return False
    return bool(os.getenv(provider.env_key))


def get_configured_providers() -> List[str]:
    """Get list of provider IDs that have API keys configured."""
    return [pid for pid in PROVIDERS if check_provider_configured(pid)]
