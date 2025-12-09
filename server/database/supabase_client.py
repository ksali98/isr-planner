"""
Supabase Client Module

Provides a singleton Supabase client for database operations.
"""

import os
from typing import Optional
from functools import lru_cache

# Supabase client instance
_supabase_client = None


def get_supabase_client():
    """
    Get the Supabase client instance.

    Returns:
        Supabase client or None if not configured
    """
    global _supabase_client

    if _supabase_client is not None:
        return _supabase_client

    supabase_url = os.environ.get("SUPABASE_URL")
    supabase_key = os.environ.get("SUPABASE_KEY")

    if not supabase_url or not supabase_key:
        return None

    try:
        from supabase import create_client, Client
        _supabase_client = create_client(supabase_url, supabase_key)
        return _supabase_client
    except ImportError:
        print("Warning: supabase package not installed. Run: pip install supabase")
        return None
    except Exception as e:
        print(f"Error creating Supabase client: {e}")
        return None


def is_supabase_configured() -> bool:
    """Check if Supabase is properly configured."""
    return get_supabase_client() is not None
