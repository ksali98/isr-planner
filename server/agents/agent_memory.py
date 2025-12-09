"""
Agent Memory Module

Memory storage for agent learning/corrections.
Uses Supabase when configured, falls back to JSON file storage.
Shared across all agent versions (v1, v2, v3).
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

# Memory file path (fallback for local development)
MEMORY_FILE = Path(__file__).parent / "agent_memory.json"


def _get_supabase():
    """Get Supabase client if available."""
    try:
        from server.database.supabase_client import get_supabase_client
        return get_supabase_client()
    except ImportError:
        return None


def _use_supabase() -> bool:
    """Check if we should use Supabase storage."""
    return _get_supabase() is not None


# ============ JSON File Storage (Fallback) ============

def _load_memory_file() -> List[Dict[str, Any]]:
    """Load all memories from JSON file."""
    if MEMORY_FILE.exists():
        try:
            return json.loads(MEMORY_FILE.read_text())
        except json.JSONDecodeError:
            return []
    return []


def _save_memory_file(memories: List[Dict[str, Any]]) -> None:
    """Save memories to JSON file."""
    MEMORY_FILE.write_text(json.dumps(memories, indent=2))


# ============ Supabase Storage ============

def _load_memory_supabase() -> List[Dict[str, Any]]:
    """Load all memories from Supabase."""
    client = _get_supabase()
    if not client:
        return []

    try:
        response = client.table("agent_memories").select("*").order("created_at", desc=False).execute()
        return [
            {
                "id": row["id"],
                "content": row["content"],
                "category": row["category"],
                "timestamp": row["created_at"],
            }
            for row in response.data
        ]
    except Exception as e:
        print(f"Error loading memories from Supabase: {e}")
        return []


def _add_memory_supabase(content: str, category: str) -> Optional[Dict[str, Any]]:
    """Add a memory to Supabase."""
    client = _get_supabase()
    if not client:
        return None

    try:
        response = client.table("agent_memories").insert({
            "content": content,
            "category": category,
        }).execute()

        if response.data:
            row = response.data[0]
            return {
                "id": row["id"],
                "content": row["content"],
                "category": row["category"],
                "timestamp": row["created_at"],
            }
    except Exception as e:
        print(f"Error adding memory to Supabase: {e}")
    return None


def _clear_memory_supabase() -> int:
    """Clear all memories from Supabase. Returns count of cleared entries."""
    client = _get_supabase()
    if not client:
        return 0

    try:
        # Get count first
        count_response = client.table("agent_memories").select("id", count="exact").execute()
        count = count_response.count or 0

        # Delete all
        client.table("agent_memories").delete().neq("id", 0).execute()
        return count
    except Exception as e:
        print(f"Error clearing memories from Supabase: {e}")
        return 0


def _delete_memory_supabase(memory_id: int) -> bool:
    """Delete a specific memory from Supabase."""
    client = _get_supabase()
    if not client:
        return False

    try:
        response = client.table("agent_memories").delete().eq("id", memory_id).execute()
        return len(response.data) > 0
    except Exception as e:
        print(f"Error deleting memory from Supabase: {e}")
        return False


# ============ Public API (Auto-selects storage backend) ============

def load_memory() -> List[Dict[str, Any]]:
    """Load all memories from storage."""
    if _use_supabase():
        return _load_memory_supabase()
    return _load_memory_file()


def save_memory(memories: List[Dict[str, Any]]) -> None:
    """Save memories to file (only used for JSON fallback)."""
    _save_memory_file(memories)


def add_memory(content: str, category: str = "correction") -> Dict[str, Any]:
    """
    Add a new memory entry.

    Args:
        content: The memory content
        category: Type of memory (correction, instruction, preference, fact)

    Returns:
        The created memory entry
    """
    if _use_supabase():
        result = _add_memory_supabase(content, category)
        if result:
            return result
        # Fall through to JSON if Supabase fails

    # JSON file storage
    memories = _load_memory_file()
    entry = {
        "id": len(memories) + 1,
        "content": content,
        "category": category,
        "timestamp": datetime.now().isoformat(),
    }
    memories.append(entry)
    _save_memory_file(memories)
    return entry


def clear_memory() -> int:
    """Clear all memories. Returns count of cleared entries."""
    if _use_supabase():
        return _clear_memory_supabase()

    memories = _load_memory_file()
    count = len(memories)
    _save_memory_file([])
    return count


def delete_memory(memory_id: int) -> bool:
    """Delete a specific memory by ID. Returns True if found and deleted."""
    if _use_supabase():
        return _delete_memory_supabase(memory_id)

    memories = _load_memory_file()
    original_count = len(memories)
    memories = [m for m in memories if m.get("id") != memory_id]

    if len(memories) < original_count:
        _save_memory_file(memories)
        return True
    return False


def get_memory_for_prompt() -> str:
    """
    Get formatted memories for inclusion in agent prompt.

    Returns:
        Formatted string of memories to include in system prompt
    """
    memories = load_memory()
    if not memories:
        return ""

    lines = ["\n## Learned Corrections & Instructions"]
    for mem in memories:
        category = mem.get("category", "note")
        content = mem.get("content", "")
        lines.append(f"- [{category}] {content}")

    return "\n".join(lines)


def get_storage_backend() -> str:
    """Get the current storage backend being used."""
    return "supabase" if _use_supabase() else "json_file"
