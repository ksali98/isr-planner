"""
Session and Conversation Storage Module

Stores sessions and conversation history in Supabase to reduce token usage
by allowing agents to retrieve previous context without resending full history.
"""

import json
from datetime import datetime
from typing import Any, Dict, List, Optional
from uuid import uuid4

from .supabase_client import get_supabase_client, is_supabase_configured


# ============ Session Management ============

def create_session(metadata: Optional[Dict[str, Any]] = None) -> Optional[str]:
    """
    Create a new session.

    Args:
        metadata: Optional metadata to store with the session

    Returns:
        Session ID (UUID) or None if storage unavailable
    """
    client = get_supabase_client()
    if not client:
        return None

    try:
        response = client.table("sessions").insert({
            "metadata": metadata or {}
        }).execute()

        if response.data:
            return response.data[0]["id"]
    except Exception as e:
    return None


def get_session(session_id: str) -> Optional[Dict[str, Any]]:
    """
    Get session details by ID.

    Args:
        session_id: The session UUID

    Returns:
        Session data or None
    """
    client = get_supabase_client()
    if not client:
        return None

    try:
        response = client.table("sessions").select("*").eq("id", session_id).execute()
        if response.data:
            return response.data[0]
    except Exception as e:
    return None


def update_session_metadata(session_id: str, metadata: Dict[str, Any]) -> bool:
    """
    Update session metadata.

    Args:
        session_id: The session UUID
        metadata: New metadata to merge with existing

    Returns:
        True if updated successfully
    """
    client = get_supabase_client()
    if not client:
        return False

    try:
        # Get existing metadata
        existing = get_session(session_id)
        if existing:
            merged = {**(existing.get("metadata") or {}), **metadata}
            client.table("sessions").update({"metadata": merged}).eq("id", session_id).execute()
            return True
    except Exception as e:
    return False


def delete_session(session_id: str) -> bool:
    """Delete a session and all its conversations."""
    client = get_supabase_client()
    if not client:
        return False

    try:
        client.table("sessions").delete().eq("id", session_id).execute()
        return True
    except Exception as e:
    return False


# ============ Conversation Management ============

def add_conversation_message(
    session_id: str,
    role: str,
    content: str,
    metadata: Optional[Dict[str, Any]] = None
) -> Optional[int]:
    """
    Add a message to the conversation history.

    Args:
        session_id: The session UUID
        role: Message role ('user', 'assistant', 'system')
        content: Message content
        metadata: Optional metadata (e.g., token count, model used)

    Returns:
        Message ID or None
    """
    client = get_supabase_client()
    if not client:
        return None

    try:
        response = client.table("conversations").insert({
            "session_id": session_id,
            "role": role,
            "content": content,
            "metadata": metadata or {}
        }).execute()

        if response.data:
            return response.data[0]["id"]
    except Exception as e:
    return None


def get_conversation_history(
    session_id: str,
    limit: Optional[int] = None,
    offset: int = 0
) -> List[Dict[str, Any]]:
    """
    Get conversation history for a session.

    Args:
        session_id: The session UUID
        limit: Maximum number of messages to return
        offset: Number of messages to skip (from oldest)

    Returns:
        List of conversation messages
    """
    client = get_supabase_client()
    if not client:
        return []

    try:
        query = client.table("conversations").select("*").eq("session_id", session_id).order("created_at", desc=False)

        if offset > 0:
            query = query.range(offset, offset + (limit or 1000) - 1)
        elif limit:
            query = query.limit(limit)

        response = query.execute()
        return [
            {
                "id": row["id"],
                "role": row["role"],
                "content": row["content"],
                "timestamp": row["created_at"],
                "metadata": row.get("metadata", {})
            }
            for row in response.data
        ]
    except Exception as e:
    return []


def get_recent_messages(session_id: str, count: int = 10) -> List[Dict[str, Any]]:
    """
    Get the most recent messages from a conversation.

    Args:
        session_id: The session UUID
        count: Number of recent messages to retrieve

    Returns:
        List of recent messages (oldest first)
    """
    client = get_supabase_client()
    if not client:
        return []

    try:
        response = client.table("conversations").select("*").eq("session_id", session_id).order("created_at", desc=True).limit(count).execute()

        # Reverse to get oldest first
        messages = list(reversed(response.data))
        return [
            {
                "id": row["id"],
                "role": row["role"],
                "content": row["content"],
                "timestamp": row["created_at"],
                "metadata": row.get("metadata", {})
            }
            for row in messages
        ]
    except Exception as e:
    return []


def get_conversation_summary(session_id: str) -> Dict[str, Any]:
    """
    Get a summary of the conversation (message counts, etc.).

    Args:
        session_id: The session UUID

    Returns:
        Summary statistics
    """
    client = get_supabase_client()
    if not client:
        return {"available": False}

    try:
        response = client.table("conversations").select("role", count="exact").eq("session_id", session_id).execute()

        # Count by role
        role_counts = {}
        for row in response.data:
            role = row["role"]
            role_counts[role] = role_counts.get(role, 0) + 1

        return {
            "available": True,
            "total_messages": response.count or 0,
            "by_role": role_counts
        }
    except Exception as e:
    return {"available": False, "error": str(e)}


def clear_conversation(session_id: str) -> int:
    """
    Clear all messages from a conversation.

    Args:
        session_id: The session UUID

    Returns:
        Number of messages deleted
    """
    client = get_supabase_client()
    if not client:
        return 0

    try:
        # Get count first
        count_response = client.table("conversations").select("id", count="exact").eq("session_id", session_id).execute()
        count = count_response.count or 0

        # Delete all
        client.table("conversations").delete().eq("session_id", session_id).execute()
        return count
    except Exception as e:
    return 0


# ============ Mission Plan Storage ============

def save_mission_plan(
    session_id: Optional[str],
    plan_data: Dict[str, Any],
    total_fuel: float,
    total_points: int,
    drone_count: int
) -> Optional[int]:
    """
    Save a completed mission plan.

    Args:
        session_id: Optional session UUID
        plan_data: The full plan data (JSON serializable)
        total_fuel: Total fuel consumption
        total_points: Total points collected
        drone_count: Number of drones used

    Returns:
        Plan ID or None
    """
    client = get_supabase_client()
    if not client:
        return None

    try:
        response = client.table("mission_plans").insert({
            "session_id": session_id,
            "plan_data": plan_data,
            "total_fuel": total_fuel,
            "total_points": total_points,
            "drone_count": drone_count
        }).execute()

        if response.data:
            return response.data[0]["id"]
    except Exception as e:
    return None


def get_mission_plans(
    session_id: Optional[str] = None,
    limit: int = 10
) -> List[Dict[str, Any]]:
    """
    Get mission plan history.

    Args:
        session_id: Optional filter by session
        limit: Maximum number of plans to return

    Returns:
        List of mission plans
    """
    client = get_supabase_client()
    if not client:
        return []

    try:
        query = client.table("mission_plans").select("*").order("created_at", desc=True).limit(limit)

        if session_id:
            query = query.eq("session_id", session_id)

        response = query.execute()
        return response.data
    except Exception as e:
    return []


# ============ Utility Functions ============

def format_messages_for_prompt(messages: List[Dict[str, Any]], max_chars: int = 4000) -> str:
    """
    Format conversation messages for inclusion in a prompt.

    Args:
        messages: List of message dictionaries
        max_chars: Maximum characters to include

    Returns:
        Formatted string of messages
    """
    if not messages:
        return ""

    lines = []
    total_chars = 0

    for msg in messages:
        role = msg.get("role", "unknown").upper()
        content = msg.get("content", "")

        line = f"[{role}]: {content}"

        if total_chars + len(line) > max_chars:
            lines.append("... (earlier messages truncated)")
            break

        lines.append(line)
        total_chars += len(line) + 1

    return "\n".join(lines)
