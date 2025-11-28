# isr_web/server/agents/graph.py

import os
import operator
from typing import Dict, Any, List, Optional

from typing_extensions import Annotated

from langgraph.graph import StateGraph, END
from langchain_core.messages import (
    BaseMessage,
    HumanMessage,
    AIMessage,
    SystemMessage,
)
from openai import OpenAI

# ---------- LangGraph State ----------

class AgentState(dict):
    """
    State passed around the LangGraph.

    Keys:
      - messages: list of chat messages
      - env: current environment (airports, targets, sams)
      - sequences: current planner sequences per drone
      - drone_configs: drone configuration table
    """
    messages: Annotated[List[BaseMessage], operator.add]
    env: Optional[Dict[str, Any]]
    sequences: Optional[Dict[str, Any]]
    drone_configs: Optional[Dict[str, Any]]


# ---------- LLM client ----------

# Uses OPENAI_API_KEY from environment
_client = OpenAI()


def _summarize_env(env: Optional[Dict[str, Any]]) -> str:
    """Make a compact textual summary of the environment for the agent."""
    if not env:
        return "No environment loaded."

    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    targets_str = ', '.join(
        f"{t.get('id','?')}[p={t.get('priority',5)},type={t.get('type','a')}]"
        for t in targets
    )
    return (
        f"Airports ({len(airports)}): "
        f"{', '.join(str(a.get('id', '?')) for a in airports)}\n"
        f"Targets ({len(targets)}): "
        f"{targets_str}\n"
        f"SAMs ({len(sams)})."
    )


def _summarize_sequences(sequences: Optional[Dict[str, Any]]) -> str:
    if not sequences:
        return "No sequences yet (planner not run)."
    parts = []
    for did, seq in sorted(sequences.items(), key=lambda x: x[0]):
        parts.append(f"D{did}: {seq}")
    return "Current sequences:\n" + "\n".join(parts)


def _summarize_drone_configs(cfg: Optional[Dict[str, Any]]) -> str:
    if not cfg:
        return "No drone config received."
    lines = []
    for did, c in sorted(cfg.items(), key=lambda x: x[0]):
        types_str = ""
        ta = c.get("target_access", {})
        if ta:
            allowed = [k.upper() for k, v in ta.items() if v]
            types_str = " types=" + (",".join(allowed) if allowed else "NONE")
        lines.append(
            f"D{did}: enabled={c.get('enabled', True)}, "
            f"fuel={c.get('fuel_budget', 0)}, "
            f"start={c.get('start_airport')}, "
            f"end={c.get('end_airport')}{types_str}"
        )
    return "Drone configs:\n" + "\n".join(lines)


# ---------- Node: planner assistant ----------

def planner_agent(state: AgentState) -> AgentState:
    """
    Single-node agent:
    - Reads the latest human message.
    - Has access to env, sequences, and drone_configs.
    - Returns a suggested set of sequences (D1: A1,T1,T2,A1 …) and reasoning.
    """
    env = state.get("env")
    sequences = state.get("sequences")
    drone_configs = state.get("drone_configs")
    messages: List[BaseMessage] = state.get("messages", [])

    user_msg = ""
    for m in reversed(messages):
        if isinstance(m, HumanMessage):
            user_msg = m.content
            break

    sys_text = (
        "You are an ISR mission planning assistant for a 100x100 grid environment.\n"
        "- Airports are labeled A1, A2, ...\n"
        "- Targets are labeled T1, T2, ... each with a priority and type (A–E).\n"
        "- SAMs / NFZs exist but are handled by a separate low-level planner; "
        "you should NOT worry about geometric avoidance, only high-level target ordering.\n\n"
        "Your job:\n"
        "1) Read the current environment summary.\n"
        "2) Read the drone configuration (fuel budgets, allowed types).\n"
        "3) Read the user's natural-language strategy.\n"
        "4) Propose a mission sequence for EACH enabled drone, in this strict format:\n"
        "   D1: A1,T3,T5,A1  D2: A2,T2,A2  D3:  D4:  D5:\n"
        "   - Use commas inside each drone's route.\n"
        "   - Separate drones by two spaces.\n"
        "   - If a drone is unused, leave it blank after 'Dk:'.\n"
        "5) Keep total distance roughly consistent with fuel budgets (fewer targets for small budgets).\n\n"
        "IMPORTANT:\n"
        "- DO NOT invent airports or targets that do not exist.\n"
        "- Always start and end each drone at its configured start/end airport.\n"
        "- If the user says 'keep existing sequences and only fix D3', you must respect current sequences."
    )

    env_text = _summarize_env(env)
    seq_text = _summarize_sequences(sequences)
    cfg_text = _summarize_drone_configs(drone_configs)

    system_message = SystemMessage(content=sys_text)
    context_message = SystemMessage(
        content=(
            "Environment summary:\n"
            f"{env_text}\n\n"
            f"{cfg_text}\n\n"
            f"{seq_text}\n\n"
            "Now respond with your reasoning followed by a single line that begins with:\n"
            "\"SEQUENCES:\" and then the exact multi-drone sequence format.\n"
            "Example:\n"
            "SEQUENCES: D1: A1,T1,T5,A1  D2: A2,T3,A2  D3:  D4:  D5:\n"
        )
    )

    llm_messages: List[BaseMessage] = [system_message, context_message] + messages

    resp = _client.chat.completions.create(
        model="gpt-4.1-mini",  # user can change this
        messages=[m.to_dict() for m in llm_messages],
        temperature=0.3,
    )

    reply_text = resp.choices[0].message.content

    ai_msg = AIMessage(content=reply_text)
    new_messages = messages + [ai_msg]

    new_state = AgentState(state)
    new_state["messages"] = new_messages
    return new_state


# ---------- Build & compile the graph ----------

def build_workflow():
    g = StateGraph(AgentState)
    g.add_node("planner", planner_agent)
    g.set_entry_point("planner")
    g.add_edge("planner", END)
    return g.compile()


# Singleton workflow used by FastAPI
workflow = build_workflow()
