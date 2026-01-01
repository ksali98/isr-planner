-- ISR Mission Planner - Supabase Database Schema
-- Run this in the Supabase SQL Editor to create the required tables

-- 1. Agent Memories Table
-- Stores corrections, instructions, and learned facts for agents
CREATE TABLE IF NOT EXISTS agent_memories (
    id BIGSERIAL PRIMARY KEY,
    content TEXT NOT NULL,
    category VARCHAR(50) DEFAULT 'correction',
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Index for faster category filtering
CREATE INDEX IF NOT EXISTS idx_memories_category ON agent_memories(category);
CREATE INDEX IF NOT EXISTS idx_memories_created_at ON agent_memories(created_at DESC);

-- 2. Sessions Table
-- Stores user sessions for conversation continuity
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

-- 3. Conversations Table
-- Stores conversation history to reduce token usage
CREATE TABLE IF NOT EXISTS conversations (
    id BIGSERIAL PRIMARY KEY,
    session_id UUID REFERENCES sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

-- Indexes for conversation queries
CREATE INDEX IF NOT EXISTS idx_conversations_session ON conversations(session_id);
CREATE INDEX IF NOT EXISTS idx_conversations_created ON conversations(created_at DESC);

-- 4. Mission Plans Table
-- Stores completed mission plans for reference/history
CREATE TABLE IF NOT EXISTS mission_plans (
    id BIGSERIAL PRIMARY KEY,
    session_id UUID REFERENCES sessions(id) ON DELETE SET NULL,
    plan_data JSONB NOT NULL,
    total_fuel FLOAT,
    total_points INTEGER,
    drone_count INTEGER,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Index for mission history queries
CREATE INDEX IF NOT EXISTS idx_plans_session ON mission_plans(session_id);
CREATE INDEX IF NOT EXISTS idx_plans_created ON mission_plans(created_at DESC);

-- 5. Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Triggers for auto-updating updated_at
DROP TRIGGER IF EXISTS update_agent_memories_updated_at ON agent_memories;
CREATE TRIGGER update_agent_memories_updated_at
    BEFORE UPDATE ON agent_memories
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

DROP TRIGGER IF EXISTS update_sessions_updated_at ON sessions;
CREATE TRIGGER update_sessions_updated_at
    BEFORE UPDATE ON sessions
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- 6. Enable Row Level Security (RLS) for production
-- Uncomment these if you want to add security policies
-- ALTER TABLE agent_memories ENABLE ROW LEVEL SECURITY;
-- ALTER TABLE sessions ENABLE ROW LEVEL SECURITY;
-- ALTER TABLE conversations ENABLE ROW LEVEL SECURITY;
-- ALTER TABLE mission_plans ENABLE ROW LEVEL SECURITY;

-- Example policies (adjust based on your auth setup):
-- CREATE POLICY "Allow all for service role" ON agent_memories FOR ALL USING (true);

-- =====================================================
-- DECISION TRACE V1 + LEARNING V1 TABLES
-- =====================================================

-- Enable UUID generation (Supabase typically has this available)
CREATE EXTENSION IF NOT EXISTS pgcrypto;

-- 7. Agent Runs Table
-- One row per v4 agentic solve (the canonical run record)
CREATE TABLE IF NOT EXISTS agent_runs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    -- Context references (optional)
    env_version_id UUID NULL,               -- link to env_versions.id if desired
    mission_id UUID NULL,                   -- optional parent mission
    user_id TEXT NULL,                      -- optional user who triggered

    -- Run configuration
    agent_version TEXT NOT NULL DEFAULT 'v4',
    mode TEXT NOT NULL DEFAULT 'agentic',   -- 'agentic' | 'manual' | 'optimizer_only'
    request_text TEXT NULL,                 -- Raw user request

    -- Intent
    objective TEXT NULL,                    -- Compiled objective
    hard_constraints JSONB NOT NULL DEFAULT '{}'::jsonb,
    soft_preferences JSONB NOT NULL DEFAULT '{}'::jsonb,

    -- Solver info (promoted for fast filtering/dashboards)
    solver_type TEXT NULL,                  -- 'orienteering_exact' | 'heuristic' | 'hybrid'
    solver_runtime_ms INTEGER NULL,

    -- Results
    total_points NUMERIC NULL,
    total_fuel_used NUMERIC NULL,
    runtime_ms INTEGER NULL,
    is_valid BOOLEAN NULL,
    routes JSONB NOT NULL DEFAULT '{}'::jsonb,   -- {"D1":["A1","T3","A2"], ...}
    summary JSONB NOT NULL DEFAULT '{}'::jsonb,  -- per-drone + totals

    -- Segment support (for checkpoint replans)
    parent_run_id UUID NULL REFERENCES agent_runs(id) ON DELETE SET NULL,
    segment_index INTEGER NULL              -- 0, 1, 2, ... for segment replans
);

-- Indexes for agent_runs
CREATE INDEX IF NOT EXISTS agent_runs_created_at_idx ON agent_runs(created_at DESC);
CREATE INDEX IF NOT EXISTS agent_runs_env_idx ON agent_runs(env_version_id);
CREATE INDEX IF NOT EXISTS agent_runs_valid_idx ON agent_runs(is_valid);
CREATE INDEX IF NOT EXISTS agent_runs_solver_type_idx ON agent_runs(solver_type);
CREATE INDEX IF NOT EXISTS agent_runs_solver_runtime_idx ON agent_runs(solver_runtime_ms);
CREATE INDEX IF NOT EXISTS agent_runs_parent_idx ON agent_runs(parent_run_id);

-- 8. Agent Traces Table
-- Full Decision Trace payload (JSONB blob for flexibility)
-- One trace per run (enforced by unique index)
CREATE TABLE IF NOT EXISTS agent_traces (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    agent_run_id UUID NOT NULL REFERENCES agent_runs(id) ON DELETE CASCADE,

    -- The full trace as JSONB for schema evolution without migrations
    -- Structure: {
    --   "env_hash": "abc123",
    --   "eligibility": { "1": { "eligible": ["T1","T2"], "excluded": [...] } },
    --   "allocation": { "algorithm": "greedy", "assignments": {...}, "rationale": {...} },
    --   "solver": { "type": "exact", "candidates": 100, "runtime_ms": 150 },
    --   "final_evidence": { "1": { "waypoints": [...], "fuel": 150.5, "points": 45 } }
    -- }
    trace JSONB NOT NULL
);

-- One trace per run (enforced)
CREATE UNIQUE INDEX IF NOT EXISTS agent_traces_one_per_run ON agent_traces(agent_run_id);
CREATE INDEX IF NOT EXISTS agent_traces_run_idx ON agent_traces(agent_run_id);

-- 9. Agent Optimizer Steps Table
-- Normalized optimizer step history for querying/analytics
CREATE TABLE IF NOT EXISTS agent_optimizer_steps (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    agent_run_id UUID NOT NULL REFERENCES agent_runs(id) ON DELETE CASCADE,

    step_index INTEGER NOT NULL,            -- 0, 1, 2, ...
    operator TEXT NOT NULL,                 -- 'insert_missed' | 'swap_closer' | 'cross_remove' | '2opt'

    before_routes JSONB NOT NULL,           -- Routes before this step
    after_routes JSONB NOT NULL,            -- Routes after this step

    delta JSONB NOT NULL,                   -- {points:+, distance:-, fuel:-, per_drone:{...}}
    accepted BOOLEAN NOT NULL DEFAULT true, -- Was this step kept?
    notes TEXT NULL,                        -- Optional explanation

    -- Validation tracking
    validation_status TEXT NOT NULL DEFAULT 'unknown',  -- 'pass' | 'fail' | 'skipped' | 'unknown'
    validation_details JSONB NOT NULL DEFAULT '{}'::jsonb  -- { "fuel_ok": true, "sam_ok": true, ... }
);

-- One step per (run, index)
CREATE UNIQUE INDEX IF NOT EXISTS agent_optimizer_steps_unique ON agent_optimizer_steps(agent_run_id, step_index);
CREATE INDEX IF NOT EXISTS agent_optimizer_steps_operator_idx ON agent_optimizer_steps(operator);

-- 10. Agent Policy Rules Table
-- Learning v1: deterministic policy rules from corrections
CREATE TABLE IF NOT EXISTS agent_policy_rules (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    -- Rule status
    active BOOLEAN NOT NULL DEFAULT true,
    scope TEXT NOT NULL DEFAULT 'global',   -- 'global' | 'scenario' | 'mission_type'
    mode TEXT NOT NULL DEFAULT 'agentic',   -- 'agentic' | 'ui' | 'both'

    -- Rule definition
    category TEXT NOT NULL,                 -- 'correction' | 'instruction' | 'preference'
    title TEXT NULL,                        -- Human-readable name
    rule JSONB NOT NULL,                    -- { "trigger": {...}, "action": {...} }

    -- Provenance
    created_by TEXT NULL,                   -- 'system' | 'user' | 'agent'
    notes TEXT NULL
);

-- Composite index for common query pattern
CREATE INDEX IF NOT EXISTS agent_policy_rules_active_idx ON agent_policy_rules(active, scope, mode);

-- =====================================================
-- DISTANCE MATRIX CACHE TABLE
-- =====================================================

-- 11. Distance Matrices Table
-- Stores metadata + the matrix payload (JSONB) for SAM-aware distances
-- Key: (env_hash, routing_model_hash) - unique combination
CREATE TABLE IF NOT EXISTS distance_matrices (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    -- Cache keys
    env_hash TEXT NOT NULL,                    -- SHA256 of canonical env (sorted airports, targets, sams)
    routing_model_hash TEXT NOT NULL,          -- SHA256 of routing model params
    sam_mode TEXT NOT NULL DEFAULT 'hard_v1',  -- hard_v1 | risk_v2 | etc.

    -- Matrix data
    node_index JSONB NOT NULL,                 -- {"A1":0,"T3":1,...} stable ordering
    matrix JSONB NOT NULL,                     -- NxN distances (flat array or list of lists)
    metadata JSONB NOT NULL DEFAULT '{}'::jsonb -- runtime_ms, params, counts, excluded_targets, etc.
);

-- Unique constraint on cache keys
CREATE UNIQUE INDEX IF NOT EXISTS distance_matrices_unique
ON distance_matrices(env_hash, routing_model_hash);

-- Indexes for distance_matrices
CREATE INDEX IF NOT EXISTS distance_matrices_env_idx ON distance_matrices(env_hash);
CREATE INDEX IF NOT EXISTS distance_matrices_mode_idx ON distance_matrices(sam_mode);

-- =====================================================
-- OPTIONAL: SAM PATHS TABLE (for visualization)
-- =====================================================

-- 12. SAM Paths Table (optional)
-- Preserves SAM-avoiding polylines for trajectory visualization
CREATE TABLE IF NOT EXISTS sam_paths (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    env_hash TEXT NOT NULL,
    routing_model_hash TEXT NOT NULL,
    sam_mode TEXT NOT NULL DEFAULT 'hard_v1',

    paths JSONB NOT NULL,                      -- {"A1->T3": [[x,y],...], ...}
    metadata JSONB NOT NULL DEFAULT '{}'::jsonb
);

CREATE UNIQUE INDEX IF NOT EXISTS sam_paths_unique
ON sam_paths(env_hash, routing_model_hash);

-- =====================================================
-- ADD REFERENCES TO AGENT RUNS (for fast filtering)
-- =====================================================

-- Add columns to agent_runs for matrix references
ALTER TABLE agent_runs
ADD COLUMN IF NOT EXISTS env_hash TEXT NULL;

ALTER TABLE agent_runs
ADD COLUMN IF NOT EXISTS routing_model_hash TEXT NULL;

ALTER TABLE agent_runs
ADD COLUMN IF NOT EXISTS distance_matrix_id UUID NULL REFERENCES distance_matrices(id) ON DELETE SET NULL;
