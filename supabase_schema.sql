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
