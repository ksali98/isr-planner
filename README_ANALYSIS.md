# Drone Configuration Flow Analysis Documentation

## Overview

This analysis traces how drone configurations (fuel_budget, start_airport, end_airport, target_access) flow from the UI through the Run Planner and Agent Solve v4 endpoints. It includes exact file paths, function names, and line numbers for all configuration sources, transformations, and overrides.

## Documents Included

### 1. **ANALYSIS_SUMMARY.txt** (Quick Overview)
- Executive summary of key findings
- 7 critical configuration areas identified
- All file paths listed
- Validation checklist
- Quick recommended next steps

**Use this when**: You need a quick 5-minute overview

### 2. **DRONE_CONFIG_FLOW.md** (Detailed Reference)
- **12 major sections** covering the complete flow:
  1. UI state storage (webapp/isr.js)
  2. Run Planner endpoint (server/main.py)
  3. Agent Solve v4 endpoint (isr_agent_multi_v4.py)
  4. Allocation and caching mechanisms
  5. start_airport overrides (5 locations)
  6. end_airport defaults/ignores (7 locations)
  7. mode defaults to "return" (3 locations)
  8. Mission ID, distance matrix, and allocation caching
  9. Critical flow diagrams
  10. Summary table of all locations
  11. Caveats and edge cases
  12. Absolute file references

**Use this when**: You need comprehensive code references and understand the full architecture

### 3. **DRONE_CONFIG_QUICK_REF.md** (Cheat Sheet)
- Critical paths (2 main flows)
- All start_airport overrides in table format (5 locations)
- All end_airport defaults in table format (7 locations)
- Mode defaults table (3 locations)
- Caching locations with implementation details
- Config normalization mapping (v4 Agent)
- Payload structures (JSON examples)
- Key functions reference table
- Debug outputs for monitoring
- Edge cases (7 important ones)
- Testing checklist

**Use this when**: You're actively debugging or looking for a specific piece of information

## Quick Navigation

### "Where is X stored?"
→ See: DRONE_CONFIG_QUICK_REF.md, section "Critical Paths"

### "How does X flow through the system?"
→ See: DRONE_CONFIG_FLOW.md, section "Critical Flow Diagrams"

### "What overrides start_airport?"
→ See: DRONE_CONFIG_QUICK_REF.md, table "All start_airport Overrides"

### "When is end_airport ignored?"
→ See: DRONE_CONFIG_QUICK_REF.md, table "All end_airport Defaults/Ignores"

### "Where are allocations cached?"
→ See: DRONE_CONFIG_FLOW.md, section "8.3 Allocation Caching in Mission Store"

### "How does the distance matrix cache work?"
→ See: DRONE_CONFIG_QUICK_REF.md, section "Caching Locations"

### "What's the v4 Agent config normalization?"
→ See: DRONE_CONFIG_FLOW.md, section "3.2 Config Normalization in v4 Agent"

## Key Findings Summary

### 1. Configuration is NOT computed, it's TRANSMITTED
- UI state stores raw configs
- Configs sent AS-IS to backend
- Backend applies defaults, not configuration

### 2. start_airport is overridden in 5 places
1. UI: Auto-sync when airport deleted (line 1010)
2. UI: Default from dropdown (line 1094)
3. Solver: Default A{did} or first (line 589)
4. v4: Normalization → home_airport (line 1312)
5. v4: Route optimizer reads home_airport (line 852)

### 3. end_airport has complex logic
- UI: Default to start if deleted (line 1020)
- UI: Dropdown supports "-" flexible option (line 993)
- Solver: Tries ALL airports if "-" (lines 694-726)
- v4: Preserves if present, else defaults to home (line 853)

### 4. Mode is COMPUTED, never configured
- Always "return" if start==end
- Always "open" if start!=end
- Never read from config dictionary

### 5. Two caching layers exist
- **Distance Matrix**: Environment-keyed, hash-based invalidation
- **Mission State**: Per-mission_id in MISSION_STORE, in-memory only

### 6. v4 Agent has a normalization layer
- Maps UI field names (snake_case) → v4 field names (camelCase)
- start_airport → home_airport
- target_access {a,b,c,d,e} → accessible_targets ["A","B",...]

### 7. Flexible endpoints (ONLY in solver, NOT in v4)
- end_airport = "-" means solver tries all airports
- Picks best result by points (then distance)
- v4 Agent does NOT support this feature

## Testing Your Changes

### Before making changes to configuration flow:
1. Read ANALYSIS_SUMMARY.txt (5 minutes)
2. Skim DRONE_CONFIG_FLOW.md section relevant to your change
3. Use DRONE_CONFIG_QUICK_REF.md to find exact line numbers
4. Set breakpoints at key override locations
5. Use debug outputs mentioned in QUICK_REF.md to monitor values

### After making changes:
1. Test all 5 start_airport override locations
2. Test all 7 end_airport default locations
3. Verify mode computation (should still default to "return")
4. Check distance matrix cache invalidation
5. Verify mission_id persistence for multi-turn conversations
6. Use testing checklist in QUICK_REF.md

## File Locations (Absolute Paths)

```
/Users/kamalali/isr-planner/
├── webapp/isr.js                                  # UI state & config
├── server/main.py                                 # API endpoints
├── server/solver/solver_bridge.py                 # Solver core
├── server/solver/sam_distance_matrix.py           # Distance matrix cache
├── server/agents/isr_agent_multi_v4.py            # v4 Agent
├── ANALYSIS_SUMMARY.txt                           # This analysis (summary)
├── DRONE_CONFIG_FLOW.md                           # This analysis (detailed)
└── DRONE_CONFIG_QUICK_REF.md                      # This analysis (quick ref)
```

## How to Use This Analysis

### Scenario 1: "I'm debugging a routing issue"
1. Open DRONE_CONFIG_QUICK_REF.md
2. Find your specific issue in the "Edge Cases" section
3. Use "All start_airport Overrides" or "All end_airport Defaults" tables
4. Navigate to exact line numbers listed
5. Set breakpoints and monitor debug outputs

### Scenario 2: "I need to add a new config field"
1. Read DRONE_CONFIG_FLOW.md section "1.2 Config Structure"
2. Identify all 5 locations where start_airport is handled
3. Add equivalent handling for your new field at each location
4. Use QUICK_REF.md "Testing Checklist" to verify

### Scenario 3: "Why is my config value being ignored?"
1. Open QUICK_REF.md tables for your config field
2. Check if it's being OVERRIDDEN at any location
3. Verify DEFAULTS are being applied
4. Check if v4 Agent NORMALIZES the field name
5. Read "Caveats & Edge Cases" in FLOW.md

### Scenario 4: "I need to understand the v4 Agent flow"
1. Read FLOW.md section "3.2 Config Normalization in v4 Agent"
2. Review section "3.3 Route Optimizer Node in v4"
3. Check QUICK_REF.md "Config Normalization Mapping" table
4. Look for debug output at line 904 of isr_agent_multi_v4.py

## Analysis Metadata

- **Analysis Date**: December 11, 2024
- **Project**: ISR Planner
- **Scope**: Drone configuration flow (UI → Solver v4 → Agent v4)
- **Files Analyzed**: 7 core files (2,000+ lines total)
- **Locations Documented**: 
  - 5 start_airport overrides
  - 7 end_airport defaults
  - 3 mode defaults
  - 2 major caching layers
  - 8 primary flow paths

## Questions About This Analysis?

Refer to the specific document sections:

- **"What's the structure of state.droneConfigs?"** → FLOW.md 1.2
- **"How does run_planner() get drone configs?"** → FLOW.md 2.2
- **"Where does v4 agent normalize configs?"** → FLOW.md 3.2
- **"What are all the start_airport defaults?"** → QUICK_REF.md, start_airport table
- **"How does flexible endpoint work?"** → FLOW.md 6.2 or QUICK_REF.md edge cases
- **"Where is mission_id stored?"** → FLOW.md 8.1
- **"How does distance matrix cache invalidate?"** → FLOW.md 4.1 or QUICK_REF.md caching section

---

**All documents are stored in the project root:**
- `/Users/kamalali/isr-planner/ANALYSIS_SUMMARY.txt`
- `/Users/kamalali/isr-planner/DRONE_CONFIG_FLOW.md`
- `/Users/kamalali/isr-planner/DRONE_CONFIG_QUICK_REF.md`
- `/Users/kamalali/isr-planner/README_ANALYSIS.md` (this file)
