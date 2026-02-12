// isr.js  — single-file version with:
// - Edit mode ON/OFF
// - Canvas drawing + editing (airports/targets/SAMs)
// - Drone config + sequences
// - Stats
// - Import/Export
// - Planner integration

"use strict";

// VERSION CHECK - Fixed multiple trajectories after accept 2026-01-29
const ISR_CODE_VERSION = "2026-01-29-single-traj-fix";

// ----------------------------------------------------
// DOM helper
// ----------------------------------------------------
function $(id) {
  return document.getElementById(id);
}

// Add this:
function setText(id, text) {
  const el = $(id);
  if (!el) return;
  el.textContent = text == null ? "" : String(text);
}

// ----------------------------------------------------
// Global state
// ----------------------------------------------------
const state = {
  env: null,                 // environment JSON (airports, targets, sams)
  envFilename: null,         // name of JSON file

  sequences: {},             // { "1": "A1,T1,A1", ... }
  routes: {},                // { "1": {route, points, distance, fuel_budget}, ... }
  wrappedPolygons: [],       // wrapped SAM polygons from solver
  currentDroneForSeq: "1",

  droneConfigs: {},          // { "1": {enabled, fuel_budget, start_airport, end_airport, target_access}, ... }

  missionId: null,           // current v4 mission_id from the backend

  solveAbortController: null, // AbortController for canceling solver requests

  // Editing-related
  editMode: true,            // Edit: ON/OFF
  addMode: null,             // "airport" | "target" | "sam" | null
  selectedObject: null,      // { kind: "airport"|"target"|"sam", index: number } or null
  dragging: null,            // { kind, index, offsetX, offsetY } during drag

  // SAM wrapping debounce
  samWrappingTimeout: null,  // timeout ID for debounced wrapping updates
  wrappingAbortController: null,  // AbortController for canceling pending wrapping requests

  // Trajectory visibility
  trajectoryVisible: {
    "1": true,
    "2": true,
    "3": true,
    "4": true,
    "5": true,
  },

  // Animation state
  animation: {
    active: false,
    drones: {},              // { "1": { progress: 0, animating: false }, ... }
    animationId: null,       // requestAnimationFrame ID
  },

  // Checkpoint / segmented solve state
  checkpoint: {
    active: false,
    pct: 0.5,
    segments: {},   // per drone: { prefix, suffix, splitPoint, checkpointDist }
  },

  // Mission history for segmented replanning
  missionHistory: [],          // Array of segment snapshots
  initialEnvSnapshot: null,    // Immutable copy of original env for reset
  visitedTargets: [],          // Target IDs completed across all segments

  // Single cut distance captured during freezeAtCheckpoint (same for all drones)
  pendingCutDistance: null,

  // Target-to-segment mapping for visualization
  // { "T1": 1, "T2": 1, "T3": 2, ... } - which segment each target belongs to
  targetSegmentMap: {},

  // Current segment number for dynamic cut-based segment assignment
  // Starts at 1, increments with each Cut button press
  currentCutSegment: 1,

  // Track which targets were assigned in previous cuts (to avoid reassigning)
  previouslyAssignedTargets: [],

  // Per-drone cut positions captured during freezeAtCheckpoint
  // { [did]: [x,y] } - used when committing segments
  pendingCutPositions: null,

  // Checkpoints for the current cut - per drone (e.g., { "1": {id: "C1-1", x: 50, y: 30}, "2": {id: "C1-2", x: 65, y: 45} })
  // Named as C{segment}-{drone} (e.g., C1-1 = Checkpoint 1 for Drone 1)
  pendingCheckpoints: null,

  // Counter for checkpoint segment number (C1, C2, C3, ...)
  checkpointSegmentCounter: 0,

  // Drones lost at the current checkpoint (disabled after cut but before solve)
  // Array of drone IDs: ["1", "3"] - used when drawing lost drone markers
  pendingLostDrones: [],

  // Legacy single cut position (backward compatibility)
  pendingCutPosition: null,

  // Agent trace runs for Agents Trace tab
  agentTraceRuns: [],  // Array of { ts, user_message, mission_id, trace_events }
};

// ----------------------------------------------------
// MissionSegments - Unified segment manager (replaces segmentStore, MissionReplayLegacy, SegmentedImportManager)
// ----------------------------------------------------
/**
 * MissionSegments - Single source of truth for segmented missions
 *
 * Handles both CREATION workflow (building segments via Cut) and IMPORT workflow (loading pre-segmented JSON).
 *
 * Each segment contains:
 * - index: segment number (0, 1, 2, ...)
 * - env: { airports, targets, sams } for this segment
 * - drone_configs: { "1": { enabled, fuel_capacity, ... }, ... }
 * - routes: { "1": { trajectory, route, fuel_remaining }, ... }
 * - frozenTargets: ["T1", "T2"] - targets visited BEFORE this segment
 * - cutPosition: { "1": [x,y], "2": [x,y] } - cut position (null for segment 0)
 */
class MissionSegments {
  constructor() {
    this.clear();
  }

  /** Clear all state */
  clear() {
    this._segments = [];
    this._currentIndex = 0;
    this._frozenTrajectories = {};  // { droneId: [[x,y], ...] } - accumulated frozen paths
    this._isImported = false;       // True if loaded from JSON (vs created via workflow)
  }

  /** Check if we have any segments */
  hasSegments() {
    return this._segments.length > 0;
  }

  /** Check if this is an imported mission */
  isImported() {
    return this._isImported;
  }

  /** Get current segment index */
  getCurrentIndex() {
    return this._currentIndex;
  }

  /** Get total segment count */
  getSegmentCount() {
    return this._segments.length;
  }

  /** Get segment by index */
  getSegment(index) {
    return this._segments[index] || null;
  }

  /** Get current segment */
  getCurrentSegment() {
    return this._segments[this._currentIndex] || null;
  }

  /** Get all segments */
  getAllSegments() {
    return this._segments;
  }

  // ========================================
  // CREATION WORKFLOW
  // ========================================

  /**
   * Add a new segment (called on Accept during creation workflow)
   * @param {object} segmentData - { env, drone_configs, routes, frozenTargets, cutPosition }
   * @returns {number} The new segment's index
   */
  addSegment(segmentData) {
    const segment = {
      index: this._segments.length,
      env: JSON.parse(JSON.stringify(segmentData.env || {})),
      drone_configs: JSON.parse(JSON.stringify(segmentData.drone_configs || {})),
      routes: this._copyRoutes(segmentData.routes || {}),
      frozenTargets: segmentData.frozenTargets ? [...segmentData.frozenTargets] : [],
      cutPosition: segmentData.cutPosition ? JSON.parse(JSON.stringify(segmentData.cutPosition)) : null
    };
    this._segments.push(segment);
    this._currentIndex = segment.index;
    return segment.index;
  }

  /**
   * Update the current segment's routes (called after optimization)
   */
  updateCurrentRoutes(routes) {
    const seg = this.getCurrentSegment();
    if (seg) {
      seg.routes = this._copyRoutes(routes);
    }
  }

  /**
   * Set frozen trajectory for a drone (accumulated from previous segments)
   * @param {string} droneId
   * @param {Array} points - [[x,y], [x,y], ...]
   */
  setFrozenTrajectory(droneId, points) {
    this._frozenTrajectories[droneId] = points ? [...points] : [];
  }

  /**
   * Get frozen trajectory for a drone
   * @param {string} droneId
   * @returns {Array} [[x,y], ...] or empty array
   */
  getFrozenTrajectory(droneId) {
    return this._frozenTrajectories[droneId] || [];
  }

  /**
   * Get all frozen trajectories
   * @returns {object} { droneId: [[x,y], ...], ... }
   */
  getAllFrozenTrajectories() {
    return this._frozenTrajectories;
  }

  /**
   * Clear frozen trajectories (called on Reset)
   */
  clearFrozenTrajectories() {
    this._frozenTrajectories = {};
  }

  // ========================================
  // IMPORT WORKFLOW
  // ========================================

  /**
   * Load segments from JSON (import workflow)
   * @param {object} data - { segments: [...], version: "2.0", ... }
   * @returns {boolean} Success
   */
  loadFromJson(data) {
    this.clear();

    const segments = data.segments || [];
    if (segments.length === 0) return false;

    segments.forEach((seg, idx) => {
      this._segments.push({
        index: idx,
        env: JSON.parse(JSON.stringify(seg.env || {})),
        drone_configs: JSON.parse(JSON.stringify(seg.drone_configs || {})),
        routes: this._copyRoutes(seg.routes || {}),
        frozenTargets: seg.frozenTargets ? [...seg.frozenTargets] : [],
        cutPosition: seg.cutPosition ? JSON.parse(JSON.stringify(seg.cutPosition)) : null
      });
    });

    this._currentIndex = 0;
    this._isImported = true;
    return true;
  }

  /**
   * Advance to next segment (called on Accept during import workflow)
   * @returns {boolean} True if advanced, false if at last segment
   */
  advanceToNextSegment() {
    if (this._currentIndex < this._segments.length - 1) {
      this._currentIndex++;
      return true;
    }
    return false;
  }

  /**
   * Check if at last segment
   */
  isLastSegment() {
    return this._currentIndex >= this._segments.length - 1;
  }

  /**
   * Reset to first segment (for animation replay)
   */
  resetToStart() {
    this._currentIndex = 0;
    this._frozenTrajectories = {};
  }

  // ========================================
  // EXPORT
  // ========================================

  /**
   * Export to JSON format
   * @returns {object} { version: "2.0", exportedAt: timestamp, segments: [...] }
   */
  toJSON() {
    return {
      version: "2.0",
      exportedAt: Date.now(),
      segments: this._segments.map(seg => ({
        index: seg.index,
        env: seg.env,
        drone_configs: seg.drone_configs,
        routes: seg.routes,
        frozenTargets: seg.frozenTargets,
        cutPosition: seg.cutPosition
      }))
    };
  }

  // ========================================
  // HELPERS
  // ========================================

  /** Deep copy routes object */
  _copyRoutes(routes) {
    const copy = {};
    Object.entries(routes).forEach(([droneId, routeData]) => {
      copy[droneId] = {
        trajectory: routeData.trajectory ? routeData.trajectory.map(p => [...p]) : [],
        route: routeData.route ? [...routeData.route] : [],
        fuel_remaining: routeData.fuel_remaining || 0
      };
    });
    return copy;
  }

  /** Debug: print segment table */
  printSegmentTable() {
    const lines = [];
    lines.push(`=== SEGMENTS (${this._segments.length} total, current=${this._currentIndex}) ===`);
    lines.push(`Idx | targets | routes | frozen | cutPos`);
    lines.push(`----+---------+--------+--------+--------`);
    this._segments.forEach(seg => {
      const targetsStr = String(seg.env?.targets?.length ?? 0).padStart(7);
      const routesStr = String(Object.keys(seg.routes || {}).filter(d => seg.routes[d]?.trajectory?.length > 0).length).padStart(6);
      const frozenStr = String(seg.frozenTargets?.length ?? 0).padStart(6);
      const cutStr = seg.cutPosition ? 'yes' : 'no';
      lines.push(`${String(seg.index).padStart(3)} | ${targetsStr} | ${routesStr} | ${frozenStr} | ${cutStr.padStart(6)}`);
    });
    lines.push(`===`);
    const table = lines.join('\n');
    console.log(table);
    return table;
  }
}

// Global mission segments instance
const mission = new MissionSegments();

// Legacy alias for backward compatibility during transition
const segmentStore = {
  segments: [],
  clear() { mission.clear(); this.segments = []; },
  addSegment(seg) {
    const idx = mission.addSegment(seg);
    this.segments = mission.getAllSegments();
    return idx;
  },
  getSegment(idx) { return mission.getSegment(idx); },
  getCurrentSegment() { return mission.getCurrentSegment(); },
  getAllSegments() { return mission.getAllSegments(); },
  count() { return mission.getSegmentCount(); },
  toJSON() { return mission.toJSON(); },
  updateLastSegment(updates) {
    const seg = mission.getCurrentSegment();
    if (seg) Object.assign(seg, updates);
  }
};

/**
 * Build a clean segment object for mission.addSegment()
 * Called from acceptSolution() to save segment data
 */
function buildCleanSegment(solutionToStore, droneConfigs, env, frozenTargets, cutPositions, checkpoint = null) {
  // Build routes object with just trajectory, route, fuel_remaining
  const routes = {};
  Object.entries(solutionToStore.routes || {}).forEach(([droneId, routeData]) => {
    routes[droneId] = {
      trajectory: routeData.trajectory ? [...routeData.trajectory] : [],
      route: routeData.route ? [...routeData.route] : [],
      fuel_remaining: routeData.fuel_remaining || 0
    };
  });

  return {
    env: {
      airports: JSON.parse(JSON.stringify(env.airports || [])),
      targets: JSON.parse(JSON.stringify(env.targets || [])),
      sams: JSON.parse(JSON.stringify(env.sams || [])),
      checkpoints: JSON.parse(JSON.stringify(env.checkpoints || []))
    },
    drone_configs: JSON.parse(JSON.stringify(droneConfigs)),
    routes: routes,
    frozenTargets: frozenTargets ? [...frozenTargets] : [],
    cutPosition: cutPositions ? JSON.parse(JSON.stringify(cutPositions)) : null,
    checkpoint: checkpoint ? JSON.parse(JSON.stringify(checkpoint)) : null  // The checkpoint ID/position for this segment
  };
}

// ----------------------------------------------------
// Mission Mode State Machine
// ----------------------------------------------------
/**
 * MissionMode - Single source of truth for UI state
 *
 * IDLE:                 No committed plan exists yet. Can edit env or solve.
 * EDITING_ENV:          User is editing draftEnv. Must Accept Edits to proceed.
 * DRAFT_READY:          A draft solution exists. Must Accept Solution or Discard.
 * READY_TO_ANIMATE:     Committed plan exists. Can start/resume animation.
 * ANIMATING:            Drones are moving. Can Pause or Cut (C).
 * PAUSED_MID_ANIMATION: Paused but resumable (no truncation). Can resume or edit.
 * CHECKPOINT:           Cut occurred. Past locked, future removed. Must replan.
 */
const MissionMode = {
  IDLE: "IDLE",
  EDITING_ENV: "EDITING_ENV",
  DRAFT_READY: "DRAFT_READY",
  READY_TO_ANIMATE: "READY_TO_ANIMATE",
  ANIMATING: "ANIMATING",
  PAUSED_MID_ANIMATION: "PAUSED_MID_ANIMATION",
  CHECKPOINT: "CHECKPOINT",
};

// Mission state machine state
const missionState = {
  mode: MissionMode.IDLE,

  // Environment state (two-phase commit)
  draftEnv: null,              // Editable environment shown in editor
  acceptedEnv: null,           // Committed environment used for solving

  // Solution state (two-phase commit)
  draftSolution: null,         // Last solver output (not yet accepted)
  committedSegments: [],       // Array of accepted solutions (mission history)
  currentSegmentIndex: -1,     // Index of current segment being animated (-1 = none)

  // Runtime state for animation resume
  pauseContext: null,          // { droneStates: {...} } - saved state for resume

  // Checkpoint source tracking - distinguishes replay cuts from manual replan
  // "replay_cut" = checkpoint entered from animation cut (solve disabled)
  // null = checkpoint entered from edits/manual (solve enabled)
  checkpointSource: null,

  // Segmented mission workflow - for solving N segments sequentially
  segmentedMission: null,      // Full segmented mission definition from import
  currentBuildSegmentIndex: 0, // Which segment we're currently solving (0-indexed)
};

// ----------------------------------------------------
// Legacy Segment class - now delegates to mission system
// ----------------------------------------------------

/**
 * Segment - Legacy wrapper for backward compatibility
 * Creates segment-like objects that work with existing code
 */
function createSegmentObject(data) {
  const seg = {
    index: data.index || 0,
    solution: data.solution ? JSON.parse(JSON.stringify(data.solution)) : { routes: {}, sequences: {} },
    env: data.env ? JSON.parse(JSON.stringify(data.env)) : {},
    cutDistance: data.cutDistance ?? null,
    isCheckpointReplan: data.isCheckpointReplan || false,
    timestamp: data.timestamp || new Date().toISOString(),
    cutPosition: data.cutPosition ? [...data.cutPosition] : null,
    cutPositions: data.cutPositions ? JSON.parse(JSON.stringify(data.cutPositions)) : null,
    lostDrones: data.lostDrones ? [...data.lostDrones] : null,
    visitedTargets: data.visitedTargets ? [...data.visitedTargets] : null,
    visited_targets: data.visitedTargets ? [...data.visitedTargets] : null,  // Alias
    drone_configs: data.drone_configs ? JSON.parse(JSON.stringify(data.drone_configs)) : null,

    getCutDistance() {
      return this.cutDistance;
    },
    hasCheckpointBoundary() {
      return this.cutDistance !== null && this.cutDistance > 0;
    }
  };
  return seg;
}

/**
 * MissionReplay - Manages multi-segment mission playback
 *
 * Responsibilities:
 * - Store mission segments in sequence
 * - Track current playback position (segment index + distance)
 * - Detect segment boundary crossings
 * - Provide segment data for animation and display
 *
 * Does NOT:
 * - Control animation timing (that's the animation loop's job)
 * - Modify UI directly (emits events/callbacks instead)
 */
class MissionReplayLegacy {
  constructor() {
    this._segments = [];
    this._currentSegmentIndex = 0;
    this._onSegmentSwitch = null;  // Callback: (fromIndex, toIndex, newSegment) => void
  }

  // --- Segment Management ---

  /**
   * Add a new segment to the mission
   */
  addSegment(segmentData) {
    const segment = createSegmentObject({
      index: this._segments.length,
      ...segmentData
    });
    this._segments.push(segment);
    return segment;
  }

  /**
   * Get all segments
   */
  getSegments() {
    return [...this._segments];
  }

  /**
   * Get segment by index
   */
  getSegment(index) {
    return this._segments[index] ?? null;
  }

  /**
   * Get current segment
   */
  getCurrentSegment() {
    return this._segments[this._currentSegmentIndex] ?? null;
  }

  /**
   * Get next segment (if any)
   */
  getNextSegment() {
    return this._segments[this._currentSegmentIndex + 1] ?? null;
  }

  /**
   * Get segment count
   */
  getSegmentCount() {
    return this._segments.length;
  }

  /**
   * Get current segment index
   */
  getCurrentSegmentIndex() {
    return this._currentSegmentIndex;
  }

  /**
   * Check if there are more segments after current
   */
  hasNextSegment() {
    return this._currentSegmentIndex < this._segments.length - 1;
  }

  // --- Playback Control ---

  /**
   * Set current segment index (for advancing after accept)
   */
  setCurrentSegmentIndex(index) {
    if (index >= 0 && index < this._segments.length) {
      this._currentSegmentIndex = index;
    }
  }

  /**
   * Reset to beginning for replay
   */
  resetToStart() {
    this._currentSegmentIndex = 0;
  }

  /**
   * Clear all solutions from all segments (for Reset in segmentInfo workflow)
   * Keeps env, cutDistance, cutPositions intact
   */
  clearAllSolutions() {
    for (const seg of this._segments) {
      seg.solution = { routes: {}, sequences: {} };
    }
  }

  /**
   * Clear all segments (full reset)
   */
  clear() {
    this._segments = [];
    this._currentSegmentIndex = 0;
  }

  /**
   * Keep only first N segments
   */
  truncateAfter(index) {
    if (index < this._segments.length - 1) {
      this._segments = this._segments.slice(0, index + 1);
      if (this._currentSegmentIndex > index) {
        this._currentSegmentIndex = index;
      }
    }
  }

  /**
   * Replace a segment at a given index with new data
   * Creates a new segment object with the provided data
   */
  replaceSegment(index, segmentData) {
    if (index < 0 || index >= this._segments.length) {
      return false;
    }
    const newSegment = createSegmentObject({
      index: index,
      ...segmentData
    });
    this._segments[index] = newSegment;
    return true;
  }

  /**
   * Update the solution in the current segment (e.g., after optimization)
   */
  updateCurrentSegmentSolution(routes, sequences) {
    const segment = this.getCurrentSegment();
    if (!segment) {
      return false;
    }
    // Update the solution's routes and sequences
    if (segment.solution) {
      segment.solution.routes = JSON.parse(JSON.stringify(routes));
      if (sequences) {
        segment.solution.sequences = JSON.parse(JSON.stringify(sequences));
      }
      return true;
    }
    return false;
  }

  /**
   * Set callback for segment switches
   */
  onSegmentSwitch(callback) {
    this._onSegmentSwitch = callback;
  }

  /**
   * Check if mission has crossed into the next segment
   * Call this from animation loop with current mission distance (fuel consumed)
   *
   * @param missionDistance - Current distance/fuel traveled (same for all drones)
   * @returns {switched: boolean, newSegment: Segment|null}
   */
  checkSegmentBoundary(missionDistance) {
    const nextSegment = this.getNextSegment();
    if (!nextSegment || !nextSegment.hasCheckpointBoundary()) {
      return { switched: false, newSegment: null };
    }

    const cutDistance = nextSegment.getCutDistance();
    if (cutDistance !== null && missionDistance >= cutDistance) {
      // Crossed! Switch to next segment
      const fromIndex = this._currentSegmentIndex;
      this._currentSegmentIndex++;
      if (this._onSegmentSwitch) {
        this._onSegmentSwitch(fromIndex, this._currentSegmentIndex, nextSegment);
      }

      return { switched: true, newSegment: nextSegment };
    }

    return { switched: false, newSegment: null };
  }

  /**
   * Get the starting distance for the current segment
   * (0 for segment 0, cutDistance for checkpoint replans)
   */
  getStartingDistance() {
    const segment = this.getCurrentSegment();
    if (!segment || segment.index === 0) {
      return 0;
    }
    return segment.getCutDistance() ?? 0;
  }

  /**
   * Export replay state for debugging
   */
  getDebugInfo() {
    return {
      segmentCount: this._segments.length,
      currentIndex: this._currentSegmentIndex,
      segments: this._segments.map(s => ({
        index: s.index,
        isCheckpointReplan: s.isCheckpointReplan,
        hasCheckpoint: s.hasCheckpointBoundary(),
        cutDistance: s.getCutDistance(),
        envTargets: s.env.targets?.length ?? 0,
        routeCount: Object.keys(s.solution.routes || {}).length,
        lostDrones: s.lostDrones || [],
        visitedTargets: s.visited_targets || []
      }))
    };
  }

  /**
   * Print segment table to console and debug panel for debugging
   */
  printSegmentTable() {
    const lines = [];
    lines.push(`=== SEGMENT TABLE (${this._segments.length} segments, current=${this._currentSegmentIndex}) ===`);
    lines.push(`Idx | cutDist | targets | routes | lostDrones | visited`);
    lines.push(`----+---------+---------+--------+------------+---------`);
    this._segments.forEach(s => {
      const cutDist = s.getCutDistance();
      const cutStr = cutDist !== null ? cutDist.toFixed(1).padStart(7) : '  null ';
      const targetsStr = String(s.env?.targets?.length ?? 0).padStart(7);
      const routesStr = String(Object.keys(s.solution?.routes || {}).length).padStart(6);
      const lostStr = (s.lostDrones || []).join(',') || '-';
      const visitedStr = (s.visited_targets || []).join(',') || '-';
      lines.push(`${String(s.index).padStart(3)} | ${cutStr} | ${targetsStr} | ${routesStr} | ${lostStr.padEnd(10)} | ${visitedStr}`);
    });
    lines.push(`===`);

    const table = lines.join('\n');
    console.log(table);
    return table;
  }
}

// Global mission replay instance (legacy system)
const missionReplay = new MissionReplayLegacy();


/**
 * SegmentedImportManager - Manages state for imported segmented missions
 *
 * This class is the SINGLE SOURCE OF TRUTH for:
 * - All targets in the mission (never changes after load)
 * - Which targets have been visited/frozen (updated on Accept)
 * - Current segment index
 * - Cut positions for each segment
 *
 * The key insight: state.env.targets should contain ALL targets for DRAWING.
 * Filtering for the solver happens in buildCheckpointEnv().
 * The drawing code uses state.visitedTargets to show green X on frozen targets.
 */
class SegmentedImportManager {
  constructor() {
    this.clear();
  }

  /** Clear all segmented import state */
  clear() {
    this._allTargets = [];           // ALL targets - never modified after load
    this._segmentTargets = [];       // Targets per segment: [[seg0 targets], [seg1 targets], ...]
    this._baseEnv = null;            // Base environment (airports, sams, drone_configs)
    this._segmentCuts = [];          // Array of {cutDistance, visitedTargets, dronePositions}
    this._currentSegmentIndex = 0;   // Which segment we're solving
    this._isActive = false;          // Is this a segmented import workflow?
    this._segmentSolutions = [];     // Solutions per segment (if present in JSON)
  }

  /** Check if we're in segmented import mode */
  isActive() {
    return this._isActive;
  }

  /** Get current segment index */
  getCurrentSegmentIndex() {
    return this._currentSegmentIndex;
  }

  /** Get all targets (for drawing) */
  getAllTargets() {
    return JSON.parse(JSON.stringify(this._allTargets));
  }

  /** Get visited targets for current segment (from cut data) */
  getVisitedTargetsForSegment(segmentIndex) {
    // Segment 0: no targets visited yet
    if (segmentIndex === 0) {
      return [];
    }
    // Segment N: use visitedTargets from cut[N-1]
    const cutIdx = segmentIndex - 1;
    if (cutIdx >= 0 && cutIdx < this._segmentCuts.length) {
      return [...(this._segmentCuts[cutIdx].visitedTargets || [])];
    }
    return [];
  }

  /** Get cut position for segment (where drone starts) */
  getCutPositionForSegment(segmentIndex) {
    // Segment 0: starts at airport, no cut position
    if (segmentIndex === 0) {
      return null;
    }
    // Segment N's cutPosition = where segment N starts (C{N} marker)
    // This is stored directly in _segmentCutPositions[N] as {"1": [x,y], "2": [x,y]}
    if (this._segmentCutPositions && segmentIndex < this._segmentCutPositions.length) {
      const rawCutPos = this._segmentCutPositions[segmentIndex];
      if (rawCutPos) {
        console.log(`[getCutPositionForSegment] Seg ${segmentIndex} returning:`, rawCutPos);
        return JSON.parse(JSON.stringify(rawCutPos));
      }
    }
    // Fallback to old _segmentCuts method
    const cutIdx = segmentIndex - 1;
    if (cutIdx >= 0 && cutIdx < this._segmentCuts.length) {
      const cut = this._segmentCuts[cutIdx];
      if (cut.dronePositions) {
        const positions = {};
        Object.entries(cut.dronePositions).forEach(([did, posData]) => {
          if (posData.position && posData.position.length === 2) {
            positions[did] = [...posData.position];
          }
        });
        if (Object.keys(positions).length > 0) {
          console.log(`[getCutPositionForSegment] Seg ${segmentIndex} fallback returning:`, positions);
          return positions;
        }
      }
    }
    console.log(`[getCutPositionForSegment] Seg ${segmentIndex} no data found`);
    return null;
  }

  /** Get cut distance for segment */
  getCutDistanceForSegment(segmentIndex) {
    if (segmentIndex === 0) return 0;
    const cutIdx = segmentIndex - 1;
    if (cutIdx >= 0 && cutIdx < this._segmentCuts.length) {
      // Return actual cutDistance (may be null for last segment)
      return this._segmentCuts[cutIdx].cutDistance;
    }
    return null;  // No cut data available
  }

  /** Get drone_configs for a specific segment (from the imported JSON) */
  getDroneConfigsForSegment(segmentIndex) {
    if (!this._segmentDroneConfigs || segmentIndex < 0 || segmentIndex >= this._segmentDroneConfigs.length) {
      return null;
    }
    return JSON.parse(JSON.stringify(this._segmentDroneConfigs[segmentIndex]));
  }

  /** Get solution for a specific segment (from the imported JSON, if present) */
  getSolutionForSegment(segmentIndex) {
    if (!this._segmentSolutions || segmentIndex < 0 || segmentIndex >= this._segmentSolutions.length) {
      return null;
    }
    return this._segmentSolutions[segmentIndex] ? JSON.parse(JSON.stringify(this._segmentSolutions[segmentIndex])) : null;
  }

  /** Check if imported JSON has solutions (for replay vs re-solve) */
  hasSolutions() {
    return this._segmentSolutions && this._segmentSolutions.some(s => s && Object.keys(s.routes || {}).length > 0);
  }

  /** Get total number of segments */
  getTotalSegments() {
    return this._segmentCuts.length + 1;
  }

  /** Advance to next segment */
  advanceToNextSegment() {
    if (this._currentSegmentIndex < this._segmentCuts.length) {
      this._currentSegmentIndex++;
      return true;
    }
    return false;
  }

  /** Check if all segments are complete */
  isComplete() {
    return this._currentSegmentIndex >= this._segmentCuts.length;
  }

  /** Get unfrozen targets for solver (current segment's targets minus visited) */
  getUnfrozenTargets() {
    const visited = this.getVisitedTargetsForSegment(this._currentSegmentIndex);
    const segmentTargets = this.getTargetsToDisplay();  // Current segment's targets
    return segmentTargets.filter(t => !visited.includes(t.id));
  }

  /** Get targets to DISPLAY for current segment */
  getTargetsToDisplay() {
    // Return CUMULATIVE targets from segment 0 through current segment
    // This ensures progressive reveal: each Accept shows all previous + new targets
    const seenTargets = new Map();

    for (let i = 0; i <= this._currentSegmentIndex && i < this._segmentTargets.length; i++) {
      const segTargets = this._segmentTargets[i] || [];
      segTargets.forEach(t => {
        if (!seenTargets.has(t.id)) {
          seenTargets.set(t.id, t);
        }
      });
    }

    return Array.from(seenTargets.values()).map(t => JSON.parse(JSON.stringify(t)));
  }

  /** Get env for DISPLAY (only targets up to current segment) */
  getEnvForDisplay() {
    if (!this._baseEnv) return null;
    const env = JSON.parse(JSON.stringify(this._baseEnv));
    env.targets = this.getTargetsToDisplay();
    return env;
  }

  /** Get base env with all targets (for drawing) */
  getFullEnv() {
    if (!this._baseEnv) return null;
    const env = JSON.parse(JSON.stringify(this._baseEnv));
    env.targets = this.getAllTargets();
    return env;
  }

  /** Get env for solver (filtered targets, synthetic starts for segment > 0) */
  getEnvForSolver() {
    if (!this._baseEnv) return null;
    const env = JSON.parse(JSON.stringify(this._baseEnv));
    env.targets = this.getUnfrozenTargets();

    // Merge state.droneConfigs to preserve UI-configured values (end_airport, fuel_budget, enabled)
    // This is critical because _baseEnv.drone_configs comes from the imported JSON,
    // but the user may have changed these values in the UI
    const droneConfigs = JSON.parse(JSON.stringify(env.drone_configs || {}));
    Object.entries(state.droneConfigs || {}).forEach(([did, cfg]) => {
      if (droneConfigs[did]) {
        // Preserve end_airport from UI config
        if (cfg.end_airport) droneConfigs[did].end_airport = cfg.end_airport;
        // Preserve fuel_budget from UI config
        if (cfg.fuel_budget !== undefined) droneConfigs[did].fuel_budget = cfg.fuel_budget;
        // Preserve enabled from UI config
        if (cfg.enabled !== undefined) droneConfigs[did].enabled = cfg.enabled;
        // Preserve target_access from UI config
        if (cfg.target_access) droneConfigs[did].target_access = cfg.target_access;
      }
    });
    env.drone_configs = droneConfigs;

    // For segment > 0, add synthetic start at cut position
    if (this._currentSegmentIndex > 0) {
      const cutPositions = this.getCutPositionForSegment(this._currentSegmentIndex);
      if (cutPositions) {
        env.synthetic_starts = env.synthetic_starts || {};

        Object.entries(cutPositions).forEach(([did, pos]) => {
          const nodeId = `D${did}_START`;
          env.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };
          if (env.drone_configs[did]) {
            env.drone_configs[did].start_airport = nodeId;
          }
        });
      }
    }
    return env;
  }

  /**
   * Load from segmentInfo format JSON
   * @param {Object} data - JSON with type:"segmented", targets, segmentInfo.segmentCuts
   */
  loadFromSegmentInfo(data) {
    this.clear();

    // Store ALL targets (never modify this)
    this._allTargets = JSON.parse(JSON.stringify(data.targets || []));

    // Store base environment (without targets - we'll add them dynamically)
    this._baseEnv = {
      airports: JSON.parse(JSON.stringify(data.airports || [])),
      sams: JSON.parse(JSON.stringify(data.sams || [])),
      drone_configs: JSON.parse(JSON.stringify(data.drone_configs || {})),
    };

    // Store segment cuts
    const segmentCuts = data.segmentInfo?.segmentCuts || [];
    this._segmentCuts = segmentCuts.map(cut => {
      const dronePositions = cut.dronePositions || {};
      const firstDronePos = Object.values(dronePositions)[0];
      return {
        cutDistance: firstDronePos?.distanceTraveled || firstDronePos?.totalDistance || 0,
        visitedTargets: [...(cut.visitedTargets || [])],
        dronePositions: JSON.parse(JSON.stringify(dronePositions)),
      };
    });

    this._currentSegmentIndex = 0;
    this._isActive = true;

    return true;
  }

  /**
   * Load from segments array format (legacy format with segments[])
   * This format has segments[0].env, segments[1].env, etc.
   * We need to collect all targets and build cut data from the segments.
   */
  loadFromSegmentsArray(data) {
    this.clear();

    const segments = data.segments || [];
    if (segments.length === 0) return false;

    // Collect ALL targets from all segments
    const seenTargetIds = new Set();
    const allTargets = [];

    // First check if top-level targets exist
    if (data.targets && data.targets.length > 0) {
      data.targets.forEach(t => {
        if (!seenTargetIds.has(t.id)) {
          seenTargetIds.add(t.id);
          allTargets.push(JSON.parse(JSON.stringify(t)));
        }
      });
    }

    // Collect targets from each segment AND store per-segment targets
    this._segmentTargets = [];
    segments.forEach((seg, idx) => {
      const segTargets = [];
      (seg.env?.targets || []).forEach(t => {
        segTargets.push(JSON.parse(JSON.stringify(t)));
        if (!seenTargetIds.has(t.id)) {
          seenTargetIds.add(t.id);
          allTargets.push(JSON.parse(JSON.stringify(t)));
        }
      });
      this._segmentTargets.push(segTargets);
    });

    this._allTargets = allTargets;

    // Build base env from segment 0
    const seg0Env = segments[0]?.env || {};
    // IMPORTANT: Check for top-level drone_configs first (new export format),
    // then segment 0's drone_configs, then seg0Env.drone_configs
    const droneConfigs = data.drone_configs ||
                         segments[0]?.drone_configs ||
                         seg0Env.drone_configs ||
                         {};
    this._baseEnv = {
      airports: JSON.parse(JSON.stringify(seg0Env.airports || [])),
      sams: JSON.parse(JSON.stringify(seg0Env.sams || [])),
      drone_configs: JSON.parse(JSON.stringify(droneConfigs)),
    };
    // Store per-segment drone_configs for retrieval during Accept
    this._segmentDroneConfigs = [];
    this._segmentSolutions = [];
    this._segmentCutPositions = [];  // Store raw cutPosition from each segment
    segments.forEach((seg, idx) => {
      const segConfigs = seg.drone_configs || seg.env?.drone_configs || {};
      this._segmentDroneConfigs.push(JSON.parse(JSON.stringify(segConfigs)));
      const enabled = Object.entries(segConfigs).filter(([,c]) => c.enabled).map(([d]) => `D${d}`).join(',');
      // Store solution if present (for replay without re-solve)
      // Check both seg.solution (legacy format) and seg.routes (V2 export format)
      let solution = seg.solution || null;
      if (!solution && seg.routes) {
        // Convert V2 export format (seg.routes) to expected solution format
        solution = { routes: seg.routes };
      }
      this._segmentSolutions.push(solution ? JSON.parse(JSON.stringify(solution)) : null);
      // Store raw cutPosition directly from segment (seg.cutPosition has format {"1": [x,y], "2": [x,y]})
      const rawCutPos = seg.cutPosition || seg.cutPositions || null;
      this._segmentCutPositions.push(rawCutPos ? JSON.parse(JSON.stringify(rawCutPos)) : null);
      if (rawCutPos) {
        console.log(`[loadFromSegmentsArray] Seg ${idx} cutPosition stored:`, rawCutPos);
      }
    });

    // Check if segment 0 has segmentInfo with segmentCuts (hybrid format)
    const seg0SegmentInfo = segments[0]?.env?.segmentInfo;
    if (seg0SegmentInfo && Array.isArray(seg0SegmentInfo.segmentCuts) && seg0SegmentInfo.segmentCuts.length > 0) {
      // Use the segmentInfo.segmentCuts data directly (preferred source)
      this._segmentCuts = seg0SegmentInfo.segmentCuts.map(cut => {
        const dronePositions = cut.dronePositions || {};
        const firstDronePos = Object.values(dronePositions)[0];
        return {
          cutDistance: firstDronePos?.distanceTraveled || firstDronePos?.totalDistance || 0,
          visitedTargets: [...(cut.visitedTargets || [])],
          dronePositions: JSON.parse(JSON.stringify(dronePositions)),
        };
      });
    } else {
      // Build segment cuts from segments[1+]
      // Each segment after 0 represents a cut point
      this._segmentCuts = [];

      // Helper function to calculate trajectory length
      const calculateTrajectoryLength = (trajectory) => {
        if (!trajectory || trajectory.length < 2) return 0;
        let totalDist = 0;
        for (let j = 1; j < trajectory.length; j++) {
          const dx = trajectory[j][0] - trajectory[j - 1][0];
          const dy = trajectory[j][1] - trajectory[j - 1][1];
          totalDist += Math.sqrt(dx * dx + dy * dy);
        }
        return totalDist;
      };

      // Helper function to calculate distance along trajectory to a cut position
      const calculateDistanceToCutPosition = (trajectory, cutPos) => {
        if (!trajectory || trajectory.length < 2 || !cutPos) return null;

        // Find the segment where cutPos lies and calculate distance to that point
        let cumulativeDist = 0;
        for (let j = 0; j < trajectory.length - 1; j++) {
          const p1 = trajectory[j];
          const p2 = trajectory[j + 1];

          // Calculate segment vector
          const dx = p2[0] - p1[0];
          const dy = p2[1] - p1[1];
          const segLen = Math.sqrt(dx * dx + dy * dy);

          // Check if cutPos is on or near this segment
          // Project cutPos onto the line segment
          if (segLen > 0) {
            const t = ((cutPos[0] - p1[0]) * dx + (cutPos[1] - p1[1]) * dy) / (segLen * segLen);
            const clampedT = Math.max(0, Math.min(1, t));

            const closestX = p1[0] + clampedT * dx;
            const closestY = p1[1] + clampedT * dy;
            const distToCutPos = Math.sqrt((cutPos[0] - closestX) ** 2 + (cutPos[1] - closestY) ** 2);

            // If cutPos is close enough to this segment (within tolerance)
            if (distToCutPos < 1.0) {
              // Return cumulative distance to the projection point
              return cumulativeDist + clampedT * segLen;
            }
          }

          cumulativeDist += segLen;
        }

        // If cutPos wasn't found on any segment, return full trajectory length
        return cumulativeDist;
      };

      // Calculate cumulative distances for each segment
      // cutDistance for segment N is the distance traveled to reach the cut position for segment N
      let cumulativeDistance = 0;

      for (let i = 1; i < segments.length; i++) {
        const seg = segments[i];
        // Use explicit cutDistance if provided, otherwise calculate from cutPosition
        let cutDistance = seg.cutDistance ?? null;

        if (cutDistance === null) {
          // Get cut position for this segment (where drones were when cut happened)
          const cutPosData = seg.cutPosition || seg.cutPositions;
          const prevSeg = segments[i - 1];
          const prevRoutes = prevSeg?.routes || prevSeg?.solution?.routes || {};

          console.log(`[loadFromSegmentsArray] Seg ${i}: Calculating cutDistance from cutPosition`);

          // Calculate distance to cut position for each drone, use the max
          let maxDistToCut = 0;
          Object.entries(prevRoutes).forEach(([did, routeData]) => {
            const traj = routeData?.trajectory || [];
            const droneCutPos = cutPosData?.[did];

            if (droneCutPos && Array.isArray(droneCutPos) && traj.length > 1) {
              const distToCut = calculateDistanceToCutPosition(traj, droneCutPos);
              console.log(`[loadFromSegmentsArray] Seg ${i}: D${did} distToCut=${distToCut?.toFixed(1) || 'null'} to pos [${droneCutPos[0].toFixed(1)},${droneCutPos[1].toFixed(1)}]`);
              if (distToCut !== null && distToCut > maxDistToCut) {
                maxDistToCut = distToCut;
              }
            } else {
              // Fallback: use full trajectory length if no cut position
              const trajLen = calculateTrajectoryLength(traj);
              console.log(`[loadFromSegmentsArray] Seg ${i}: D${did} no cutPos, using full trajLen=${trajLen.toFixed(1)}`);
              if (trajLen > maxDistToCut) {
                maxDistToCut = trajLen;
              }
            }
          });

          cumulativeDistance += maxDistToCut;
          cutDistance = cumulativeDistance;
          console.log(`[loadFromSegmentsArray] Seg ${i}: cutDistance = ${cutDistance.toFixed(1)} (cumulative)`);
        } else {
          cumulativeDistance = cutDistance;  // Use explicit value
          console.log(`[loadFromSegmentsArray] Seg ${i}: Using explicit cutDistance = ${cutDistance}`);
        }

        // Get visited targets: targets that appeared in PREVIOUS segments but NOT in this segment
        // OR from explicit visitedTargets/frozenTargets if available
        let visitedTargets = [];
        if (seg.visitedTargets) {
          visitedTargets = [...seg.visitedTargets];
        } else if (seg.frozenTargets) {
          // V2 export format uses frozenTargets
          visitedTargets = [...seg.frozenTargets];
        } else {
          // Infer: targets that appeared in previous segments but are missing from this one
          const thisSegTargetIds = new Set((seg.env?.targets || []).map(t => t.id));
          const prevSegTargetIds = new Set();

          // Collect all targets from segments 0 to i-1
          for (let j = 0; j < i; j++) {
            const prevSegTargets = this._segmentTargets[j] || [];
            prevSegTargets.forEach(t => prevSegTargetIds.add(t.id));
          }

          // Visited = targets in previous segments but NOT in current segment
          visitedTargets = Array.from(prevSegTargetIds).filter(id => !thisSegTargetIds.has(id));
        }

        // Get drone positions from cutPosition or cutPositions if available
        const dronePositions = {};
        // Check both cutPosition (singular, from JSON) and cutPositions (plural, legacy)
        const cutPosData = seg.cutPosition || seg.cutPositions;
        console.log(`[loadFromSegmentsArray] Segment ${i} cutPosData:`, cutPosData);
        if (cutPosData) {
          Object.entries(cutPosData).forEach(([did, pos]) => {
            // Handle both formats: direct array [x,y] or object {position: [x,y]}
            if (Array.isArray(pos)) {
              dronePositions[did] = { position: pos };
              console.log(`[loadFromSegmentsArray] Seg ${i} D${did}: stored position [${pos[0].toFixed(2)}, ${pos[1].toFixed(2)}]`);
            } else if (pos && pos.position) {
              dronePositions[did] = { position: pos.position };
              console.log(`[loadFromSegmentsArray] Seg ${i} D${did}: stored position [${pos.position[0].toFixed(2)}, ${pos.position[1].toFixed(2)}]`);
            }
          });
        }

        this._segmentCuts.push({
          cutDistance,
          visitedTargets,
          dronePositions,
        });
        console.log(`[loadFromSegmentsArray] _segmentCuts[${this._segmentCuts.length - 1}] cutDistance=${cutDistance?.toFixed(1) || 'null'}, dronePositions:`, dronePositions);
      }
    }

    // Log summary of all segment cuts for debugging
    console.log(`[loadFromSegmentsArray] Segment cuts summary:`);
    this._segmentCuts.forEach((cut, idx) => {
      console.log(`   Cut ${idx + 1}: cutDistance=${cut.cutDistance?.toFixed(1) || 'null'}, visited=[${cut.visitedTargets.join(',')}]`);
    });

    this._currentSegmentIndex = 0;
    this._isActive = true;

    return true;
  }

  /** Debug info */
  getDebugInfo() {
    return {
      isActive: this._isActive,
      currentSegment: this._currentSegmentIndex,
      totalSegments: this.getTotalSegments(),
      allTargets: this._allTargets.map(t => t.id),
      visitedNow: this.getVisitedTargetsForSegment(this._currentSegmentIndex),
      unfrozen: this.getUnfrozenTargets().map(t => t.id),
    };
  }
}

// Global segmented import manager instance
const segmentedImport = new SegmentedImportManager();

/**
 * Merge arrays by ID - upserts objects from srcArr into baseArr
 * Objects with matching IDs are replaced, new objects are added
 */
function mergeById(baseArr, srcArr) {
  const m = new Map(baseArr.map(o => [String(o.id), o]));
  srcArr.forEach(o => m.set(String(o.id), o));
  return Array.from(m.values());
}

/**
 * Merge current state.env forward into all future segments
 * Updates both missionReplay segments AND missionState.segmentedMission.segments
 * Call this after accepting a solution to propagate env edits to later segments
 */
function mergeEnvForwardFromCurrent(startSegmentIndex) {
  // For segmentInfo workflow (imported segmented missions), DO NOT merge targets forward
  // Each segment should keep its own distinct targets
  const isSegmentInfoWorkflow = state.importedSegmentCuts && state.importedSegmentCuts.length > 0;
  if (isSegmentInfoWorkflow) {
    return;
  }

  const src = JSON.parse(JSON.stringify(state.env));

  // Update missionReplay segments
  for (let i = startSegmentIndex; i < missionReplay.getSegmentCount(); i++) {
    const seg = missionReplay.getSegment(i);
    if (!seg) continue;

    const merged = JSON.parse(JSON.stringify(seg.env));

    merged.targets = mergeById(merged.targets || [], src.targets || []);
    merged.sams = mergeById(merged.sams || [], src.sams || []);
    merged.airports = mergeById(merged.airports || [], src.airports || []);

    missionReplay.replaceSegment(i, {
      solution: seg.solution,
      env: merged,
      prefixDistances: seg.prefixDistances,
      isCheckpointReplan: seg.isCheckpointReplan,
      cutPosition: seg.cutPosition,
      cutPositions: seg.cutPositions,
      cutDistance: seg.cutDistance,  // Preserve cutDistance for segmentInfo workflow
    });
  }

  // Also update missionState.segmentedMission.segments (for segment-by-segment workflow)
  if (missionState.segmentedMission && missionState.segmentedMission.segments) {
    const buildIdx = missionState.currentBuildSegmentIndex || 0;
    appendDebugLine(`[mergeForward] buildIdx=${buildIdx}, src.targets=${(src.targets||[]).map(t=>t.id).join(",")}`);
    for (let i = buildIdx + 1; i < missionState.segmentedMission.segments.length; i++) {
      const seg = missionState.segmentedMission.segments[i];
      if (!seg || !seg.env) {
        appendDebugLine(`[mergeForward] seg ${i} has no env, skipping`);
        continue;
      }
      const beforeIds = (seg.env.targets || []).map(t => t.id);
      const srcIds = (src.targets || []).map(t => t.id);
      // Find IDs unique to this segment (not in src)
      const uniqueToSeg = beforeIds.filter(id => !srcIds.includes(id));
      appendDebugLine(`[mergeForward] seg ${i} BEFORE: ${beforeIds.join(",")}`);
      if (uniqueToSeg.length > 0) {
        appendDebugLine(`[mergeForward] seg ${i} UNIQUE targets (should be preserved): ${uniqueToSeg.join(",")}`);
      }

      seg.env.targets = mergeById(seg.env.targets || [], src.targets || []);
      seg.env.sams = mergeById(seg.env.sams || [], src.sams || []);
      seg.env.airports = mergeById(seg.env.airports || [], src.airports || []);

      const afterIds = seg.env.targets.map(t => t.id);
      appendDebugLine(`[mergeForward] seg ${i} AFTER: ${afterIds.join(",")}`);
    }
  }

  if (startSegmentIndex < missionReplay.getSegmentCount()) {
  }
}

/**
 * Get UI permissions based on current mission mode
 * Returns which actions are allowed in the current state
 */
function getUiPermissions() {
  const mode = missionState.mode;
  const hasCommittedPlan = missionState.committedSegments.length > 0 ||
                          missionReplay.getSegmentCount() > 0 ||
                          segmentedImport.isActive();
  const hasDraftSolution = missionState.draftSolution !== null;

  // CHECKPOINT from replay/cut disables solve; CHECKPOINT from edits allows solve
  const checkpointAllowsSolve = (mode === MissionMode.CHECKPOINT && missionState.checkpointSource !== "replay_cut");

  // Segmented mission: block animate until all segments solved and accepted
  // Support both old segmentedMission system AND new SegmentedImportManager
  const useNewManager = segmentedImport.isActive();

  let segmentedComplete = true;
  if (useNewManager) {
    // New SegmentedImportManager: check if all segments are complete
    // OR if we have routes to animate (e.g., after Reset which sets index back to 0)
    const hasRoutesToAnimate = Object.keys(state.routes || {}).some(did => {
      const r = state.routes[did];
      return r && Array.isArray(r.trajectory) && r.trajectory.length >= 2;
    });
    segmentedComplete = segmentedImport.isComplete() || hasRoutesToAnimate;
  } else if (missionState.segmentedMission) {
    // Old segmentedMission workflow
    const segCount = missionState.segmentedMission.segments.length;
    const segIdx = missionState.currentBuildSegmentIndex || 0;
    segmentedComplete = segIdx >= segCount;
  }

  return {
    // Animation controls
    canAnimate: mode === MissionMode.READY_TO_ANIMATE && segmentedComplete,
    canResume: mode === MissionMode.PAUSED_MID_ANIMATION,
    canPause: mode === MissionMode.ANIMATING,
    canCut: mode === MissionMode.ANIMATING || mode === MissionMode.PAUSED_MID_ANIMATION,

    // Editing controls
    canEnterEdit: mode === MissionMode.IDLE ||
                  mode === MissionMode.CHECKPOINT ||
                  mode === MissionMode.PAUSED_MID_ANIMATION ||
                  mode === MissionMode.READY_TO_ANIMATE,
    canAcceptEdits: mode === MissionMode.EDITING_ENV,

    // Planning controls
    canSolve: ((mode === MissionMode.IDLE) || checkpointAllowsSolve) &&
              (mode !== MissionMode.READY_TO_ANIMATE),
    canOptimize: mode === MissionMode.DRAFT_READY,
    canAcceptSolution: mode === MissionMode.DRAFT_READY,
    canDiscardDraft: mode === MissionMode.DRAFT_READY,

    // Reset
    canReset: mode !== MissionMode.ANIMATING &&
              mode !== MissionMode.EDITING_ENV &&
              hasCommittedPlan,

    // Derived states for UI
    isAnimating: mode === MissionMode.ANIMATING,
    isEditing: mode === MissionMode.EDITING_ENV,
    hasDraftSolution: hasDraftSolution,
    hasCommittedPlan: hasCommittedPlan,
  };
}

/**
 * Set mission mode and update UI accordingly
 */
function setMissionMode(newMode, reason = "") {
  const oldMode = missionState.mode;
  if (oldMode === newMode) return;

  missionState.mode = newMode;
  appendDebugLine(`Mode: ${newMode}${reason ? ` - ${reason}` : ""}`);

  // Update UI elements based on new mode
  updateStatusBanner();
  updateButtonStates();

  // Update Mission Control panel
  if (typeof updateMissionControlState === "function") {
    updateMissionControlState();
  }
}

/**
 * Get status banner text for current mode
 */
function getStatusBannerText() {
  const mode = missionState.mode;

  switch (mode) {
    case MissionMode.IDLE:
      return "Ready. Load environment, edit, or Run Planner.";
    case MissionMode.EDITING_ENV:
      return "Editing environment. Click Accept Edits when done.";
    case MissionMode.DRAFT_READY:
      return "Draft solution ready. Optimize or Accept Solution to proceed.";
    case MissionMode.READY_TO_ANIMATE:
      return "Solution accepted. Click Animate to start mission.";
    case MissionMode.ANIMATING:
      return "Animating. Press Pause to stop, or C to cut/checkpoint.";
    case MissionMode.PAUSED_MID_ANIMATION:
      return "Paused. Click Animate to resume, or Edit to modify environment.";
    case MissionMode.CHECKPOINT:
      return "Checkpoint. Edit environment and Run Planner to replan remainder.";
    default:
      return `Mode: ${mode}`;
  }
}

/**
 * Update the status banner display
 */
function updateStatusBanner() {
  const banner = $("status-banner");
  const bannerText = $("status-banner-text");
  if (!banner) return;

  const text = getStatusBannerText();
  const mode = missionState.mode;
  const perms = getUiPermissions();

  // Update text
  if (bannerText) {
    bannerText.textContent = text;
  }

  // Set banner color based on mode
  banner.className = "status-banner";
  switch (mode) {
    case MissionMode.ANIMATING:
      banner.classList.add("status-animating");
      break;
    case MissionMode.EDITING_ENV:
      banner.classList.add("status-editing");
      break;
    case MissionMode.DRAFT_READY:
      banner.classList.add("status-draft-ready");
      break;
    case MissionMode.READY_TO_ANIMATE:
      banner.classList.add("status-ready-to-animate");
      break;
    case MissionMode.CHECKPOINT:
      banner.classList.add("status-checkpoint");
      break;
    case MissionMode.PAUSED_MID_ANIMATION:
      banner.classList.add("status-paused");
      break;
    default:
      banner.classList.add("status-idle");
  }

  // Show/hide action buttons based on permissions
  const acceptEditsBtn = $("btn-accept-edits");
  const acceptSolutionBtn = $("btn-accept-solution");
  const discardBtn = $("btn-discard-draft");

  if (acceptEditsBtn) {
    acceptEditsBtn.style.display = perms.canAcceptEdits ? "inline-block" : "none";
  }
  if (acceptSolutionBtn) {
    acceptSolutionBtn.style.display = perms.canAcceptSolution ? "inline-block" : "none";
  }
  if (discardBtn) {
    discardBtn.style.display = perms.canDiscardDraft ? "inline-block" : "none";
  }
}

/**
 * Update button enabled/disabled states based on current mode
 */
function updateButtonStates() {
  const perms = getUiPermissions();

  // Animation buttons
  const animateBtn = $("anim-all");
  if (animateBtn) {
    animateBtn.disabled = !(perms.canAnimate || perms.canResume);
    animateBtn.textContent = perms.canResume ? "Resume" : "Animate";
  }

  // Pause button (if exists)
  const pauseBtn = $("btn-pause");
  if (pauseBtn) {
    pauseBtn.disabled = !perms.canPause;
  }

  // Edit button
  const editBtn = $("btn-edit-mode");
  if (editBtn) {
    editBtn.disabled = !perms.canEnterEdit && !perms.isEditing;
  }

  // Accept Edits button
  const acceptEditsBtn = $("btn-accept-edits");
  if (acceptEditsBtn) {
    acceptEditsBtn.disabled = !perms.canAcceptEdits;
    acceptEditsBtn.style.display = perms.isEditing ? "inline-block" : "none";
  }

  // Run Planner / Solve button
  const solveBtn = $("btn-run-planner");
  if (solveBtn) {
    solveBtn.disabled = !perms.canSolve;
  }

  // Accept Solution button
  const acceptSolutionBtn = $("btn-accept-solution");
  if (acceptSolutionBtn) {
    acceptSolutionBtn.disabled = !perms.canAcceptSolution;
    acceptSolutionBtn.style.display = perms.hasDraftSolution ? "inline-block" : "none";
  }

  // Discard Draft button
  const discardBtn = $("btn-discard-draft");
  if (discardBtn) {
    discardBtn.disabled = !perms.canDiscardDraft;
    discardBtn.style.display = perms.hasDraftSolution ? "inline-block" : "none";
  }

  // Keep the Agent panel visually consistent with Mission Control permissions.
  updateAgentUiState(perms);
}

/**
 * Keep the Agent panel state-aware (disable input/send) based on the same mission
 * state machine used by Mission Control.
 */
function updateAgentUiState(perms = null) {
  const p = perms || (typeof getUiPermissions === "function" ? getUiPermissions() : null);

  const inputEl = $("agent-input");
  const btnSend = $("btn-send-agent");
  const statusEl = $("agent-status");

  if (!inputEl || !btnSend) return;

  // Default: enabled
  let disable = false;
  let msg = "";
  let cls = "";

  if (p && p.isAnimating) {
    disable = true;
    msg = "Agent Solve disabled during animation. Pause or stop the animation first.";
    cls = "agent-status-warn";
  } else if (p && p.isEditing) {
    disable = true;
    msg = "Agent Solve disabled during Edit mode. Accept or cancel edits first.";
    cls = "agent-status-warn";
  } else if (p && !p.canSolve) {
    disable = true;
    msg = "Agent Solve disabled while paused/replaying/checkpoint-locked or when a solution is already accepted.";
    cls = "agent-status-warn";
  }

  inputEl.disabled = disable;
  btnSend.disabled = disable;

  // Optional: make the disabled state obvious in the placeholder
  if (disable) {
    inputEl.placeholder = "Agent disabled by mission state. Use Mission Control to Pause/Stop/Reset or enter Checkpoint replanning.";
  } else {
    inputEl.placeholder = "Type your question and press Enter...";
    cls = "agent-status-ok";
    msg = "Agent ready.";
  }

  if (statusEl) {
    statusEl.classList.remove("agent-status-warn", "agent-status-ok");
    if (cls) statusEl.classList.add(cls);
    statusEl.textContent = msg;
  }
}

// ----------------------------------------------------
// Mission Mode Action Handlers
// ----------------------------------------------------

/**
 * Enter editing mode - start tracking changes to environment
 */
function enterEditMode() {
  const perms = getUiPermissions();
  if (!perms.canEnterEdit) {
    appendDebugLine("Cannot enter edit mode in current state");
    return;
  }

  // Save a copy of current env as draft
  if (state.env) {
    missionState.draftEnv = JSON.parse(JSON.stringify(state.env));
  }

  setMissionMode(MissionMode.EDITING_ENV, "entered edit mode");
  state.editMode = true;

  const btn = $("btn-toggle-edit");
  if (btn) {
    btn.textContent = "Editing...";
    btn.classList.remove("off");
  }
}

/**
 * Accept edits - commit the draft environment
 */
function acceptEdits() {
  const perms = getUiPermissions();
  if (!perms.canAcceptEdits) {
    appendDebugLine("Cannot accept edits in current state");
    return;
  }

  // Check for duplicate target IDs (common cause of "only one target considered")
  const ids = (state.env.targets || []).map(t => String(t.id));
  const dup = ids.filter((id, i) => ids.indexOf(id) !== i);
  if (dup.length) {
    appendDebugLine(`❌ DUP target IDs: ${dup.join(", ")}`);
  }

  // Commit the draft environment
  missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));
  missionState.draftEnv = null;

  // Determine next state
  const hasCommittedPlan = missionState.committedSegments.length > 0;

  if (hasCommittedPlan) {
    // If we have a committed plan, we're in checkpoint mode - need to replan
    // Clear replay_cut flag since this is intentional replan from edits
    missionState.checkpointSource = null;
    setMissionMode(MissionMode.CHECKPOINT, "edits accepted, ready to replan");
  } else {
    // No committed plan yet - go back to IDLE
    setMissionMode(MissionMode.IDLE, "edits accepted");
  }

  state.editMode = false;
  const btn = $("btn-toggle-edit");
  if (btn) {
    btn.textContent = "Edit";
    btn.classList.add("off");
  }

  appendDebugLine("✅ Edits accepted");
  drawEnvironment();
}

/**
 * Cancel editing - discard draft changes
 */
function cancelEdits() {
  if (missionState.mode !== MissionMode.EDITING_ENV) return;

  // Restore from draft or accepted env
  if (missionState.draftEnv) {
    state.env = JSON.parse(JSON.stringify(missionState.draftEnv));
  } else if (missionState.acceptedEnv) {
    state.env = JSON.parse(JSON.stringify(missionState.acceptedEnv));
  }
  missionState.draftEnv = null;

  // Determine which state to return to
  const hasCommittedPlan = missionState.committedSegments.length > 0;
  if (hasCommittedPlan) {
    // Clear replay_cut flag since this is intentional replan from edits
    missionState.checkpointSource = null;
    setMissionMode(MissionMode.CHECKPOINT, "edits cancelled");
  } else {
    setMissionMode(MissionMode.IDLE, "edits cancelled");
  }

  state.editMode = false;
  const btn = $("btn-toggle-edit");
  if (btn) {
    btn.textContent = "Edit";
    btn.classList.add("off");
  }

  appendDebugLine("❌ Edits cancelled, reverted to previous state");
  drawEnvironment();
}

/**
 * Accept solution - commit the draft solution as the next segment
 */
function acceptSolution() {
  const perms = getUiPermissions();
  if (!perms.canAcceptSolution) {
    appendDebugLine("Cannot accept solution in current state");
    return;
  }

  if (!missionState.draftSolution) {
    appendDebugLine("No draft solution to accept");
    return;
  }

  // Check if we're using the new SegmentedImportManager
  const useNewManager = segmentedImport.isActive();

  if (useNewManager) {
    // ========================================
    // NEW CLEAN PATH: Use SegmentedImportManager
    // ========================================
    acceptSolutionWithManager();
    return;
  }

  // ========================================
  // LEGACY PATH: Original complex logic
  // ========================================

  // NOTE: We do NOT mark targets as visited here!
  // Targets only get marked as visited (green X) when user presses C (cut) during animation.
  // The cut determines which targets are "frozen" (behind the marker).

  // Snapshot the authoritative current env
  const envSnapshot = JSON.parse(JSON.stringify(state.env));
  envSnapshot.targets = (envSnapshot.targets || []).filter(t => !state.visitedTargets.includes(t.id));

  let segment;
  const currentIdx = missionReplay.getCurrentSegmentIndex();
  const existingSegment = missionReplay.getSegment(currentIdx);
  const nextSegment = missionReplay.getSegment(currentIdx + 1);

  // Check if we're in segmented import workflow (either old segmentInfo or new segmentedImport)
  const isSegmentInfoWorkflow = (
    segmentedImport.isActive() ||  // NEW: Check new segmentedImport manager
    (state.importedSegmentCuts && state.importedSegmentCuts.length > 0) ||
    (existingSegment &&
      existingSegment.solution &&
      Object.keys(existingSegment.solution.routes || {}).length === 0 &&
      missionReplay.getSegmentCount() > 1)
  );

  // For NON-checkpoint replans: Use state.routes (which may have been optimized).
  // For CHECKPOINT replans: Use draftSolution.routes (segment trajectory only, not combined).
  // applyDraftSolutionToUI combines frozen+new for DISPLAY, but we store only the new segment.
  const solutionToStore = JSON.parse(JSON.stringify(missionState.draftSolution || {}));
  const isCheckpointReplan = solutionToStore.isCheckpointReplan;
  if (!isCheckpointReplan && state.routes && Object.keys(state.routes).length > 0) {
    solutionToStore.routes = JSON.parse(JSON.stringify(state.routes));
  }
  // For checkpoint replans, keep draftSolution.routes as-is (segment trajectory only)

  // CRITICAL: Save FULL solution BEFORE any truncation happens
  // This is used by Reset to show complete seg-0 trajectory and by animation for segment switches
  // For isSegmentInfoWorkflow: segments exist, use currentIdx
  // For LEGACY: adding new segment, use count (which equals currentIdx for sequential adds)
  const segmentIdxForSave = isSegmentInfoWorkflow ? currentIdx : missionReplay.getSegmentCount();
  if (!missionState.fullSolutionsPerSegment) {
    missionState.fullSolutionsPerSegment = {};
  }
  missionState.fullSolutionsPerSegment[segmentIdxForSave] = JSON.parse(JSON.stringify(solutionToStore));
  if (segmentIdxForSave === 0) {
    missionState.seg0FullSolution = JSON.parse(JSON.stringify(solutionToStore));
    appendDebugLine(`📦 [ACCEPT] Saved seg-0 FULL solution BEFORE truncation (${Object.keys(solutionToStore.routes || {}).length} drones)`);
  }

// CRITICAL: Only truncate when committing a non-checkpoint segment.
// For checkpoint replans, solver already returns routes starting at D?_START (synthetic start),
// so truncating here will destroy the new segment trajectory.
if (!solutionToStore.isCheckpointReplan &&
    state.pendingCutPositions && Object.keys(state.pendingCutPositions).length > 0) {
  Object.entries(state.pendingCutPositions).forEach(([did, cutPos]) => {
      if (!cutPos || cutPos.length !== 2) return;
      const routeData = solutionToStore.routes?.[did];
      if (!routeData?.trajectory || routeData.trajectory.length === 0) return;

      const traj = routeData.trajectory;
      // Find trajectory point closest to cut position
      let cutIdx = 0;
      let minDist = Infinity;
      for (let i = 0; i < traj.length; i++) {
        const dx = traj[i][0] - cutPos[0];
        const dy = traj[i][1] - cutPos[1];
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < minDist) {
          minDist = dist;
          cutIdx = i;
        }
      }

      // Truncate trajectory at cut point
      const truncatedTraj = traj.slice(0, cutIdx + 1);
      // Add exact cut position as final point if needed
      const lastPt = truncatedTraj[truncatedTraj.length - 1];
      if (Math.abs(lastPt[0] - cutPos[0]) > 0.01 || Math.abs(lastPt[1] - cutPos[1]) > 0.01) {
        truncatedTraj.push([cutPos[0], cutPos[1]]);
      }

      solutionToStore.routes[did].trajectory = truncatedTraj;
      // Also update state.routes for immediate display
      if (state.routes[did]) {
        state.routes[did].trajectory = JSON.parse(JSON.stringify(truncatedTraj));
      }
    });
  }

  if (isSegmentInfoWorkflow) {
    // Old SegmentInfo workflow
    // Use drone_configs from SOLVE time (saved in draftSolution), NOT current UI state
    const segInfoDroneConfigs = missionState.draftSolution?.droneConfigs || state.droneConfigs;

    // Compute visited_targets for this segment
    const segVisitedTargets = new Set();
    Object.values(solutionToStore.routes || {}).forEach(routeData => {
      (routeData.route || []).forEach(wp => {
        if (String(wp).startsWith('T')) {
          segVisitedTargets.add(wp);
        }
      });
    });

    missionReplay.replaceSegment(currentIdx, {
      solution: solutionToStore,
      env: envSnapshot,
      cutDistance: existingSegment.cutDistance,
      cutPositions: existingSegment.cutPositions,
      isCheckpointReplan: existingSegment.isCheckpointReplan,
      drone_configs: JSON.parse(JSON.stringify(segInfoDroneConfigs)),  // Save SOLVE-time config
      visited_targets: [...segVisitedTargets],  // Store visited targets for deterministic replay
    });
    segment = missionReplay.getSegment(currentIdx);
    if (nextSegment) {
      missionReplay.setCurrentSegmentIndex(currentIdx + 1);
      missionState.currentBuildSegmentIndex = currentIdx + 1;
    }
  } else {
    // Normal workflow: ADD new segment
    // Read drone_configs directly from the UI table - what the user sees is what gets saved
    const segmentDroneConfigs = readDroneConfigsFromUI();
    appendDebugLine(`[ACCEPT] UI drone_configs: ${Object.entries(segmentDroneConfigs).map(([d,c]) => `D${d}:${c.enabled?'✓':'✗'}`).join(' ')}`);

    // Compute visited_targets: all targets in this segment's routes
    // This makes replay deterministic - we don't rely on geometric checks
    const segmentVisitedTargets = new Set();
    Object.values(solutionToStore.routes || {}).forEach(routeData => {
      (routeData.route || []).forEach(wp => {
        if (String(wp).startsWith('T')) {
          segmentVisitedTargets.add(wp);
        }
      });
    });

    // Use cutDistance from draftSolution (captured at solve time) as primary source
    // Fall back to state.pendingCutDistance for non-checkpoint replans
    const segmentCutDistance = solutionToStore.cutDistance ?? state.pendingCutDistance ?? null;

    segment = missionReplay.addSegment({
      solution: solutionToStore,
      env: envSnapshot,
      cutDistance: segmentCutDistance,
      isCheckpointReplan: solutionToStore.isCheckpointReplan || false,
      cutPositions: state.pendingCutPositions
        ? JSON.parse(JSON.stringify(state.pendingCutPositions))
        : null,
      lostDrones: state.pendingLostDrones.length > 0
        ? [...state.pendingLostDrones]
        : null,
      drone_configs: JSON.parse(JSON.stringify(segmentDroneConfigs)),  // Save SOLVE-time config
      visited_targets: [...segmentVisitedTargets],  // Store visited targets for deterministic replay
    });
    missionReplay.setCurrentSegmentIndex(segment.index);

    // DEBUG: Print segment table after adding segment
    appendDebugLine(missionReplay.printSegmentTable());

    // ========================================
    // SEGMENT STORE: Save clean segment for export
    // ========================================
    const cleanSegment = buildCleanSegment(
      solutionToStore,
      segmentDroneConfigs,
      envSnapshot,
      [...state.visitedTargets],  // frozenTargets = targets visited BEFORE this segment
      segment.index === 0 ? null : state.pendingCutPositions
    );
    segmentStore.addSegment(cleanSegment);
    appendDebugLine(`[SegmentStore] Segment ${cleanSegment.index} saved: ${Object.keys(cleanSegment.routes).length} drones`);
  }

  // Save cut positions before clearing (needed for fresh segmentation workflow)
  const savedCutPositions = state.pendingCutPositions
    ? JSON.parse(JSON.stringify(state.pendingCutPositions))
    : null;
  const savedLostDrones = state.pendingLostDrones.length > 0
    ? [...state.pendingLostDrones]
    : null;

  // Clear the pending cut data after use
  state.pendingCutPositions = null;
  state.pendingCheckpoints = null;  // Clear the checkpoints after they've been added to env
  state.pendingLostDrones = [];
  state.pendingCutDistance = null;

  // Clear checkpoint state so next solve doesn't use stale data
  // (Bug fix: prevents trajectory from going back to airport after accept→solve)
  state.checkpoint = { active: false, pct: 0.5, segments: {} };

  // Also keep in missionState.committedSegments for backward compatibility
  missionState.committedSegments.push({
    index: segment.index,
    solution: JSON.parse(JSON.stringify(solutionToStore)),
    env: envSnapshot,
    timestamp: segment.timestamp,
    cutDistance: segment.cutDistance,
    isCheckpointReplan: solutionToStore.isCheckpointReplan || false,
  });

  // Propagate env edits forward into all future segments
  const finalIdx = missionReplay.getCurrentSegmentIndex();
  mergeEnvForwardFromCurrent(finalIdx + 1);
  missionState.currentSegmentIndex = segment.index;

  // Clear the draft
  missionState.draftSolution = null;

  appendDebugLine(`✅ Solution accepted as segment ${segment.index + 1}`);

  // --- Old SegmentInfo workflow: advance to next segment or finish ---
  if (isSegmentInfoWorkflow) {
    const nextIdx = missionReplay.getCurrentSegmentIndex();
    const nextSeg = missionReplay.getSegment(nextIdx);

    const prevCutIdx = nextIdx - 1;
    if (state.importedSegmentCuts && prevCutIdx >= 0 && prevCutIdx < state.importedSegmentCuts.length) {
      const cutData = state.importedSegmentCuts[prevCutIdx];
      state.visitedTargets = [...(cutData.visitedTargets || [])];
    }

    if (nextSeg && Object.keys(nextSeg.solution.routes || {}).length === 0) {
      const allTargetsEnv = state.initialEnvSnapshot || nextSeg.env;
      state.env = JSON.parse(JSON.stringify(allTargetsEnv));
      missionState.acceptedEnv = JSON.parse(JSON.stringify(allTargetsEnv));
      state.droneConfigs = JSON.parse(JSON.stringify(allTargetsEnv.drone_configs || state.droneConfigs || {}));
      state.env.drone_configs = state.droneConfigs;
      missionState.draftSolution = null;

      if (nextSeg.cutPositions) {
        state.pendingCutPositions = JSON.parse(JSON.stringify(nextSeg.cutPositions));
      }

      initDroneConfigsFromEnv();
      updateSamWrappingClientSide();
      setMissionMode(MissionMode.IDLE, `segment ${nextIdx + 1} ready to solve`);
      drawEnvironment();
      return;
    } else {
      const combinedRoutes = buildCombinedRoutesFromSegments();
      state.routes = combinedRoutes;
      Object.keys(combinedRoutes).forEach(did => {
        state.trajectoryVisible[did] = true;
      });
      setMissionMode(MissionMode.READY_TO_ANIMATE, "all segments accepted; ready to replay");
      appendDebugLine("✅ All segments accepted. Ready to Animate full mission.");
      drawEnvironment();
      return;
    }
  }

  // --- Original segmentedMission workflow (for old format) ---
  if (missionState.segmentedMission) {
    const nextIdx = (missionState.currentBuildSegmentIndex || 0) + 1;
    missionState.currentBuildSegmentIndex = nextIdx;

    const nextSeg = missionState.segmentedMission.segments[nextIdx];
    if (nextSeg) {
      state.env = JSON.parse(JSON.stringify(nextSeg.env));
      missionState.acceptedEnv = JSON.parse(JSON.stringify(nextSeg.env));
      state.droneConfigs = JSON.parse(JSON.stringify(nextSeg.drone_configs || state.env.drone_configs || {}));
      state.env.drone_configs = state.droneConfigs;
      missionState.draftSolution = null;

      setMissionMode(MissionMode.IDLE, `segment ${nextIdx + 1} ready to solve`);
      appendDebugLine(`➡️ Loaded segment ${nextIdx + 1} env. Ready to Solve.`);

      initDroneConfigsFromEnv();
      updateSamWrappingClientSide();
      drawEnvironment();
      return;
    } else {
      // Build combined routes from all segments for display and animation
      const combinedRoutes = buildCombinedRoutesFromSegments();
      state.routes = combinedRoutes;
      Object.keys(combinedRoutes).forEach(did => {
        state.trajectoryVisible[did] = true;
      });
      setMissionMode(MissionMode.READY_TO_ANIMATE, "all segments accepted; ready to replay");
      appendDebugLine("✅ All segments accepted. Ready to Animate full mission.");
      drawEnvironment();
      return;
    }
  }

  // Fresh segmentation workflow (building segments from N1 JSON)
  // After accepting a segment, keep currently displayed targets AND add targets from the new solution
  // The frozen targets (visited) will have green X marks drawn by drawEnvironment
  if (state.initialEnvSnapshot && state.visitedTargets.length > 0) {
    // We have frozen targets from pressing C - this is fresh segmentation

    // Extract targets from the newly accepted solution's routes
    const newSolutionRouteTargets = new Set();
    if (missionState.draftSolution?.routes) {
      Object.values(missionState.draftSolution.routes).forEach(routeData => {
        const route = routeData.route || [];
        route.forEach(stop => {
          if (stop && stop.startsWith('T')) {
            newSolutionRouteTargets.add(stop);
          }
        });
      });
    }

    // Add new targets from the solution to state.env.targets (if not already present)
    const currentTargetIds = new Set((state.env.targets || []).map(t => t.id));
    const targetsToAdd = (state.initialEnvSnapshot.targets || []).filter(
      t => newSolutionRouteTargets.has(t.id) && !currentTargetIds.has(t.id)
    );

    if (targetsToAdd.length > 0) {
      state.env.targets = [...(state.env.targets || []), ...targetsToAdd];
      appendDebugLine(`[ACCEPT] Added ${targetsToAdd.length} new targets: ${targetsToAdd.map(t=>t.id).join(",")}`);
    }

    // Also update initialEnvSnapshot to include the new targets for future reference
    const snapshotTargetIds = new Set((state.initialEnvSnapshot.targets || []).map(t => t.id));
    const newTargetsForSnapshot = targetsToAdd.filter(t => !snapshotTargetIds.has(t.id));
    if (newTargetsForSnapshot.length > 0) {
      state.initialEnvSnapshot.targets = [...(state.initialEnvSnapshot.targets || []), ...newTargetsForSnapshot];
    }

    // CRITICAL: Use the segment's stored drone_configs (not state.env which has initial configs)
    // The segment stores the correct cumulative disabled drone state
    state.droneConfigs = JSON.parse(JSON.stringify(segment.drone_configs || state.droneConfigs || {}));
    state.env.drone_configs = state.droneConfigs;
    missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));

    // Restore cut positions so the marker stays visible
    // The marker shows where the drone will start for the next segment
    if (savedCutPositions) {
      state.pendingCutPositions = savedCutPositions;
    }

    // Set to READY_TO_ANIMATE so Animate button is enabled
    setMissionMode(MissionMode.READY_TO_ANIMATE, "segment accepted, ready to animate");
    drawEnvironment();
    return;
  }

  // Non-segmented mission: normal flow (no cuts made yet)
  setMissionMode(MissionMode.READY_TO_ANIMATE, "solution accepted");
  drawEnvironment();
}

/**
 * Accept solution using the new SegmentedImportManager
 * This is the CLEAN path - simple and modular
 */
function acceptSolutionWithManager() {
  const currentSegIdx = segmentedImport.getCurrentSegmentIndex();
  appendDebugLine(`[ACCEPT-NEW] Segment ${currentSegIdx}, manager: ${JSON.stringify(segmentedImport.getDebugInfo())}`);

  // For NON-checkpoint replans: Use state.routes (which may have been optimized).
  // For CHECKPOINT replans: Use draftSolution.routes (segment trajectory only, not combined).
  const solutionToStore = JSON.parse(JSON.stringify(missionState.draftSolution || {}));
  const isCheckpointReplan = solutionToStore.isCheckpointReplan;

  if (!isCheckpointReplan && state.routes && Object.keys(state.routes).length > 0) {
    solutionToStore.routes = JSON.parse(JSON.stringify(state.routes));
  }
  // For checkpoint replans, keep draftSolution.routes as-is (segment trajectory only)

  // 1. Update MissionReplay segment with the solution (using optimized routes)
  // IMPORTANT: Use drone_configs from SOLVE time (saved in draftSolution), NOT current UI state
  // This ensures we save what was actually solved, not what the UI shows after user changes
  const currentSeg = missionReplay.getSegment(currentSegIdx);
  const solveTimeDroneConfigs = missionState.draftSolution?.droneConfigs || state.droneConfigs;

  // Save the FULL original solution before it gets truncated later
  // This is used by Reset and animation segment switches to show complete trajectories
  if (!missionState.fullSolutionsPerSegment) {
    missionState.fullSolutionsPerSegment = {};
  }
  missionState.fullSolutionsPerSegment[currentSegIdx] = JSON.parse(JSON.stringify(solutionToStore));
  // Keep seg0FullSolution for backward compatibility with Reset
  if (currentSegIdx === 0) {
    missionState.seg0FullSolution = JSON.parse(JSON.stringify(solutionToStore));
    // Log trajectory lengths at save time for debugging
    const seg0Lens = Object.entries(missionState.seg0FullSolution.routes || {}).map(([did, rd]) => `D${did}:${rd.trajectory?.length || 0}`).join(', ');
    appendDebugLine(`📦 [ACCEPT] seg0FullSolution saved: [${seg0Lens}]`);
  }

  if (currentSeg) {
    // DEBUG: Log trajectory lengths being stored
    const trajLens = Object.entries(solutionToStore.routes || {}).map(([did, rd]) => `D${did}:${rd.trajectory?.length || 0}`).join(', ');
    appendDebugLine(`📦 [ACCEPT] Storing seg ${currentSegIdx} in missionReplay: [${trajLens}]`);

    missionReplay.replaceSegment(currentSegIdx, {
      solution: solutionToStore,
      env: segmentedImport.getFullEnv(),
      cutDistance: currentSeg.cutDistance,
      cutPositions: currentSeg.cutPositions,
      lostDrones: currentSeg.lostDrones,
      visitedTargets: currentSeg.visitedTargets,
      isCheckpointReplan: currentSeg.isCheckpointReplan,
      drone_configs: JSON.parse(JSON.stringify(solveTimeDroneConfigs)),
    });

    // DEBUG: Verify what was stored
    const storedSeg = missionReplay.getSegment(currentSegIdx);
    const storedLens = Object.entries(storedSeg?.solution?.routes || {}).map(([did, rd]) => `D${did}:${rd.trajectory?.length || 0}`).join(', ');
    appendDebugLine(`📦 [ACCEPT] Verified in missionReplay: [${storedLens}]`);

    // ========================================
  }

  // 2. Add to committed segments for backward compatibility
  missionState.committedSegments.push({
    index: currentSegIdx,
    solution: JSON.parse(JSON.stringify(solutionToStore)),
    env: segmentedImport.getFullEnv(),
    timestamp: Date.now(),
    cutDistance: currentSeg?.cutDistance || null,
    isCheckpointReplan: currentSegIdx > 0,
  });

  // 3. Clear draft
  missionState.draftSolution = null;

  // 4. Advance to next segment
  const hasMore = segmentedImport.advanceToNextSegment();
  const newSegIdx = segmentedImport.getCurrentSegmentIndex();

  if (hasMore) {
    // More segments to solve - calculate marker and freeze trajectory

    // 5. Calculate marker position and visited targets using cutDistance on the CONCATENATED trajectory
    // IMPORTANT: If the JSON didn't have solutions, cutDistance may be 0 from import.
    // We need to calculate it from the ACTUAL solved trajectory.
    let cutDistance = segmentedImport.getCutDistanceForSegment(newSegIdx);

    // If cutDistance is 0 or null, calculate from the solved trajectory
    if (!cutDistance || cutDistance <= 0) {
      // Calculate from the trajectory we just solved (segment currentSegIdx)
      const solvedRoutes = solutionToStore.routes || {};
      let maxTrajLen = 0;
      Object.values(solvedRoutes).forEach(routeData => {
        const traj = routeData?.trajectory || [];
        if (traj.length >= 2) {
          let trajLen = 0;
          for (let i = 1; i < traj.length; i++) {
            const dx = traj[i][0] - traj[i-1][0];
            const dy = traj[i][1] - traj[i-1][1];
            trajLen += Math.sqrt(dx * dx + dy * dy);
          }
          if (trajLen > maxTrajLen) maxTrajLen = trajLen;
        }
      });
      cutDistance = maxTrajLen;
      appendDebugLine(`[ACCEPT] cutDistance calculated from solved trajectory: ${cutDistance.toFixed(1)}`);

      // Update segmentedImport's _segmentCuts so future calls get the correct value
      if (newSegIdx > 0 && segmentedImport._segmentCuts && segmentedImport._segmentCuts[newSegIdx - 1]) {
        segmentedImport._segmentCuts[newSegIdx - 1].cutDistance = cutDistance;
        appendDebugLine(`[ACCEPT] Updated _segmentCuts[${newSegIdx - 1}].cutDistance = ${cutDistance.toFixed(1)}`);
      }
    }

    state.pendingCutPositions = {};

    // Derive lostDrones: drones that started segment N but won't start segment N+1
    // Two ways a drone becomes "lost":
    // 1. Was flying at C{N} but not at C{N+1} (crashed/ran out of fuel)
    // 2. Was enabled in segment N-1 but disabled before solving segment N (manually disabled)
    const savedLostDrones = [];
    const thisCutPositions = segmentedImport.getCutPositionForSegment(newSegIdx);
    const nextCutPositions = segmentedImport.getCutPositionForSegment(newSegIdx + 1);
    
    if (thisCutPositions && nextCutPositions) {
      // Case 1: Drones in this cut but not in next cut are "lost" at this cut
      Object.keys(thisCutPositions).forEach(did => {
        if (!nextCutPositions[did]) {
          savedLostDrones.push(did);
        }
      });
    } else if (thisCutPositions && !nextCutPositions) {
      // This is the last cut - no drones lost at this marker
      // (drones finish at their end airports, not "lost")
    }
    
    // Case 2: Also check for drones that were enabled in previous segment but disabled now
    // These are "lost" even if they weren't flying at a checkpoint
    const prevSeg = missionReplay.getSegment(newSegIdx - 1);
    const prevDroneConfigs = prevSeg?.drone_configs || {};
    
    // IMPORTANT: Get the NEXT segment's drone_configs from JSON to compare
    // Don't use state.droneConfigs because it may have been updated already
    const nextSegmentConfigs = segmentedImport.getDroneConfigsForSegment(newSegIdx) || {};
    
    Object.keys(prevDroneConfigs).forEach(did => {
      const wasEnabled = prevDroneConfigs[did]?.enabled === true;
      const isEnabled = nextSegmentConfigs[did]?.enabled === true;
      
      if (wasEnabled && !isEnabled && !savedLostDrones.includes(did)) {
        // Drone was active before but disabled now - mark as lost
        savedLostDrones.push(did);
      }
    });
    
    // For segmented import: Use state.routes (which has the ACTUAL displayed trajectory: frozen + merged)
    // instead of rebuilding from MissionReplay (which has RAW solver outputs before truncation)
    const concatenatedRoutes = newSegIdx === 1 ? {} : null;

    if (newSegIdx === 1) {
      // First Accept - build from MissionReplay
      for (let i = 0; i < newSegIdx; i++) {
        const seg = missionReplay.getSegment(i);
        if (!seg || !seg.solution || !seg.solution.routes) continue;

        Object.entries(seg.solution.routes).forEach(([droneId, routeData]) => {
          const trajectory = routeData.trajectory || [];
          if (trajectory.length === 0) return;

          if (!concatenatedRoutes[droneId]) {
            concatenatedRoutes[droneId] = {
              trajectory: [...trajectory],
            };
          } else {
            const existing = concatenatedRoutes[droneId].trajectory;
            const lastPoint = existing[existing.length - 1];
            const firstPoint = trajectory[0];
            const isDuplicate = Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
                               Math.abs(lastPoint[1] - firstPoint[1]) < 0.001;
            if (isDuplicate) {
              concatenatedRoutes[droneId].trajectory = existing.concat(trajectory.slice(1));
            } else {
              concatenatedRoutes[droneId].trajectory = existing.concat(trajectory);
            }
          }
        });
      }
    }

    // For visited targets calculation, we need the routes regardless of cutPositions source
    // For newSegIdx > 1: use state.routes (actual displayed trajectory)
    // For newSegIdx === 1: use concatenatedRoutes (built from MissionReplay)
    const routesToUse = newSegIdx === 1 ? concatenatedRoutes : state.routes;

    // DEBUG: Log what routesToUse contains
    const routeDroneIds = Object.keys(routesToUse || {});
    appendDebugLine(`[ACCEPT] routesToUse has ${routeDroneIds.length} drones: [${routeDroneIds.join(",")}]`);
    routeDroneIds.forEach(did => {
      const trajLen = routesToUse[did]?.trajectory?.length || 0;
      appendDebugLine(`   D${did}: trajectory.length = ${trajLen}`);
    });

    // Get cutPositions - PRIORITIZE the JSON's stored positions (thisCutPositions)
    // Only calculate from trajectories if JSON doesn't have positions
    appendDebugLine(`[ACCEPT] thisCutPositions from JSON: ${thisCutPositions ? JSON.stringify(thisCutPositions) : 'null'}`);
    if (thisCutPositions && Object.keys(thisCutPositions).length > 0) {
      // USE THE JSON's CUT POSITIONS DIRECTLY
      state.pendingCutPositions = JSON.parse(JSON.stringify(thisCutPositions));
      const cutPosStr = Object.entries(state.pendingCutPositions).map(([did, pos]) => `D${did}:[${pos[0].toFixed(1)},${pos[1].toFixed(1)}]`).join(", ");
      appendDebugLine(`[ACCEPT] Using JSON cut positions: ${cutPosStr}`);
    } else {
      appendDebugLine(`[ACCEPT] WARNING: No JSON cut positions, falling back to trajectory endpoints`);
      // No JSON positions - calculate from trajectories
      Object.entries(routesToUse || {}).forEach(([did, routeData]) => {
        const traj = routeData.trajectory || [];

        if (traj.length < 2) return;

        // Calculate total trajectory length for this drone
        let trajTotalDist = 0;
        for (let i = 1; i < traj.length; i++) {
          const dx = traj[i][0] - traj[i - 1][0];
          const dy = traj[i][1] - traj[i - 1][1];
          trajTotalDist += Math.sqrt(dx * dx + dy * dy);
        }

        if (cutDistance > 0) {
          // Skip drones whose trajectory ended BEFORE cutDistance (already at end airport)
          // These drones don't get a marker at this cut point
          if (trajTotalDist < cutDistance) {
            return;
          }

          // Calculate marker position by splitting trajectory at cutDistance
          const result = split_polyline_at_distance(traj, cutDistance);
          if (result.splitPoint && result.splitPoint.length === 2) {
            state.pendingCutPositions[did] = result.splitPoint;
          }
        } else {
          // No cutDistance from JSON - use the END of trajectory as the cut position
          // This is the position where the drone will start the next segment
          const lastPoint = traj[traj.length - 1];
          state.pendingCutPositions[did] = [lastPoint[0], lastPoint[1]];
          appendDebugLine(`   D${did}: Using trajectory end as cut position [${lastPoint[0].toFixed(1)}, ${lastPoint[1].toFixed(1)}]`);
        }
      });
    }

    // Get visited targets from the JSON's stored data
    // The JSON knows which targets were visited in each segment
    const jsonVisitedTargets = segmentedImport.getVisitedTargetsForSegment(newSegIdx);
    if (jsonVisitedTargets && jsonVisitedTargets.length > 0) {
      state.visitedTargets = [...jsonVisitedTargets];
      appendDebugLine(`[ACCEPT] Using JSON visited targets for segment ${newSegIdx}: [${jsonVisitedTargets.join(",")}]`);
    } else {
      // Fallback: calculate from trajectory if JSON doesn't have visited targets
      const calculatedVisited = calculateVisitedTargetsFromRoutes(routesToUse, cutDistance);
      state.visitedTargets = calculatedVisited;
      appendDebugLine(`[ACCEPT] Calculated visited targets (before cutDist=${cutDistance?.toFixed(1)}): [${calculatedVisited.join(",")}]`);
    }

    if (Object.keys(state.pendingCutPositions).length === 0) {
      appendDebugLine(`[ACCEPT] WARNING: No cut positions calculated! pendingCutPositions is empty.`);
      state.pendingCutPositions = null;
    } else {
      const cutPosStr = Object.entries(state.pendingCutPositions).map(([did, pos]) => `D${did}:[${pos[0].toFixed(1)},${pos[1].toFixed(1)}]`).join(", ");
      appendDebugLine(`[ACCEPT] Calculated cut positions: ${cutPosStr}`);
    }

    appendDebugLine(`[ACCEPT] Advanced to segment ${newSegIdx}, cutDist=${cutDistance?.toFixed(1) || 'null'}, visited=[${state.visitedTargets.join(",")}]`);

    // 6. Update MissionReplay index and save cut positions for the NEXT segment
    missionReplay.setCurrentSegmentIndex(newSegIdx);
    missionState.currentBuildSegmentIndex = newSegIdx;

    // Save the cut positions, visited targets, and lost drones into the next segment
    // This allows us to draw all previous markers and restore correct visited targets on Reset
    const nextSeg = missionReplay.getSegment(newSegIdx);
    if (nextSeg) {
      const savedCutPositions = state.pendingCutPositions ? JSON.parse(JSON.stringify(state.pendingCutPositions)) : nextSeg.cutPositions;
      missionReplay.replaceSegment(newSegIdx, {
        ...nextSeg,
        cutPositions: savedCutPositions,
        visitedTargets: [...state.visitedTargets],  // Save calculated visited targets
        lostDrones: savedLostDrones,  // Drones lost at this checkpoint
      });

      // Verify what was saved
      const savedSeg = missionReplay.getSegment(newSegIdx);
      const savedCutPosStr = savedSeg?.cutPositions
        ? Object.entries(savedSeg.cutPositions).map(([did, pos]) => `D${did}:[${pos[0].toFixed(1)},${pos[1].toFixed(1)}]`).join(", ")
        : "null";
      appendDebugLine(`[ACCEPT] Saved to missionReplay segment ${newSegIdx}: cutPositions=${savedCutPosStr}`);
    }

    // CRITICAL: Clear pendingCutPositions after saving to missionReplay
    // This prevents double-drawing: missionReplay loop draws with correct lostDrones,
    // while pendingCutPositions block would incorrectly use current droneConfigs state
    state.pendingCutPositions = null;

    // More segments to solve
    // PRESERVE existing frozen trajectory and DON'T recalculate it
    // The frozen trajectory is IMMUTABLE once placed
    const frozenRoutes = {};

    // For each drone, keep the existing frozen trajectory (already in state.routes)
    // Do NOT rebuild from MissionReplay - that would use NEW solver trajectories
    // ALSO include drones from previous segments that may now be disabled
    const activeDroneIds = newSegIdx === 1
      ? Object.keys(concatenatedRoutes || {})
      : Object.keys(state.routes || {});

    // Collect drone IDs from previous MissionReplay segments (these may have been disabled)
    const previousDroneIds = new Set();
    for (let i = 0; i < newSegIdx; i++) {
      const seg = missionReplay.getSegment(i);
      if (seg?.solution?.routes) {
        Object.keys(seg.solution.routes).forEach(did => previousDroneIds.add(did));
      }
    }

    // Combine: active drones + previously active drones (now disabled)
    const droneIds = [...new Set([...activeDroneIds, ...previousDroneIds])];

    // BUILD LOST DRONE MAP FIRST - needed for immediate truncation
    // lostDrones in segment N = drones that died at cut CN (end of segment N-1)
    const earlyLostDroneMap = {};
    for (let i = 1; i <= missionReplay.getSegmentCount(); i++) {
      const seg = missionReplay.getSegment(i);
      if (seg?.lostDrones) {
        seg.lostDrones.forEach(did => {
          earlyLostDroneMap[String(did)] = i;
        });
      }
    }
    // Note: lostDrones are stored in missionReplay segments, not segmentedImport
    // segmentedImport only manages targets/cuts, missionReplay has the actual segment data
    if (Object.keys(earlyLostDroneMap).length > 0) {
      appendDebugLine(`[ACCEPT] Lost drones: ${Object.entries(earlyLostDroneMap).map(([d,s]) => `D${d}@seg${s}`).join(', ')}`);
    }

    droneIds.forEach((droneId) => {
      // Check if this drone is LOST
      const lostAtSeg = earlyLostDroneMap[droneId];
      const isLostDrone = lostAtSeg !== undefined && lostAtSeg <= newSegIdx;

      // KEEP FULL TRAJECTORY AT ACCEPT TIME
      // Truncation now happens at Run Planner time (applyDraftSolutionToUI)
      // EXCEPT: Lost drones need immediate truncation at kill position
      if (newSegIdx > 1 && state.routes[droneId] && state.routes[droneId].trajectory) {
        // state.routes has the MERGED trajectory (frozen + new segment)
        let mergedTraj = state.routes[droneId].trajectory || [];
        if (mergedTraj.length > 0) {
          // If drone is lost, truncate at kill position
          if (isLostDrone) {
            const killSeg = missionReplay.getSegment(lostAtSeg);
            // cutPositions are stored in the segment, or use segmentedImport.getCutPositionForSegment
            const killPos = killSeg?.cutPositions?.[droneId] || segmentedImport.getCutPositionForSegment(lostAtSeg)?.[droneId];
            if (killPos) {
              mergedTraj = truncateTrajectoryAtPosition([...mergedTraj], killPos);
              appendDebugLine(`[ACCEPT] D${droneId}: LOST drone - truncated at kill pos [${killPos[0].toFixed(1)},${killPos[1].toFixed(1)}], ${state.routes[droneId].trajectory.length} → ${mergedTraj.length} pts`);
            }
          }
          frozenRoutes[droneId] = {
            ...JSON.parse(JSON.stringify(state.routes[droneId])),
            trajectory: mergedTraj,
          };
          if (!isLostDrone) {
            appendDebugLine(`[ACCEPT] D${droneId}: Storing FULL merged trajectory (length ${mergedTraj.length}), truncation deferred to Run Planner`);
          }
        }
      } else if (newSegIdx === 1 && concatenatedRoutes && concatenatedRoutes[droneId]) {
        // First Accept
        let fullTraj = concatenatedRoutes[droneId].trajectory || [];

        // Get the full route data from the last segment for this drone
        let lastRouteData = null;
        for (let i = newSegIdx - 1; i >= 0; i--) {
          const seg = missionReplay.getSegment(i);
          if (seg?.solution?.routes?.[droneId]) {
            lastRouteData = seg.solution.routes[droneId];
            break;
          }
        }

        if (fullTraj.length > 0) {
          // If drone is lost, truncate at kill position
          if (isLostDrone) {
            const killSeg = missionReplay.getSegment(lostAtSeg);
            // cutPositions are stored in the segment, or use segmentedImport.getCutPositionForSegment
            const killPos = killSeg?.cutPositions?.[droneId] || segmentedImport.getCutPositionForSegment(lostAtSeg)?.[droneId];
            if (killPos) {
              fullTraj = truncateTrajectoryAtPosition([...fullTraj], killPos);
              appendDebugLine(`[ACCEPT] D${droneId}: LOST drone - truncated at kill pos [${killPos[0].toFixed(1)},${killPos[1].toFixed(1)}], ${concatenatedRoutes[droneId].trajectory.length} → ${fullTraj.length} pts`);
            }
          }
          frozenRoutes[droneId] = {
            ...(lastRouteData || {}),
            trajectory: fullTraj,
          };
          if (!isLostDrone) {
            appendDebugLine(`[ACCEPT] D${droneId}: Storing FULL trajectory (length ${fullTraj.length}), truncation deferred to Run Planner`);
          }
        }
      }
    });

    // UNCONDITIONAL FINAL PASS: Ensure ALL trajectories from MissionReplay are preserved
    // This catches any drone that wasn't handled above (disabled, edge cases, etc.)
    //
    // BUG FIX: Previously, for each segment i, we only built trajectory up to segment i.
    // This caused dead drone trajectories to disappear because their cumulative trajectory
    // wasn't being built correctly. Now we collect ALL unique droneIds from ALL segments
    // and build each drone's cumulative trajectory from ALL segments where it appears.

    const allDroneIdsInReplay = new Set();
    for (let i = 0; i < newSegIdx; i++) {
      const seg = missionReplay.getSegment(i);
      if (seg?.solution?.routes) {
        Object.keys(seg.solution.routes).forEach(did => allDroneIdsInReplay.add(did));
      }
    }

    // Build map of which drones were lost at which segment
    // lostDrones in segment N = drones that died at cut CN (end of segment N-1)
    const droneLostAtSegment = {};
    for (let i = 1; i < missionReplay.getSegmentCount(); i++) {
      const seg = missionReplay.getSegment(i);
      if (seg?.lostDrones) {
        seg.lostDrones.forEach(did => {
          droneLostAtSegment[String(did)] = i;
        });
      }
    }

    allDroneIdsInReplay.forEach(droneId => {
      // If this drone already has a frozen route, skip
      if (frozenRoutes[droneId]?.trajectory?.length > 0) return;

      // Check if this drone was lost
      const lostAtSeg = droneLostAtSegment[droneId];

      // Build cumulative trajectory from ALL segments where this drone appears
      // But stop at the kill position if drone was lost
      const cumulativeTraj = [];
      let lastRouteData = null;

      for (let j = 0; j < newSegIdx; j++) {
        // Skip if drone was already lost before this segment
        if (lostAtSeg !== undefined && lostAtSeg <= j) {
          break;
        }

        const s = missionReplay.getSegment(j);
        const routeData = s?.solution?.routes?.[droneId];
        let t = routeData?.trajectory || [];

        if (t.length === 0) continue;

        // Check if drone is killed at the END of this segment (lost at j+1)
        const droneKilledHere = (lostAtSeg === j + 1);
        if (droneKilledHere) {
          // Truncate at kill position
          const nextSeg = missionReplay.getSegment(j + 1);
          const killPos = nextSeg?.cutPositions?.[droneId];
          if (killPos) {
            t = truncateTrajectoryAtPosition([...t], killPos);
            appendDebugLine(`[ACCEPT] D${droneId}: LOST - truncated at kill pos [${killPos[0].toFixed(1)},${killPos[1].toFixed(1)}]`);
          }
        }

        lastRouteData = routeData;  // Keep track of the last route data for this drone
        if (cumulativeTraj.length === 0) {
          cumulativeTraj.push(...t);
        } else {
          // Skip first point if duplicate
          const lastPt = cumulativeTraj[cumulativeTraj.length - 1];
          const firstPt = t[0];
          const startIdx = (lastPt && firstPt &&
            Math.abs(lastPt[0] - firstPt[0]) < 0.001 &&
            Math.abs(lastPt[1] - firstPt[1]) < 0.001) ? 1 : 0;
          cumulativeTraj.push(...t.slice(startIdx));
        }

        // If drone was killed here, don't continue to next segment
        if (droneKilledHere) break;
      }

      if (cumulativeTraj.length > 0 && lastRouteData) {
        frozenRoutes[droneId] = {
          ...lastRouteData,
          trajectory: cumulativeTraj,
        };
        if (lostAtSeg !== undefined) {
          appendDebugLine(`[ACCEPT] D${droneId}: LOST drone frozen trajectory preserved (${cumulativeTraj.length} pts)`);
        }
      }
    });

    // IMPORTANT: Update ALL previous MissionReplay segments with their PER-SEGMENT trajectories
    // frozenRoutes contains the cumulative frozen trajectory (all segments concatenated)
    // We need to split it back into individual segment trajectories for MissionReplay

    // Get the previous frozen routes (before this Accept)
    const prevFrozenTraj = {};
    if (newSegIdx > 1) {
      // For segments after the first, we have previous frozen trajectory in state.routes (before updating)
      // But we just overwrote state.routes with frozenRoutes, so we need to rebuild it from Mission Replay
      for (let i = 0; i < newSegIdx - 1; i++) {
        const seg = missionReplay.getSegment(i);
        if (seg && seg.solution && seg.solution.routes) {
          Object.entries(seg.solution.routes).forEach(([did, routeData]) => {
            const traj = routeData.trajectory || [];
            if (traj.length === 0) return;

            if (!prevFrozenTraj[did]) {
              prevFrozenTraj[did] = [...traj];
            } else {
              // Concatenate
              const existing = prevFrozenTraj[did];
              const lastPoint = existing[existing.length - 1];
              const firstPoint = traj[0];
              const isDuplicate = Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
                                 Math.abs(lastPoint[1] - firstPoint[1]) < 0.001;
              if (isDuplicate) {
                prevFrozenTraj[did] = existing.concat(traj.slice(1));
              } else {
                prevFrozenTraj[did] = existing.concat(traj);
              }
            }
          });
        }
      }
    }

    // Now update the CURRENT segment (currentSegIdx) with its portion
    const currentSeg = missionReplay.getSegment(currentSegIdx);
    if (currentSeg && currentSeg.solution && currentSeg.solution.routes) {
      const segmentSolution = {
        routes: {},
        sequences: currentSeg.solution.sequences || {},
      };

      Object.entries(currentSeg.solution.routes).forEach(([did, routeData]) => {
        // Extract this segment's trajectory from frozenRoutes
        const cumulativeTraj = frozenRoutes[did] ? frozenRoutes[did].trajectory : [];
        const prevTraj = prevFrozenTraj[did] || [];

        // This segment's trajectory = cumulative - previous
        const segmentTraj = cumulativeTraj.slice(prevTraj.length);

        segmentSolution.routes[did] = {
          ...routeData,
          trajectory: segmentTraj,
        };

      });

      missionReplay.replaceSegment(currentSegIdx, {
        ...currentSeg,
        solution: segmentSolution,
      });
    }

    state.routes = frozenRoutes;
    // Debug: Log trajectory lengths and route sequences for all frozen routes
    appendDebugLine(`[ACCEPT] state.routes set with ${Object.keys(frozenRoutes).length} drones:`);
    Object.entries(frozenRoutes).forEach(([did, routeData]) => {
      const trajLen = routeData?.trajectory?.length || 0;
      const routeSeq = routeData?.route || [];
      const trajFirst = routeData?.trajectory?.[0];
      const trajLast = routeData?.trajectory?.[trajLen - 1];
      appendDebugLine(`  D${did}: route=[${routeSeq.join(',')}], traj=${trajLen}pts (${trajFirst ? `${trajFirst[0].toFixed(1)},${trajFirst[1].toFixed(1)}` : '?'} → ${trajLast ? `${trajLast[0].toFixed(1)},${trajLast[1].toFixed(1)}` : '?'})`);
    });

    // Make frozen trajectories visible
    Object.keys(frozenRoutes).forEach(did => {
      state.trajectoryVisible[did] = true;
    });

    // Keep the visited targets we set earlier (from JSON or calculated)
    // Don't recalculate - the JSON's visitedTargets is the source of truth
    appendDebugLine(`[ACCEPT] Final visited targets: [${state.visitedTargets.join(",")}]`);

    // Set up env for drawing - targets up to current segment (progressive reveal)
    const displayEnv = segmentedImport.getEnvForDisplay();
    state.env = displayEnv;

    // SIMPLE: Get drone_configs directly from the JSON for this segment
    // The JSON stores per-segment drone_configs which specify exactly which drones are enabled
    const segmentConfigs = segmentedImport.getDroneConfigsForSegment(newSegIdx);

    let derivedConfigs;
    if (segmentConfigs && Object.keys(segmentConfigs).length > 0) {
      // MERGE JSON configs with current UI configs, preserving user modifications
      derivedConfigs = JSON.parse(JSON.stringify(segmentConfigs));

      // Override with current UI values where they exist
      // UI values (like fuel_budget) should ALWAYS trump JSON values
      Object.keys(derivedConfigs).forEach(did => {
        const currentConfig = state.droneConfigs?.[did];
        if (currentConfig) {
          // Preserve UI-modified values: fuel_budget, target_access, etc.
          if (currentConfig.fuel_budget !== undefined) {
            derivedConfigs[did].fuel_budget = currentConfig.fuel_budget;
          }
          if (currentConfig.target_access) {
            derivedConfigs[did].target_access = JSON.parse(JSON.stringify(currentConfig.target_access));
          }
        }
      });



      // Update start_airport for drones that have cutPositions
      const thisSegCutPositions = segmentedImport.getCutPositionForSegment(newSegIdx);
      if (thisSegCutPositions) {
        Object.keys(thisSegCutPositions).forEach(did => {
          if (derivedConfigs[did]) {
            derivedConfigs[did].start_airport = `D${did}_START`;
          }
        });
      }

      const enabledDrones = Object.entries(derivedConfigs).filter(([,c]) => c.enabled).map(([d]) => d);
    } else {
      // Fallback to displayEnv drone_configs
      derivedConfigs = JSON.parse(JSON.stringify(displayEnv.drone_configs || {}));
    }

    state.droneConfigs = derivedConfigs;
    state.env.drone_configs = state.droneConfigs;
    missionState.acceptedEnv = JSON.parse(JSON.stringify(displayEnv));

    initDroneConfigsFromEnv();
    updateSamWrappingClientSide();

    const unfrozen = segmentedImport.getUnfrozenTargets();
    appendDebugLine(`➡️ Segment ${newSegIdx}: ${segmentedImport.getAllTargets().length} total, ${unfrozen.length} to visit, frozen: ${state.visitedTargets.join(",") || "none"}`);

    setMissionMode(MissionMode.IDLE, `segment ${newSegIdx} ready to solve`);
    drawEnvironment();
  } else {
    // All segments complete - ready to animate
    // CRITICAL FIX: Before building combined routes, we need to extract just the final
    // segment's portion from state.routes (which has the full merged trajectory).
    // Otherwise buildCombinedRoutesFromSegments() will concatenate the FULL trajectory
    // from segment 3 onto segments 0-2, causing phantom overlay trajectories.

    // Build previous frozen trajectory from segments 0 to currentSegIdx-1
    const prevFrozenTraj = {};
    for (let i = 0; i < currentSegIdx; i++) {
      const seg = missionReplay.getSegment(i);
      if (seg && seg.solution && seg.solution.routes) {
        Object.entries(seg.solution.routes).forEach(([did, routeData]) => {
          const traj = routeData.trajectory || [];
          if (traj.length === 0) return;

          if (!prevFrozenTraj[did]) {
            prevFrozenTraj[did] = [...traj];
          } else {
            // Concatenate, skipping duplicate junction point
            const existing = prevFrozenTraj[did];
            const lastPoint = existing[existing.length - 1];
            const firstPoint = traj[0];
            const isDuplicate = Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
                               Math.abs(lastPoint[1] - firstPoint[1]) < 0.001;
            if (isDuplicate) {
              prevFrozenTraj[did] = existing.concat(traj.slice(1));
            } else {
              prevFrozenTraj[did] = existing.concat(traj);
            }
          }
        });
      }
    }

    // Now update the FINAL segment (currentSegIdx) with just its portion
    const finalSeg = missionReplay.getSegment(currentSegIdx);
    if (finalSeg && finalSeg.solution && finalSeg.solution.routes) {
      const segmentSolution = {
        routes: {},
        sequences: finalSeg.solution.sequences || {},
      };

      Object.entries(finalSeg.solution.routes).forEach(([did, routeData]) => {
        // state.routes has the full merged trajectory
        const fullTraj = state.routes[did]?.trajectory || routeData.trajectory || [];
        const prevTraj = prevFrozenTraj[did] || [];

        // This segment's trajectory = full - previous
        const segmentTraj = fullTraj.slice(prevTraj.length);

        segmentSolution.routes[did] = {
          ...routeData,
          trajectory: segmentTraj,
        };

      });

      missionReplay.replaceSegment(currentSegIdx, {
        ...finalSeg,
        solution: segmentSolution,
      });
    }

    // After all segments accepted: Show the FULL combined trajectory from all segments
    // This is the complete frozen mission trajectory
    appendDebugLine(`[FINAL ACCEPT] missionReplay has ${missionReplay.getSegmentCount()} segments`);
    const combinedRoutes = buildCombinedRoutesFromSegments();
    appendDebugLine(`[FINAL ACCEPT] buildCombinedRoutesFromSegments returned ${combinedRoutes ? Object.keys(combinedRoutes).length : 0} drones`);

    // Use the concatenated/truncated trajectories directly
    // buildCombinedRoutesFromSegments() already truncates each segment at the correct cut point
    if (combinedRoutes && Object.keys(combinedRoutes).length > 0) {
      state.routes = JSON.parse(JSON.stringify(combinedRoutes));
      Object.keys(state.routes).forEach(did => {
        state.trajectoryVisible[did] = true;
      });
      // Log trajectory details and extract sequence by detecting waypoints
      const fullEnv = segmentedImport.getFullEnv() || state.env;
      const waypointMap = {};
      (fullEnv.airports || []).forEach(a => { waypointMap[`${a.x.toFixed(1)},${a.y.toFixed(1)}`] = a.id; });
      (fullEnv.targets || []).forEach(t => { waypointMap[`${t.x.toFixed(1)},${t.y.toFixed(1)}`] = t.id; });
      // Add checkpoints
      for (let segIdx = 1; segIdx < missionReplay.getSegmentCount(); segIdx++) {
        const seg = missionReplay.getSegment(segIdx);
        if (seg?.cutPositions) {
          Object.entries(seg.cutPositions).forEach(([droneId, pos]) => {
            if (pos && pos.length === 2) {
              waypointMap[`${pos[0].toFixed(1)},${pos[1].toFixed(1)}`] = `C${segIdx}-${droneId}`;
            }
          });
        }
      }

      Object.entries(state.routes).forEach(([did, rd]) => {
        const traj = rd.trajectory || [];
        const first = traj[0];
        const last = traj[traj.length - 1];

        // Extract sequence by checking each trajectory point against waypoints
        const sequence = [];
        const visited = new Set();
        for (const pt of traj) {
          const key = `${pt[0].toFixed(1)},${pt[1].toFixed(1)}`;
          const wpId = waypointMap[key];
          if (wpId && !visited.has(wpId)) {
            // Only include checkpoints for THIS drone
            if (wpId.match(/^C\d+-\d+$/)) {
              const cpDroneId = wpId.split('-')[1];
              if (cpDroneId !== did) continue;
            }
            sequence.push(wpId);
            visited.add(wpId);
          }
        }

        appendDebugLine(`  D${did}: ${traj.length} pts, SEQUENCE = ${sequence.join(' → ')}`);
      });
      appendDebugLine(`📺 Showing combined trajectory`);
    } else {
      // Fallback to segment 0 if combined routes fail
      const seg0Solution = missionState.seg0FullSolution || missionReplay.getSegment(0)?.solution;
      if (seg0Solution?.routes) {
        state.routes = JSON.parse(JSON.stringify(seg0Solution.routes));
        Object.keys(state.routes).forEach(did => {
          state.trajectoryVisible[did] = true;
        });
        appendDebugLine(`📺 Fallback: Showing segment 0 trajectory (${Object.entries(state.routes).map(([d,r]) => `D${d}:${r.trajectory?.length||0}`).join(', ')})`);
      } else {
        state.routes = {};
      }
    }

    // After final Accept, ALL targets are visited because the full mission was completed
    // The drone flew from airport through all checkpoints back to airport
    const allTargetIds = (state.env?.targets || segmentedImport.getAllTargets() || []).map(t => t.id);
    state.visitedTargets = allTargetIds;
    appendDebugLine(`[FINAL ACCEPT] All targets marked as visited: [${allTargetIds.join(",")}]`);
    // Clear any pending cut markers
    state.pendingCutPositions = null;

    // Set env to full environment with all targets
    const fullEnv = segmentedImport.getFullEnv();
    state.env = fullEnv || segmentedImport.getEnvForDisplay();
    // Use final segment's drone_configs (reflects final state of drones)
    const finalSegConfigs = missionReplay.getSegment(currentSegIdx)?.drone_configs || state.env.drone_configs || {};
    state.droneConfigs = JSON.parse(JSON.stringify(finalSegConfigs));
    state.env.drone_configs = state.droneConfigs;
    missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));

    // Reset segment manager to start (so animation starts from segment 0)
    segmentedImport._currentSegmentIndex = 0;
    missionReplay.resetToStart();

    setMissionMode(MissionMode.READY_TO_ANIMATE, "all segments accepted; ready to replay");
    appendDebugLine("✅ All segments accepted. Ready to Animate full mission.");
    drawEnvironment();
  }
}

/**
 * Discard draft solution - go back to previous state
 */
function discardDraftSolution() {
  // DEBUG: Log who called discard and from where
  const perms = getUiPermissions();
  if (!perms.canDiscardDraft) {
    appendDebugLine("Cannot discard draft in current state");
    return;
  }

  // CRITICAL: If discarding a checkpoint replan, restore the checkpoint state
  // so the next solve will still respect frozen trajectories
  const draft = missionState.draftSolution;
  if (draft && draft.isCheckpointReplan) {
    appendDebugLine("🔄 Restoring checkpoint state from discarded draft...");

    // Restore checkpoint segments (frozen trajectory data)
    if (draft.checkpointSegments) {
      state.checkpoint = {
        active: true,
        pct: 0.5,
        segments: JSON.parse(JSON.stringify(draft.checkpointSegments)),
      };
      appendDebugLine(`   Restored ${Object.keys(draft.checkpointSegments).length} checkpoint segments`);
    }

    // Restore pending cut positions (cut marker locations)
    if (draft.pendingCutPositions) {
      state.pendingCutPositions = JSON.parse(JSON.stringify(draft.pendingCutPositions));
      appendDebugLine(`   Restored pendingCutPositions for drones: [${Object.keys(draft.pendingCutPositions).join(", ")}]`);
    }

    // Restore cut distance
    if (draft.cutDistance !== null && draft.cutDistance !== undefined) {
      state.pendingCutDistance = draft.cutDistance;
      appendDebugLine(`   Restored pendingCutDistance: ${draft.cutDistance.toFixed(1)}`);
    }
  }

  // Clear the draft solution
  missionState.draftSolution = null;

  // Clear pending lost drones (from rejected solve attempts)
  state.pendingLostDrones = [];

  // Check if we restored checkpoint state (determines what mode to go to)
  const restoredCheckpoint = draft && draft.isCheckpointReplan && state.checkpoint?.active;

  // Check if we have a committed solution to restore from MissionReplay
  const currentSegment = missionReplay.getCurrentSegment();
  const hasCommittedPlan = currentSegment && currentSegment.solution;

  if (hasCommittedPlan) {
    // Restore the committed segment's routes
    const solution = currentSegment.solution;
    if (solution.routes) {
      // Solution already has routes in the new format
      state.routes = JSON.parse(JSON.stringify(solution.routes));
    } else if (solution.drone_routes) {
      // Legacy format
      state.routes = {};
      solution.drone_routes.forEach(r => {
        state.routes[String(r.drone_id)] = {
          route: r.route,
          trajectory: r.trajectory,
          distance: r.distance,
          fuel_budget: r.fuel_budget,
          points: r.points || 0,
        };
      });
    }
    // If checkpoint state was restored, go to CHECKPOINT mode so user can solve again
    // Otherwise go back to ready to animate
    if (restoredCheckpoint) {
      setMissionMode(MissionMode.CHECKPOINT, "draft discarded, checkpoint state restored - ready to solve again");
    } else {
      setMissionMode(MissionMode.READY_TO_ANIMATE, "draft discarded, restored committed solution");
    }
  } else {
    // Fallback to old system
    const oldSegment = missionState.committedSegments[missionState.committedSegments.length - 1];
    if (oldSegment && oldSegment.solution && oldSegment.solution.drone_routes) {
      state.routes = {};
      oldSegment.solution.drone_routes.forEach(r => {
        state.routes[String(r.drone_id)] = {
          route: r.route,
          trajectory: r.trajectory,
          distance: r.distance,
          fuel_budget: r.fuel_budget,
          points: r.points || 0,
        };
      });
      if (restoredCheckpoint) {
        setMissionMode(MissionMode.CHECKPOINT, "draft discarded, checkpoint state restored - ready to solve again");
      } else {
        setMissionMode(MissionMode.READY_TO_ANIMATE, "draft discarded, restored committed solution");
      }
    } else {
      // No committed solution - clear routes and go to IDLE
      state.routes = {};
      if (restoredCheckpoint) {
        setMissionMode(MissionMode.CHECKPOINT, "draft discarded, checkpoint state restored - ready to solve again");
      } else {
        setMissionMode(MissionMode.IDLE, "draft discarded");
      }
    }
  }

  appendDebugLine(restoredCheckpoint
    ? "🔄 Draft discarded - checkpoint state restored, ready to solve again"
    : "❌ Draft solution discarded");
  drawEnvironment();
}

/**
 * Reset mission - return to initial state with first solution visible
 * Goes to READY_TO_ANIMATE so user can replay from the beginning
 */
function resetMission() {
  const perms = getUiPermissions();
  if (!perms.canReset) {
    appendDebugLine("Cannot reset in current state");
    return;
  }

  // Must have segments to reset to (check both old and new system)
  // Check BEFORE clearing anything
  if (missionReplay.getSegmentCount() === 0 && missionState.committedSegments.length === 0) {
    appendDebugLine("No committed segments to reset to");
    return;
  }

  // --- HARD RESET: stop playback + clear segmented replay ---
  // Note: We do NOT clear missionReplay here - we need it to restore segment 0
  try {
    // Stop animation if running
    if (state.animationFrameId) {
      cancelAnimationFrame(state.animationFrameId);
      state.animationFrameId = null;
    }
    if (state.animationTimerId) {
      clearInterval(state.animationTimerId);
      state.animationTimerId = null;
    }

    // Clear segment-related state (but NOT missionReplay - we need it for reset)
    missionState.currentSegmentIndex = 0;
    missionState.currentBuildSegmentIndex = 0;

    // Clear cut/freeze bookkeeping
    state.pendingCutPositions = null;
    state.pendingCutDistance = null;
    state.pendingLostDrones = [];
    state.visitedTargets = [];

    // Clear routes (will be restored from segment 0)
    state.routes = {};
    state.sequences = {};
  } catch (e) {
  }

  // =====================================================
  // 1. Stop animation
  // =====================================================
  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }
  state.animation.active = false;
  state.animation.drones = {};

  // =====================================================
  // 2. Clear ALL state first (before restoring solution)
  // =====================================================
  state.checkpoint = { active: false, pct: 0.5, segments: {} };
  state.pendingCutDistance = null;
  state.pendingCutPositions = null;
  state.pendingCheckpoints = null;
  state.checkpointSegmentCounter = 0;  // Reset checkpoint counter on full reset
  state.pendingLostDrones = [];
  state.missionHistory = [];
  state.visitedTargets = [];
  state.currentCutSegment = 1;
  state.previouslyAssignedTargets = [];
  state.targetSegmentMap = {};
  state.trajectoryVisible = {};  // No trajectories shown until Display is clicked

  missionState.currentSegmentIndex = 0;
  missionState.draftEnv = null;
  missionState.draftSolution = null;
  missionState.pauseContext = null;
  missionState.checkpointSource = null;

  // =====================================================
  // 3. Reset MissionReplay to start
  // =====================================================
  missionReplay.resetToStart();

  // =====================================================
  // NEW PATH: For multi-segment missions (via import OR cut workflow)
  // =====================================================
  const resetSegCount = missionReplay.getSegmentCount();
  // Use this path if we have multiple segments (whether from import or from cut workflow)
  if (resetSegCount > 1 || segmentedImport.isActive()) {
    // At Reset, load segment 0's FULL trajectory (airport → airport)
    // This is the same as creation workflow - animation will switch segments at checkpoints
    // For import: use segmentedImport. For creation: use seg0FullSolution.
    let seg0Solution = null;
    if (segmentedImport.isActive()) {
      seg0Solution = segmentedImport.getSolutionForSegment(0);
    }
    if (!seg0Solution?.routes || Object.keys(seg0Solution.routes).length === 0) {
      seg0Solution = missionState.seg0FullSolution;
    }
    if (!seg0Solution?.routes || Object.keys(seg0Solution.routes).length === 0) {
      const seg0 = missionReplay.getSegment(0);
      seg0Solution = seg0?.solution;
    }

    if (seg0Solution?.routes && Object.keys(seg0Solution.routes).length > 0) {
      state.routes = JSON.parse(JSON.stringify(seg0Solution.routes));
      const trajInfo = Object.entries(state.routes).map(([did, rd]) => `D${did}:${rd.trajectory?.length || 0}`).join(', ');
      appendDebugLine(`🔄 Reset: Loaded seg-0 FULL trajectory [${trajInfo}]`);
    } else {
      state.routes = {};
      appendDebugLine(`🔄 Reset: No seg-0 routes found`);
    }
    Object.keys(state.routes).forEach(did => {
      state.trajectoryVisible[did] = true;
    });

    // Store drone loss info for animation to use when extending trajectories
    // Check missionReplay, segmentedImport, AND segmentManager for lostDrones
    const droneLostAtSegment = {};
    const segCount = missionReplay.getSegmentCount();
    // Also check segmentManager if available
    const smActive = typeof segmentManager !== 'undefined' && segmentManager?.isActive?.();
    const smSegCount = smActive ? segmentManager.getSegmentCount?.() : 0;
    const maxSegCount = Math.max(segCount, smSegCount || 0);
    for (let segIdx = 1; segIdx < maxSegCount; segIdx++) {
      // Check missionReplay first
      let lostDrones = missionReplay.getSegment(segIdx)?.lostDrones || [];
      // Fallback to segmentedImport
      if (lostDrones.length === 0 && segmentedImport.isActive()) {
        const importSeg = segmentedImport.getSegment?.(segIdx);
        lostDrones = importSeg?.lostDrones || [];
      }
      // Fallback to segmentManager
      if (lostDrones.length === 0 && smActive) {
        const smSeg = segmentManager.getSegment?.(segIdx);
        lostDrones = smSeg?.lostDrones || [];
        if (lostDrones.length > 0) {
        }
      }
      if (lostDrones.length > 0) {
        lostDrones.forEach(did => {
          if (droneLostAtSegment[did] === undefined) {
            droneLostAtSegment[did] = segIdx;
          }
        });
      }
    }
    // Store for animation segment switch code to use
    missionState.droneLostAtSegment = droneLostAtSegment;

    // NOTE: Trajectory truncation for lost drones happens during ANIMATION, not at Reset
    // At Reset, we show seg-0 trajectories. Animation will extend them and truncate lost drones.
    // Log final state
    Object.keys(state.routes).forEach(did => {
      const r = state.routes[did];
      const isLost = droneLostAtSegment[did] !== undefined;
    });

    // On Reset: Clear visited targets - checkmarks will appear during animation as targets are passed
    state.visitedTargets = [];
    // Set pending cut positions for C1 marker (so it's visible after Reset)
    // Check segmentedImport first (original JSON), then missionReplay
    let c1Positions = null;
    if (segmentedImport.isActive()) {
      c1Positions = segmentedImport.getCutPositionForSegment(1);
    }
    if (!c1Positions || Object.keys(c1Positions).length === 0) {
      const seg1 = missionReplay.getSegment(1);
      c1Positions = seg1?.cutPositions || null;
    }
    state.pendingCutPositions = c1Positions ? JSON.parse(JSON.stringify(c1Positions)) : null;

    // Reset segmented import manager to segment 0
    segmentedImport._currentSegmentIndex = 0;

    // Keep missionReplay at segment 0 for animation (so it checks C1 boundary at segment 1)
    missionReplay.setCurrentSegmentIndex(0);

    // Set env to display targets - collect ALL targets from ALL segments
    // This ensures targets added in later segments are available for animation
    const seg0 = missionReplay.getSegment(0);
    if (seg0?.env) {
      state.env = JSON.parse(JSON.stringify(seg0.env));

      // Collect all targets from all segments
      const allTargetsMap = new Map();
      for (let i = 0; i < missionReplay.getSegmentCount(); i++) {
        const seg = missionReplay.getSegment(i);
        if (seg?.env?.targets) {
          seg.env.targets.forEach(t => {
            if (!allTargetsMap.has(t.id)) {
              allTargetsMap.set(t.id, t);
            }
          });
        }
      }
      const allTargets = Array.from(allTargetsMap.values());

      // Store all targets in initialEnvSnapshot for segment switch to use
      state.initialEnvSnapshot = JSON.parse(JSON.stringify(state.env));
      state.initialEnvSnapshot.targets = allTargets;

      // For display at Reset, show only targets from segment 0's solution routes
      // state.initialEnvSnapshot has ALL targets for marking as visited during animation
      // state.env.targets only has current segment's targets for display
      if (seg0.solution && seg0.solution.routes) {
        // Extract target IDs from segment 0's solution routes
        const seg0RouteTargets = new Set();
        Object.values(seg0.solution.routes).forEach(routeData => {
          const route = routeData.route || [];
          route.forEach(stop => {
            if (stop && stop.startsWith('T')) {
              seg0RouteTargets.add(stop);
            }
          });
        });
        // Filter to only targets in segment 0's routes
        const allEnvTargets = seg0.env.targets || [];
        state.env.targets = allEnvTargets.filter(t => seg0RouteTargets.has(t.id));
      } else {
        state.env.targets = JSON.parse(JSON.stringify(seg0.env.targets || []));
      }
    }
    // Load drone_configs from segment 0 (the source of truth)
    if (seg0 && seg0.drone_configs) {
      state.droneConfigs = JSON.parse(JSON.stringify(seg0.drone_configs));
      if (state.env) state.env.drone_configs = state.droneConfigs;
      refreshDroneConfigUI();  // Update the UI table
    } else if (state.env) {
      state.droneConfigs = JSON.parse(JSON.stringify(state.env.drone_configs || {}));
      state.env.drone_configs = state.droneConfigs;
    }
    missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));

    setMissionMode(MissionMode.READY_TO_ANIMATE, "segmented mission reset - ready to animate from segment 0");
    appendDebugLine("🔄 Segmented mission reset - ready to replay from beginning");
    updateAirportDropdowns();
    updateAnimationButtonStates();
    drawEnvironment();
    return;
  }

  // For segmentInfo workflow (imported segmented missions), clear ALL solutions
  // This ensures Reset returns to import state - no solutions, no markers
  const isSegmentInfoWorkflow = state.importedSegmentCuts && state.importedSegmentCuts.length > 0;
  if (isSegmentInfoWorkflow) {
    missionReplay.clearAllSolutions();
  }

  // =====================================================
  // 4. Restore environment and initialEnvSnapshot
  // For replay, we need ALL targets visible (from initialEnvSnapshot)
  // =====================================================
  const firstSegment = missionReplay.getSegment(0);

  // First, ensure we have initialEnvSnapshot with ALL targets
  // Collect all unique targets from all segments
  const allTargetsMap = new Map();
  for (let i = 0; i < missionReplay.getSegmentCount(); i++) {
    const seg = missionReplay.getSegment(i);
    if (seg && seg.env && seg.env.targets) {
      seg.env.targets.forEach(t => {
        if (!allTargetsMap.has(t.id)) {
          allTargetsMap.set(t.id, t);
        }
      });
    }
  }
  const allTargetsFromSegments = Array.from(allTargetsMap.values());
  const existingSnapshotTargetCount = state.initialEnvSnapshot?.targets?.length || 0;
  // Only update initialEnvSnapshot if we collected MORE targets than it already has
  // (The original snapshot from first solve should have all targets)
  if (allTargetsFromSegments.length > existingSnapshotTargetCount && firstSegment && firstSegment.env) {
    const fullEnv = JSON.parse(JSON.stringify(firstSegment.env));
    fullEnv.targets = allTargetsFromSegments;
    state.initialEnvSnapshot = fullEnv;
  } else if (existingSnapshotTargetCount > 0) {
  } else if (firstSegment && firstSegment.env) {
    // No existing snapshot and no better source - use segment targets
    const fullEnv = JSON.parse(JSON.stringify(firstSegment.env));
    fullEnv.targets = allTargetsFromSegments;
    state.initialEnvSnapshot = fullEnv;
  }

  // Set state.env from segment 0 - show only segment 0's targets after Reset
  // During animation, targets will appear as the drone enters each segment
  if (firstSegment && firstSegment.env) {
    state.env = JSON.parse(JSON.stringify(firstSegment.env));
    missionState.acceptedEnv = JSON.parse(JSON.stringify(firstSegment.env));

    // Load drone_configs from segment 0 (the source of truth)
    if (firstSegment.drone_configs) {
      state.droneConfigs = JSON.parse(JSON.stringify(firstSegment.drone_configs));
      state.env.drone_configs = state.droneConfigs;
      refreshDroneConfigUI();  // Update the UI table
    }

    // For fresh segmentation (NOT segmentInfo workflow), filter targets to only those in segment 0's routes
    // This is because during fresh segmentation, segment 0 is created with ALL targets in env,
    // but we only want to show the targets that the solution actually visits
    if (!isSegmentInfoWorkflow && firstSegment.solution && firstSegment.solution.routes) {
      // Extract target IDs from segment 0's solution routes
      const seg0RouteTargets = new Set();
      Object.values(firstSegment.solution.routes).forEach(routeData => {
        const route = routeData.route || [];
        route.forEach(stop => {
          if (stop && stop.startsWith('T')) {
            seg0RouteTargets.add(stop);
          }
        });
      });

      if (seg0RouteTargets.size > 0) {
        // Filter env.targets to only include targets from segment 0's routes
        const allEnvTargets = state.env.targets || [];
        state.env.targets = allEnvTargets.filter(t => seg0RouteTargets.has(t.id));
        missionState.acceptedEnv.targets = state.env.targets;
      } else {
      }
    } else {
      const seg0Targets = (firstSegment.env.targets || []).map(t => t.id);
    }
  }

  // =====================================================
  // 5. Restore first segment's routes (without drawing)
  // =====================================================
  // For segmentInfo workflow, we've cleared all solutions - start fresh with no routes
  if (isSegmentInfoWorkflow) {
    state.sequences = {};
    state.routes = {};
    state.wrappedPolygons = [];
    state.allocations = {};
    state.allocationStrategy = null;
    state.distanceMatrix = null;
  } else if (firstSegment && firstSegment.solution) {
    const solution = firstSegment.solution;
    // Manually restore routes without calling applyDraftSolutionToUI (which draws)
    state.sequences = JSON.parse(JSON.stringify(solution.sequences || {}));
    state.routes = JSON.parse(JSON.stringify(solution.routes || {}));
    state.wrappedPolygons = JSON.parse(JSON.stringify(solution.wrappedPolygons || []));
    state.allocations = JSON.parse(JSON.stringify(solution.allocations || {}));
    state.allocationStrategy = solution.allocationStrategy || null;
    state.distanceMatrix = solution.distanceMatrix ? JSON.parse(JSON.stringify(solution.distanceMatrix)) : null;
  } else {
    // Fallback to old system
    const oldFirstSegment = missionState.committedSegments[0];
    if (oldFirstSegment && oldFirstSegment.solution) {
      const solution = oldFirstSegment.solution;
      state.sequences = JSON.parse(JSON.stringify(solution.sequences || {}));
      state.routes = JSON.parse(JSON.stringify(solution.routes || {}));
      state.wrappedPolygons = JSON.parse(JSON.stringify(solution.wrappedPolygons || []));
      state.allocations = JSON.parse(JSON.stringify(solution.allocations || {}));
      state.allocationStrategy = solution.allocationStrategy || null;
    }
  }

  // Clear any _fullTrajectory overrides
  Object.values(state.routes).forEach(routeData => {
    if (routeData._fullTrajectory) {
      delete routeData._fullTrajectory;
    }
  });

  // Make trajectories visible for all restored routes (so user can see them before/during animation)
  Object.keys(state.routes).forEach(did => {
    state.trajectoryVisible[did] = true;
  });

  // =====================================================
  // 6. Set mode and redraw
  // =====================================================
  if (isSegmentInfoWorkflow) {
    // For segmentInfo workflow, reset to EDITABLE (no solution yet)
    setMissionMode(MissionMode.EDITABLE, "segmentInfo workflow reset - solve segment 0");
    appendDebugLine("🔄 Mission reset - ready to solve segment 0");
  } else {
    setMissionMode(MissionMode.READY_TO_ANIMATE, "mission reset to start");
    appendDebugLine("🔄 Mission reset - ready to replay from beginning");
  }
  updateAirportDropdowns();
  updateAnimationButtonStates();
  updateStatsFromRoutes();
  drawEnvironment();
}

// ----------------------------------------------------
// Utility helpers
// ----------------------------------------------------
function appendDebugLine(msg) {
  const pre = $("debug-output");
  if (!pre) return;
  const now = new Date().toLocaleTimeString();
  pre.textContent = `[${now}] ${msg}\n` + pre.textContent;
}

function invalidateMission(reason) {
  // Drop current mission so backend will treat next v4 call as a new mission
  state.missionId = null;

  // IMPORTANT: During checkpoint workflow, preserve frozen trajectories
  // Only clear routes if we don't have committed segments (no frozen trajectories to preserve)
  const hasCommittedSegments = missionState.committedSegments && missionState.committedSegments.length > 0;
  if (!hasCommittedSegments) {
    state.routes = {};
    state.trajectoryVisible = {};
  } else {
    // In checkpoint workflow - keep frozen routes/trajectories for continuity
  }

  if (reason) {
    appendDebugLine("Mission invalidated: " + reason);
  }
}

// ----------------------------------------------------
// Client-side SAM wrapping (no server round-trip!)
// ----------------------------------------------------
function _sampleCircle(cx, cy, r, minSeg = 2.0) {

  if (r <= 0) return [[cx, cy]];
  let dtheta = minSeg / Math.max(r, 0.001);
  dtheta = Math.max(dtheta, Math.PI / 36); // 5 degrees
  dtheta = Math.min(dtheta, Math.PI / 6);  // 30 degrees
  const nSteps = Math.max(8, Math.ceil(2 * Math.PI / dtheta));
  const thetaStep = 2 * Math.PI / nSteps;
  const pts = [];
  for (let i = 0; i < nSteps; i++) {
    const theta = i * thetaStep;
    pts.push([cx + r * Math.cos(theta), cy + r * Math.sin(theta)]);
  }
  return pts;
}

function _cross(o, a, b) {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

function _convexHull(points) {
  const pts = [...new Set(points.map(p => JSON.stringify(p)))].map(s => JSON.parse(s));
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (pts.length <= 1) return pts;

  const lower = [];
  for (const p of pts) {
    while (lower.length >= 2 && _cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
      lower.pop();
    }
    lower.push(p);
  }

  const upper = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && _cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
      upper.pop();
    }
    upper.push(p);
  }

  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

function _distance(a, b) {
  return Math.hypot(b[0] - a[0], b[1] - a[1]);
}

function _samsOverlap(sam1, sam2) {
  const [x1, y1] = sam1.pos || [sam1.x || 0, sam1.y || 0];
  const [x2, y2] = sam2.pos || [sam2.x || 0, sam2.y || 0];
  const r1 = sam1.range || sam1.radius || 0;
  const r2 = sam2.range || sam2.radius || 0;
  return _distance([x1, y1], [x2, y2]) <= (r1 + r2);
}

// ----------------------------------------------------
// Checkpoint freeze key handler (C) - REMOVED, handled in main keydown listener
// ----------------------------------------------------
// Old duplicate handler removed - C key is now handled in the main window.addEventListener("keydown")
// which properly uses cutAtCheckpoint() with state machine permissions

// Reset mission key handler (R)
// ----------------------------------------------------
if (!window.__resetMissionKeyInstalled) {
  window.__resetMissionKeyInstalled = true;

  document.addEventListener(
    "keydown",
    (e) => {
      if (String(e.key).toLowerCase() === "r") {
        // Skip if user is typing in an input field
        const activeTag = document.activeElement?.tagName?.toLowerCase();
        if (activeTag === "input" || activeTag === "textarea" || activeTag === "select") {
          return;
        }

        appendDebugLine("R key pressed - attempting reset...");

        // Use state machine permissions
        const perms = getUiPermissions();
        if (perms.canReset) {
          resetMission();
        } else {
          appendDebugLine("Cannot reset in current state: " + missionState.mode);
        }
      }
    },
    true
  );
}


function _clusterOverlappingSams(sams) {
  if (!sams || sams.length === 0) return [];
  const n = sams.length;
  const parent = Array.from({ length: n }, (_, i) => i);

  function find(x) {
    if (parent[x] !== x) parent[x] = find(parent[x]);
    return parent[x];
  }

  function union(x, y) {
    const px = find(x), py = find(y);
    if (px !== py) parent[px] = py;
  }

  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      if (_samsOverlap(sams[i], sams[j])) {
        union(i, j);
      }
    }
  }

  const clustersMap = {};
  for (let i = 0; i < n; i++) {
    const root = find(i);
    if (!clustersMap[root]) clustersMap[root] = [];
    clustersMap[root].push(sams[i]);
  }

  return Object.values(clustersMap);
}

function computeWrappedPolygonsClientSide(sams) {
  if (!sams || sams.length === 0) return [];

  const clusters = _clusterOverlappingSams(sams);
  const polygons = [];

  for (const cluster of clusters) {
    const allPoints = [];
    for (const sam of cluster) {
      const [cx, cy] = sam.pos || [sam.x || 0, sam.y || 0];
      const r = sam.range || sam.radius || 0;
      allPoints.push(..._sampleCircle(cx, cy, r, 2.0));
    }
    if (allPoints.length === 0) continue;

    const hull = _convexHull(allPoints);
    if (hull.length >= 3) {
      polygons.push(hull);
    }
  }

  return polygons;
}

// Update SAM wrapping instantly (client-side) during drag
function updateSamWrappingClientSide() {
  if (state.env && state.env.sams && state.env.sams.length > 0) {
    state.wrappedPolygons = computeWrappedPolygonsClientSide(state.env.sams);
  } else {
    state.wrappedPolygons = [];
  }
}

// ----------------------------------------------------
// Calculate visited targets from route and cutDistance
// Returns array of target IDs that are BEFORE the cut point
// ----------------------------------------------------
/**
 * Calculate visited targets based on the cut position on the solved trajectory.
 * Walks along the trajectory from START and marks targets as visited if they
 * appear BEFORE the cut position.
 *
 * @param {Object} segment - The accepted segment with solution
 * @param {Object} cutPositions - Object with drone ID -> [x, y] cut position
 * @param {Object} env - Environment with target positions
 * @returns {Array} List of visited target IDs
 */
function calculateVisitedTargetsFromCutPosition(segment, cutPositions, env) {
  if (!segment || !segment.solution || !cutPositions) {
    return [];
  }

  const visited = [];
  const routes = segment.solution.routes || {};
  const targets = env?.targets || [];

  // Build target position lookup
  const targetPos = {};
  targets.forEach(t => {
    targetPos[t.id] = { x: t.x, y: t.y };
  });

  Object.entries(routes).forEach(([did, routeData]) => {
    const traj = routeData.trajectory || [];
    const route = routeData.route || [];
    const cutPos = cutPositions[did];

    if (traj.length < 2 || !cutPos) return;

    // Find where the cut position is on the trajectory
    // Find the trajectory point closest to the cut position
    let cutPointIdx = -1;
    let minCutDist = Infinity;
    traj.forEach((pt, idx) => {
      const dist = Math.sqrt(Math.pow(pt[0] - cutPos[0], 2) + Math.pow(pt[1] - cutPos[1], 2));
      if (dist < minCutDist) {
        minCutDist = dist;
        cutPointIdx = idx;
      }
    });

    if (cutPointIdx < 0) return;

    // For each target in the route ORDER, check if it appears before the cut point
    for (const wp of route) {
      if (!String(wp).startsWith("T")) continue;
      if (visited.includes(wp)) continue;

      const pos = targetPos[wp];
      if (!pos) continue;

      // Find trajectory point closest to this target
      let minDist = Infinity;
      let targetIdx = -1;
      traj.forEach((pt, idx) => {
        const dist = Math.sqrt(Math.pow(pt[0] - pos.x, 2) + Math.pow(pt[1] - pos.y, 2));
        if (dist < minDist) {
          minDist = dist;
          targetIdx = idx;
        }
      });

      // Target is visited if its trajectory index is BEFORE the cut point index
      if (targetIdx >= 0 && minDist < 20.0) {
        if (targetIdx < cutPointIdx) {
          visited.push(wp);
        } else {
        }
      }
    }
  });

  return visited;
}

// ----------------------------------------------------
// Polyline utilities
// points: array of [x,y]
// dist: distance along polyline in same units as coords
// ----------------------------------------------------

/**
 * Truncate a trajectory at a given target position.
 * Finds the closest point on the trajectory to the target and returns
 * all points up to and including that point (plus the target itself).
 * @param {Array} trajectory - Array of [x, y] points
 * @param {Array} targetPos - [x, y] target position to truncate at
 * @returns {Array} - Truncated trajectory ending at targetPos
 */
function truncateTrajectoryAtPosition(trajectory, targetPos) {
  if (!trajectory || trajectory.length === 0 || !targetPos || targetPos.length !== 2) {
    return trajectory || [];
  }

  // Find the segment where the target position lies (closest approach)
  let bestDist = Infinity;
  let bestIdx = -1;

  for (let i = 0; i < trajectory.length - 1; i++) {
    const p1 = trajectory[i];
    const p2 = trajectory[i + 1];

    // Calculate closest point on segment p1-p2 to targetPos
    const dx = p2[0] - p1[0];
    const dy = p2[1] - p1[1];
    const segLenSq = dx * dx + dy * dy;

    let t = 0;
    if (segLenSq > 0) {
      t = ((targetPos[0] - p1[0]) * dx + (targetPos[1] - p1[1]) * dy) / segLenSq;
      t = Math.max(0, Math.min(1, t));
    }

    const closestX = p1[0] + t * dx;
    const closestY = p1[1] + t * dy;
    const distSq = (targetPos[0] - closestX) ** 2 + (targetPos[1] - closestY) ** 2;

    if (distSq < bestDist) {
      bestDist = distSq;
      bestIdx = i;
    }
  }

  if (bestIdx === -1) {
    // Couldn't find a good segment, return full trajectory
    return trajectory;
  }

  // Build truncated trajectory: all points up to bestIdx, plus the target position
  const truncated = trajectory.slice(0, bestIdx + 1);

  // If t > 0, add the interpolated point (which should be close to targetPos)
  // For simplicity, just add the target position itself to ensure trajectory ends exactly there
  truncated.push([...targetPos]);

  return truncated;
}

function split_polyline_at_distance(points, dist) {
  // Defensive defaults
  const n = Array.isArray(points) ? points.length : 0;

  if (n === 0) {
    return {
      prefixPoints: [],
      suffixPoints: [],
      splitPoint: null,
      totalDistance: 0,
      splitIndex: -1,
      t: 0,
    };
  }

  if (n === 1) {
    // Only one point: prefix/suffix are identical
    return {
      prefixPoints: [points[0]],
      suffixPoints: [points[0]],
      splitPoint: points[0],
      totalDistance: 0,
      splitIndex: 0,
      t: 0,
    };
  }

  // Build cumulative distances
  const cum = [0];
  let total = 0;

  for (let i = 1; i < n; i++) {
    const dx = points[i][0] - points[i - 1][0];
    const dy = points[i][1] - points[i - 1][1];
    total += Math.sqrt(dx * dx + dy * dy);
    cum.push(total);
  }

  // Clamp dist into [0, total]
  let d = dist;
  if (!Number.isFinite(d)) d = 0;
  d = Math.max(0, Math.min(d, total));

  // Edge: split at start
  if (d === 0) {
    const sp = points[0];
    return {
      prefixPoints: [sp],
      suffixPoints: [sp, ...points.slice(1)],
      splitPoint: sp,
      totalDistance: total,
      splitIndex: 0,
      t: 0,
    };
  }

  // Edge: split at end
  if (d === total) {
    const sp = points[n - 1];
    return {
      prefixPoints: [...points],
      suffixPoints: [sp],
      splitPoint: sp,
      totalDistance: total,
      splitIndex: n - 2, // last segment
      t: 1,
    };
  }

  // Find segment containing distance d:
  // cum[i] <= d < cum[i+1]
  let seg = 0;
  for (let i = 0; i < n - 1; i++) {
    if (cum[i] <= d && d < cum[i + 1]) {
      seg = i;
      break;
    }
  }

  const segStart = points[seg];
  const segEnd = points[seg + 1];
  const segLen = cum[seg + 1] - cum[seg];

  // Interpolation parameter along the segment
  const t = segLen > 0 ? (d - cum[seg]) / segLen : 0;

  const splitPoint = [
    segStart[0] + t * (segEnd[0] - segStart[0]),
    segStart[1] + t * (segEnd[1] - segStart[1]),
  ];

  // Construct prefix:
  // include points[0..seg] plus splitPoint (unless it equals points[seg] exactly)
  const prefixPoints = points.slice(0, seg + 1);
  const lastPrefix = prefixPoints[prefixPoints.length - 1];
  if (lastPrefix[0] !== splitPoint[0] || lastPrefix[1] !== splitPoint[1]) {
    prefixPoints.push(splitPoint);
  }

  // Construct suffix:
  // start with splitPoint, then include points[seg+1..end]
  const suffixPoints = [splitPoint, ...points.slice(seg + 1)];

  return {
    prefixPoints,
    suffixPoints,
    splitPoint,
    totalDistance: total,
    splitIndex: seg,
    t,
  };
}


// ----------------------------------------------------
// Canvas sizing
// ----------------------------------------------------
function resizeCanvasToContainer() {
  const canvas = $("env-canvas");
  if (!canvas) return;

  // Keep fixed 800x800 internal resolution for quality
  // The CSS will handle visual scaling
  canvas.width = 800;
  canvas.height = 800;

  drawEnvironment();
}

// ----------------------------------------------------
// Environment drawing
// ----------------------------------------------------

// Helper: Draw a diamond cut marker with optional label
// isLost: if true, draw a larger solid red diamond (drone lost at this checkpoint)
function drawCutMarker(ctx, mx, my, label, isLost = false) {
  const r = isLost ? 12 : 8;  // Larger sizes: lost=12, normal=8

  ctx.beginPath();
  ctx.moveTo(mx, my - r);      // top
  ctx.lineTo(mx + r, my);      // right
  ctx.lineTo(mx, my + r);      // bottom
  ctx.lineTo(mx - r, my);      // left
  ctx.closePath();

  if (isLost) {
    // Lost drone: red diamond outline (not filled)
    ctx.strokeStyle = "#ef4444";  // red-500
    ctx.lineWidth = 3;
    ctx.stroke();
  } else {
    // Normal: white diamond outline
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2.5;
    ctx.stroke();
  }

  // Optional label
  if (label) {
    ctx.font = "bold 10px system-ui";
    ctx.fillStyle = isLost ? "#ef4444" : "#ffffff";
    ctx.fillText(label, mx + r + 4, my + 4);
  }
}

let _drawEnvFrameCount = 0;
function drawEnvironment(fromAnimationLoop = false) {
  const canvas = $("env-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  // During animation, ONLY allow draws from the animation loop itself
  // This prevents redraw storms from other sources (event handlers, timers, etc.)
  const isAnimating = missionState.mode === MissionMode.ANIMATING || state.animation.active;
  if (isAnimating && !fromAnimationLoop) {
    return; // Skip non-animation-loop draws during animation
  }

  // Debug: Log visited targets every 120 frames (about once per second at 30fps)
  _drawEnvFrameCount++;
  if (_drawEnvFrameCount % 120 === 1 && state.visitedTargets.length > 0) {
  }

  // CRITICAL: Clear the ENTIRE canvas before drawing anything
  // Use identity transform to ensure we clear the full canvas regardless of any transforms
  ctx.save();
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.restore();

  // If no environment yet, draw placeholder grid
  if (!state.env) {
    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    ctx.strokeStyle = "#1f2937";
    ctx.lineWidth = 0.5;
    for (let x = 0; x < canvas.width; x += 30) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, canvas.height);
      ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += 30) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width, y);
      ctx.stroke();
    }
    return;
  }

  const airports = state.env.airports || [];
  const targets = state.env.targets || [];
  const sams = state.env.sams || [];
  const gridSize = 100.0; // world coordinates 0..100

  // Debug: log target count being drawn (throttled to avoid spam)
  if (_drawEnvFrameCount % 120 === 1) {
  }

  // world -> canvas coordinates
  function w2c(x, y) {
    const pad = 20;
    const w = canvas.width - 2 * pad;
    const h = canvas.height - 2 * pad;
    const sx = pad + (x / gridSize) * w;
    const sy = canvas.height - (pad + (y / gridSize) * h);
    return [sx, sy];
  }

  // Background
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Grid
  ctx.strokeStyle = "#1f2937";
  ctx.lineWidth = 0.5;
  for (let i = 0; i <= 10; i++) {
    const t = (i / 10) * gridSize;

    const [x1, y1] = w2c(t, 0);
    const [x2, y2] = w2c(t, gridSize);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();

    const [xa, ya] = w2c(0, t);
    const [xb, yb] = w2c(gridSize, t);
    ctx.beginPath();
    ctx.moveTo(xa, ya);
    ctx.lineTo(xb, yb);
    ctx.stroke();
  }

  // Helper to find world position by label
  function findWaypointPosition(label) {
    const id = String(label);
    const a = airports.find((x) => String(x.id) === id);
    if (a) return { x: a.x, y: a.y };
    const t = targets.find((x) => String(x.id) === id);
    if (t) return { x: t.x, y: t.y };
    return null;
  }

  // SAMs
  sams.forEach((sam, idx) => {
    const pos = sam.pos || sam.position;
    if (!pos || pos.length !== 2) return;
    const [sx, sy] = w2c(pos[0], pos[1]);
    const r = Number(sam.range || 0);
    const rpx = (r / gridSize) * (canvas.width - 40);

    ctx.beginPath();
    ctx.arc(sx, sy, rpx, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(234,179,8,0.18)";
    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = 1;
    ctx.fill();
    ctx.stroke();

    const size = 7;
    ctx.beginPath();
    ctx.moveTo(sx, sy - size);
    ctx.lineTo(sx - size, sy + size);
    ctx.lineTo(sx + size, sy + size);
    ctx.closePath();
    ctx.fillStyle = "#f97316";
    ctx.strokeStyle = "#c2410c";
    ctx.fill();
    ctx.stroke();

    // Draw SAM number label (S0, S1, S2, ...)
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 10px system-ui";
    ctx.fillText(`S${idx}`, sx + 10, sy + 4);

    // Highlight if selected - show dashed boundary
    if (
      state.selectedObject &&
      state.selectedObject.kind === "sam" &&
      state.selectedObject.index === idx
    ) {
      ctx.beginPath();
      ctx.arc(sx, sy, rpx, 0, Math.PI * 2);
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  });

  // Wrapped SAM Polygons - draw dashed yellow line around wrapped SAMs
  if (state.wrappedPolygons && state.wrappedPolygons.length > 0) {
    state.wrappedPolygons.forEach((polygon) => {
      if (!polygon || polygon.length < 3) return;

      ctx.beginPath();
      const firstPoint = w2c(polygon[0][0], polygon[0][1]);
      ctx.moveTo(firstPoint[0], firstPoint[1]);

      for (let i = 1; i < polygon.length; i++) {
        const point = w2c(polygon[i][0], polygon[i][1]);
        ctx.lineTo(point[0], point[1]);
      }
      ctx.closePath();

      // Dashed yellow line
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 1;
      ctx.setLineDash([5, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }

  // Airports and Checkpoint Markers (synthetic starts)
  airports.forEach((a, idx) => {
    const [x, y] = w2c(a.x, a.y);

    // Check if this is a synthetic start (checkpoint marker)
    const isSyntheticStart = a.id && (a.id.includes("_START") || a.id.startsWith("D") && a.id.endsWith("_START"));

    // Use smaller size and white border for checkpoint markers
    const size = isSyntheticStart ? 5 : 10;  // Checkpoint markers are half size (5 vs 10)
    const fillColor = "#3b82f6";  // Same blue for both
    const strokeColor = isSyntheticStart ? "#ffffff" : "#1d4ed8";  // White border for checkpoints
    const strokeWidth = isSyntheticStart ? 2 : 1;  // Thicker white border for checkpoints

    ctx.fillStyle = fillColor;
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = strokeWidth;
    ctx.beginPath();
    ctx.rect(x - size, y - size, size * 2, size * 2);
    ctx.fill();
    ctx.stroke();

    // Draw label (smaller font for checkpoint markers)
    ctx.fillStyle = "#e5e7eb";
    ctx.font = isSyntheticStart ? "8px system-ui" : "10px system-ui";
    ctx.fillText(String(a.id), x + 8, y - 8);

    // Highlight if selected
    if (
      state.selectedObject &&
      state.selectedObject.kind === "airport" &&
      state.selectedObject.index === idx
    ) {
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2;
      ctx.strokeRect(x - size - 3, y - size - 3, (size * 2) + 6, (size * 2) + 6);
    }
  });

  // Helper function to check if a point is inside any SAM polygon
  function isInsideAnySAM(px, py) {
    for (const sam of sams) {
      const pos = sam.pos || sam.position;
      if (!pos || pos.length !== 2) continue;

      const samX = pos[0];
      const samY = pos[1];
      const range = Number(sam.range || 0);

      // Calculate distance from point to SAM center
      const dx = px - samX;
      const dy = py - samY;
      const distance = Math.sqrt(dx * dx + dy * dy);

      // If inside this SAM's range, return true
      if (distance < range) {
        return true;
      }
    }
    return false;
  }

  // Targets
  targets.forEach((t, idx) => {
    const [x, y] = w2c(t.x, t.y);
    const r = 4;
    const type = (t.type || "a").toLowerCase();
    let color = "#93c5fd";
    if (type === "b") color = "#facc15";
    if (type === "c") color = "#fb923c";
    if (type === "d") color = "#ef4444";
    if (type === "e") color = "#b91c1c";

    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.strokeStyle = "#111827";
    ctx.lineWidth = 1;
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#e5e7eb";
    ctx.font = "9px system-ui";
    const priority = t.priority || 5;
    ctx.fillText(`${t.id}-${priority}`, x + 6, y + 3);

    // Draw green X if target is visited (completed in previous checkpoint segment)
    const isVisited = state.visitedTargets && state.visitedTargets.includes(t.id);
    if (isVisited) {
      ctx.strokeStyle = "#22c55e";  // green
      ctx.lineWidth = 3;
      const xSize = 10;
      ctx.beginPath();
      ctx.moveTo(x - xSize, y - xSize);
      ctx.lineTo(x + xSize, y + xSize);
      ctx.moveTo(x + xSize, y - xSize);
      ctx.lineTo(x - xSize, y + xSize);
      ctx.stroke();
    }
    // Draw red X if target is inside a SAM polygon (no-fly zone)
    else if (isInsideAnySAM(t.x, t.y)) {
      ctx.strokeStyle = "#ef4444";
      ctx.lineWidth = 2;
      const xSize = 8;
      ctx.beginPath();
      ctx.moveTo(x - xSize, y - xSize);
      ctx.lineTo(x + xSize, y + xSize);
      ctx.moveTo(x + xSize, y - xSize);
      ctx.lineTo(x - xSize, y + xSize);
      ctx.stroke();
    }

    if (
      state.selectedObject &&
      state.selectedObject.kind === "target" &&
      state.selectedObject.index === idx
    ) {
      ctx.beginPath();
      ctx.arc(x, y, r + 5, 0, Math.PI * 2);
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  });

  // Routes (per drone) - only draw if trajectory is visible
  const colors = {
    "1": "#3b82f6",
    "2": "#f97316",
    "3": "#22c55e",
    "4": "#ef4444",
    "5": "#a855f7",
  };

  // Debug: Log trajectory info once per draw cycle (throttled)
  if (_drawEnvFrameCount % 120 === 1) {
    const routeKeys = Object.keys(state.routes || {});
    const visibleKeys = Object.keys(state.trajectoryVisible || {}).filter(k => state.trajectoryVisible[k]);
    Object.entries(state.routes || {}).forEach(([did, info]) => {
      const trajLen = info.trajectory ? info.trajectory.length : 0;
      const hasFullTraj = info._fullTrajectory ? info._fullTrajectory.length : 0;
      const isVisible = state.trajectoryVisible[did] ? 'YES' : 'NO';
      if (trajLen > 0) {
        const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
        const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
      } else {
      }
    });
  }

  Object.entries(state.routes || {}).forEach(([did, info]) => {
    // Check if this trajectory should be visible
    if (!state.trajectoryVisible[did]) return;

    // Use trajectory if available (contains SAM-avoiding paths), otherwise use route waypoints
    const trajectory = info.trajectory || [];
    const route = info.route || [];

    ctx.beginPath();

    if (trajectory.length > 0) {
      // Draw the full SAM-avoiding trajectory
      trajectory.forEach((point, idx) => {
        const [cx, cy] = w2c(point[0], point[1]);
        if (idx === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      });
    } else if (route.length > 0) {
      // Fallback: draw straight lines between waypoints
      route.forEach((label, idx) => {
        const pos = findWaypointPosition(label);
        if (!pos) return;
        const [cx, cy] = w2c(pos.x, pos.y);
        if (idx === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      });
    }

    ctx.strokeStyle = colors[did] || "#22c55e";
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 3]);
    ctx.stroke();
    ctx.setLineDash([]);
  });

  // Draw animated drones (active animation OR frozen at checkpoint OR drones positioned after replan)
  const shouldDrawDrones = state.animation.active ||
                          state.checkpoint?.active ||
                          (state.animation.drones && Object.keys(state.animation.drones).length > 0);
  if (shouldDrawDrones && state.animation.drones) {
    Object.entries(state.animation.drones).forEach(([did, droneState]) => {
      // Draw if animating OR frozen at checkpoint OR drone has a valid position (including lost drones)
      const isLostDrone = droneState._lostAtCutDistance !== undefined;
      if (!droneState.animating && !state.checkpoint?.active && !isLostDrone && droneState.distanceTraveled === undefined) return;
      if (!state.trajectoryVisible[did]) return;

      const routeInfo = state.routes[did];
      if (!routeInfo) return;

      // Use trajectory if available (SAM-avoiding path), otherwise fall back to route waypoints
      const trajectory = routeInfo.trajectory || [];
      const route = routeInfo.route || [];
      const progress = droneState.progress;

      let droneX, droneY;

      // If drone is lost (stopped at cut marker), draw at the saved cut position (matches marker)
      if (isLostDrone) {
        if (droneState._lostAtPosition) {
          // Use saved cut position to match the red diamond marker
          droneX = droneState._lostAtPosition[0];
          droneY = droneState._lostAtPosition[1];
        } else if (trajectory.length > 0) {
          // Fallback: use end of truncated trajectory
          const lastPt = trajectory[trajectory.length - 1];
          droneX = lastPt[0];
          droneY = lastPt[1];
        } else {
          return; // No position to draw at
        }
      // If frozen at checkpoint, draw drone at the splitPoint (end of prefix)
      } else if (state.checkpoint?.active && state.checkpoint.segments[did]?.splitPoint) {
        const sp = state.checkpoint.segments[did].splitPoint;
        droneX = sp[0];
        droneY = sp[1];
      } else if (trajectory.length >= 2) {
        // Animate along the full SAM-avoiding trajectory using distance-based interpolation
        // This ensures uniform speed regardless of segment count (SAM circles have many short segments)
        const cumulativeDistances = droneState.cumulativeDistances || [];
        const totalDistance = droneState.totalDistance || 0;

        if (totalDistance > 0 && cumulativeDistances.length === trajectory.length) {
          // Find position based on distance traveled (uniform speed)
          const targetDistance = droneState.distanceTraveled;

          // Binary search for the segment containing this distance
          let segmentIdx = 0;
          for (let i = 1; i < cumulativeDistances.length; i++) {
            if (cumulativeDistances[i] >= targetDistance) {
              segmentIdx = i - 1;
              break;
            }
            segmentIdx = i - 1;
          }

          const fromPoint = trajectory[segmentIdx];
          const toPoint = trajectory[segmentIdx + 1] || fromPoint;
          const segmentStartDist = cumulativeDistances[segmentIdx];
          const segmentEndDist = cumulativeDistances[segmentIdx + 1] || segmentStartDist;
          const segmentLength = segmentEndDist - segmentStartDist;

          const segmentProgress = segmentLength > 0
            ? (targetDistance - segmentStartDist) / segmentLength
            : 0;

          droneX = fromPoint[0] + (toPoint[0] - fromPoint[0]) * segmentProgress;
          droneY = fromPoint[1] + (toPoint[1] - fromPoint[1]) * segmentProgress;
        } else {
          // Fallback to segment-based (old behavior)
          const totalSegments = trajectory.length - 1;
          const currentSegmentFloat = progress * totalSegments;
          const currentSegment = Math.min(Math.floor(currentSegmentFloat), totalSegments - 1);
          const segmentProgress = currentSegmentFloat - currentSegment;

          const fromPoint = trajectory[currentSegment];
          const toPoint = trajectory[currentSegment + 1] || fromPoint;

          droneX = fromPoint[0] + (toPoint[0] - fromPoint[0]) * segmentProgress;
          droneY = fromPoint[1] + (toPoint[1] - fromPoint[1]) * segmentProgress;
        }
      } else if (route.length >= 2) {
        // Fallback: animate along waypoint route (straight lines)
        const totalSegments = route.length - 1;
        const currentSegmentFloat = progress * totalSegments;
        const currentSegment = Math.floor(currentSegmentFloat);
        const segmentProgress = currentSegmentFloat - currentSegment;

        if (currentSegment >= totalSegments) return;

        const fromLabel = route[currentSegment];
        const toLabel = route[currentSegment + 1];
        const fromPos = findWaypointPosition(fromLabel);
        const toPos = findWaypointPosition(toLabel);

        if (!fromPos || !toPos) return;

        droneX = fromPos.x + (toPos.x - fromPos.x) * segmentProgress;
        droneY = fromPos.y + (toPos.y - fromPos.y) * segmentProgress;
      } else {
        return;
      }

      const [cx, cy] = w2c(droneX, droneY);

      // Draw drone as a filled circle
      ctx.beginPath();
      ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      ctx.fillStyle = colors[did] || "#22c55e";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.fill();
      ctx.stroke();

      // Draw drone label
      ctx.fillStyle = "#ffffff";
      ctx.font = "bold 8px system-ui";
      ctx.fillText(`D${did}`, cx + 8, cy - 8);
    });
  }

  // ===========================================================================
  // UNIFIED CUT MARKER DRAWING
  // Single source of truth: missionReplay segments
  // Each segment N (for N >= 1) stores cutPositions for cut C(N)
  // lostDrones in segment N = drones that were disabled at cut C(N)
  // ===========================================================================
  console.log(`[CUT MARKERS] Starting cut marker drawing section`);

  const isAtAirport = (x, y) => {
    const airports = state.env?.airports || [];
    const threshold = 0.5;  // Reduced from 1.0 - only skip if VERY close to airport
    return airports.some(a => {
      const dx = a.x - x;
      const dy = a.y - y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < threshold) {
        console.log(`[isAtAirport] Position [${x.toFixed(1)},${y.toFixed(1)}] is ${dist.toFixed(2)} from airport ${a.id} at [${a.x},${a.y}]`);
        return true;
      }
      return false;
    });
  };

  // Collect all markers to draw (deduplicated by position)
  const markersToDrawMap = new Map(); // key: "x,y" -> { label, isLost, did }

  // Draw markers from missionReplay segments (segments 1+ have cutPositions for C1, C2, etc.)
  // Only draw markers up to the current segment we're working on
  const totalSegs = missionReplay.getSegmentCount();
  const currentSegIdx = missionReplay.getCurrentSegmentIndex();

  // After final accept (READY_TO_ANIMATE), show ALL checkpoint markers
  // The currentSegIdx gets reset to 0 for animation, but we should still show all checkpoints
  const showAllCheckpoints = (missionState.mode === MissionMode.READY_TO_ANIMATE);

  // DEBUG: Log marker drawing context
  console.log(`[drawCheckpointMarkers] totalSegs=${totalSegs}, currentSegIdx=${currentSegIdx}, showAll=${showAllCheckpoints}`);

  for (let segIdx = 1; segIdx < totalSegs; segIdx++) {
    // Don't draw markers beyond the current segment (e.g., after Reset)
    // UNLESS we're in READY_TO_ANIMATE mode (all segments accepted)
    if (!showAllCheckpoints && segIdx > currentSegIdx) {
      console.log(`[drawCheckpointMarkers] Skipping seg ${segIdx}: beyond currentSegIdx ${currentSegIdx}`);
      continue;
    }

    const seg = missionReplay.getSegment(segIdx);
    if (!seg?.cutPositions) {
      console.log(`[drawCheckpointMarkers] Skipping seg ${segIdx}: no cutPositions`);
      continue;
    }

    // Only draw if previous segment has a solution (trajectory exists up to this cut)
    const prevSeg = missionReplay.getSegment(segIdx - 1);
    const prevHasSolution = prevSeg && Object.keys(prevSeg.solution?.routes || {}).length > 0;
    if (!prevHasSolution) {
      console.log(`[drawCheckpointMarkers] Skipping seg ${segIdx}: prevSeg has no solution (routes=${Object.keys(prevSeg?.solution?.routes || {}).length})`);
      continue;
    }

    console.log(`[drawCheckpointMarkers] Processing seg ${segIdx}: cutPositions=${JSON.stringify(seg.cutPositions)}`);

    const lostDrones = seg.lostDrones || [];
    Object.entries(seg.cutPositions).forEach(([did, pos]) => {
      if (!pos || pos.length !== 2) return;
      if (isAtAirport(pos[0], pos[1])) {
        console.log(`[drawCheckpointMarkers] Skipping D${did} at seg ${segIdx}: at airport`);
        return;
      }
      const key = `${pos[0].toFixed(4)},${pos[1].toFixed(4)}`;
      const isLost = lostDrones.includes(did);
      markersToDrawMap.set(key, { pos, label: `C${segIdx}`, isLost, did });
      console.log(`[drawCheckpointMarkers] Added marker C${segIdx} for D${did} at [${pos[0].toFixed(1)}, ${pos[1].toFixed(1)}]`);
    });
  }

  // Draw all collected markers (no duplicates)
  markersToDrawMap.forEach(({ pos, label, isLost }) => {
    const [mx, my] = w2c(pos[0], pos[1]);
    drawCutMarker(ctx, mx, my, label, isLost);
  });

  // Target Type Legend (top-right corner)
  const legendTypes = [
    { label: "A", color: "#93c5fd" },
    { label: "B", color: "#facc15" },
    { label: "C", color: "#fb923c" },
    { label: "D", color: "#ef4444" },
    { label: "E", color: "#b91c1c" },
  ];
  const legendX = canvas.width - 22;
  const legendY = 15;
  const dotRadius = 4;
  const lineHeight = 14;

  // Semi-transparent background box
  ctx.fillStyle = "rgba(2, 6, 23, 0.8)";
  ctx.strokeStyle = "#374151";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(legendX - 8, legendY - 8, 28, legendTypes.length * lineHeight + 10, 4);
  ctx.fill();
  ctx.stroke();

  // Draw each legend item
  legendTypes.forEach((item, i) => {
    const y = legendY + i * lineHeight;
    // Colored dot
    ctx.beginPath();
    ctx.arc(legendX, y + 4, dotRadius, 0, Math.PI * 2);
    ctx.fillStyle = item.color;
    ctx.fill();
    // Label
    ctx.fillStyle = "#e5e7eb";
    ctx.font = "bold 10px system-ui";
    ctx.fillText(item.label, legendX + 7, y + 8);
  });
}

// ----------------------------------------------------
// Edit mode toggle
// ----------------------------------------------------
function setEditMode(on) {
  const perms = getUiPermissions();

  if (on) {
    // Entering edit mode
    if (!perms.canEnterEdit) {
      appendDebugLine("Cannot enter edit mode in current state");
      return;
    }
    enterEditMode();
  } else {
    // Exiting edit mode - if currently editing, this cancels edits
    if (missionState.mode === MissionMode.EDITING_ENV) {
      cancelEdits();
    } else {
      // Not in EDITING_ENV, just update the low-level edit flag
      state.editMode = false;
      const btn = $("btn-toggle-edit");
      if (btn) {
        btn.textContent = "View";
        btn.classList.add("off");
      }
      state.addMode = null;
      state.selectedObject = null;
      state.dragging = null;
      drawEnvironment();
    }
  }
}

function attachEditToggle() {
  const btn = $("btn-toggle-edit");
  if (!btn) return;

  // Start with edit mode OFF for the state machine
  state.editMode = false;

  btn.addEventListener("click", () => {
    const perms = getUiPermissions();

    if (missionState.mode === MissionMode.EDITING_ENV) {
      // Currently editing - toggle OFF means ACCEPT edits (save changes)
      // User can use Mission Control "Discard" or reload to cancel
      acceptEdits();
    } else if (perms.canEnterEdit) {
      // Can enter edit mode
      enterEditMode();
    } else {
      appendDebugLine("Cannot toggle edit mode in current state");
    }
  });
}

// ----------------------------------------------------
// Canvas editing: hit-test, drag, add, delete
// ----------------------------------------------------
function setAddMode(kind) {
  state.addMode = kind; // "airport", "target", "sam"
  if (kind) {
    appendDebugLine(`Add mode: ${kind.toUpperCase()} (click on canvas to place)`);
  } else {
    appendDebugLine("Add mode: off");
  }
}

function deleteSelected() {
  if (!state.env || !state.selectedObject) return;

  const { kind, index } = state.selectedObject;
  if (kind === "airport") {
    state.env.airports.splice(index, 1);
    updateAirportDropdowns();
  } else if (kind === "target") {
    state.env.targets.splice(index, 1);
  } else if (kind === "sam") {
    state.env.sams.splice(index, 1);
    // Recalculate wrapping after SAM deletion
    updateSamWrappingClientSide();
  }

  state.selectedObject = null;
  appendDebugLine(`Deleted ${kind}.`);
  drawEnvironment();
}

/**
 * Export environment to JSON file
 * Two modes based on radio button selection:
 * - Mission: exports segments with env + cut distances (no solutions), filename _Nx_
 * - Environment: exports flat env with all targets combined, filename _N1_
 */
// Export counter for unique filenames
let _exportCounter = 1;

/**
 * Export mission using segmentStore (clean, simple format)
 * @returns {boolean} True if export succeeded
 */
function exportSegmentStore() {
  if (segmentStore.count() === 0) {
    appendDebugLine(`⚠️ [SegmentStore] Cannot export - no segments`);
    return false;
  }

  try {
    const exportData = segmentStore.toJSON();

    // Generate filename
    const now = new Date();
    const yy = String(now.getFullYear()).slice(-2);
    const mo = String(now.getMonth() + 1).padStart(2, "0");
    const dd = String(now.getDate()).padStart(2, "0");
    const hh = String(now.getHours()).padStart(2, "0");
    const mm = String(now.getMinutes()).padStart(2, "0");
    const segCount = segmentStore.count();
    const filename = `isr_seg_${yy}${mo}${dd}${hh}${mm}_N${segCount}_${_exportCounter}.json`;
    _exportCounter++;

    // Download
    const jsonStr = JSON.stringify(exportData, null, 2);
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    appendDebugLine(`✅ [SegmentStore] Exported ${segCount} segments as ${filename}`);
    return true;
  } catch (err) {
    console.error('[SegmentStore] Export failed:', err);
    appendDebugLine(`❌ [SegmentStore] Export failed: ${err.message}`);
    return false;
  }
}


async function exportEnvironment() {
  try {
    if (!state.env) {
      alert("No environment loaded.");
      return;
    }

    // Check which export type is selected
    const exportTypeMission = document.getElementById("export-type-mission");
    const isMissionExport = exportTypeMission && exportTypeMission.checked;

    appendDebugLine(`📤 Exporting as ${isMissionExport ? "Mission" : "Environment"}...`);

    // SEGMENT STORE EXPORT: If we have segments in segmentStore, use that (simplest format)
    if (isMissionExport && segmentStore.count() > 0) {
      if (exportSegmentStore()) {
        return;  // SegmentStore export succeeded
      }
      appendDebugLine(`⚠️ SegmentStore export failed, using legacy format...`);
    }

    // Always capture current drone configs into env snapshot
    const envSnapshot = JSON.parse(JSON.stringify(state.env));
    envSnapshot.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs || envSnapshot.drone_configs || {}));

    // Debug: Log current drone configs being exported
    Object.entries(state.droneConfigs || {}).forEach(([did, cfg]) => {
      if (cfg.enabled) {
        appendDebugLine(`   D${did}: ${cfg.start_airport}→${cfg.end_airport}, fuel=${cfg.fuel_budget}`);
      }
    });

    // Check if MissionReplay has segments
    const segments = (typeof missionReplay !== "undefined" && missionReplay.getSegmentCount)
      ? missionReplay.getSegments()
      : [];

    // Build export object
    let exportObj;
    let segmentCount;

    if (isMissionExport && segments && segments.length > 1) {
      // MISSION export - export segments with env, cut distances, cut positions (NO solutions)
      segmentCount = segments.length;

      // DEBUG: Log all segment drone_configs before export
      segments.forEach((s, i) => {
        const hasDroneConfigs = !!s.drone_configs;
        const enabled = s.drone_configs
          ? Object.entries(s.drone_configs).filter(([,c])=>c.enabled).map(([d])=>`D${d}`).join(',')
          : 'MISSING';
      });
      // Use segment 0's drone_configs as the top-level default
      const seg0Configs = segments[0]?.drone_configs || state.droneConfigs || {};

      exportObj = {
        schema: "isr_env_v1",
        is_segmented: true,
        segment_count: segmentCount,
        drone_configs: JSON.parse(JSON.stringify(seg0Configs)),  // Top-level uses segment 0's config
        segments: segments.map(s => {
          // IMPORTANT: Each segment gets its OWN drone_configs that was saved when it was accepted
          const segDroneConfigs = s.drone_configs || state.droneConfigs || {};
          const enabled = Object.entries(segDroneConfigs).filter(([,c])=>c.enabled).map(([d])=>`D${d}`).join(',');
          return {
            index: s.index,
            env: s.env,
            cutDistance: s.cutDistance || null,
            cutPositions: s.cutPositions || null,  // Save exact cut positions for marker display
            lostDrones: s.lostDrones || null,  // Drones that crashed/were lost at this checkpoint
            drone_configs: JSON.parse(JSON.stringify(segDroneConfigs)),  // Per-segment config
            // NO solution - user re-solves each segment to allow edits/optimizations
          };
        }),
      };
      appendDebugLine(`   ${segmentCount} segments with cut distances and positions (no solutions)`);
    } else {
      // ENVIRONMENT export - combine all targets into flat env, no segments
      segmentCount = 1;

      // If we have segments, merge all targets from all segments
      let combinedEnv = JSON.parse(JSON.stringify(envSnapshot));
      if (segments && segments.length > 1) {
        const allTargets = new Map();
        const allSams = new Map();
        const allAirports = new Map();

        segments.forEach(seg => {
          if (!seg.env) return;
          (seg.env.targets || []).forEach(t => allTargets.set(t.id, t));
          (seg.env.sams || []).forEach((s) => {
            const key = s.id || `sam_${s.pos?.[0]}_${s.pos?.[1]}`;
            allSams.set(key, s);
          });
          (seg.env.airports || []).forEach(a => allAirports.set(a.id, a));
        });

        combinedEnv.targets = Array.from(allTargets.values());
        combinedEnv.sams = Array.from(allSams.values());
        combinedEnv.airports = Array.from(allAirports.values());
        appendDebugLine(`   Combined ${combinedEnv.targets.length} targets from ${segments.length} segments`);
      }

      exportObj = {
        schema: "isr_env_v1",
        is_segmented: false,
        segment_count: 1,
        env: combinedEnv,
      };
    }

    // Filename with segment count: _Nx_ where x is segment count
    const now = new Date();
    const yy = String(now.getFullYear()).slice(-2);
    const mo = String(now.getMonth() + 1).padStart(2, "0");
    const dd = String(now.getDate()).padStart(2, "0");
    const hh = String(now.getHours()).padStart(2, "0");
    const mm = String(now.getMinutes()).padStart(2, "0");

    const filename = `isr_env${yy}${mo}${dd}${hh}${mm}_N${segmentCount}_${_exportCounter}.json`;
    _exportCounter++;

    // Download
    const blob = new Blob([JSON.stringify(exportObj, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    appendDebugLine(`✅ Exported as ${filename}`);
  } catch (err) {
    appendDebugLine(`❌ Export error: ${err.message}`);
  }
}

// world <-> canvas helpers for editing
function canvasToWorld(canvas, clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const cx = (clientX - rect.left) * scaleX;
  const cy = (clientY - rect.top) * scaleY;

  const pad = 20;
  const w = canvas.width - 2 * pad;
  const h = canvas.height - 2 * pad;
  const gridSize = 100.0;

  const xWorld = ((cx - pad) / w) * gridSize;
  const yWorld = ((canvas.height - cy - pad) / h) * gridSize;

  return [xWorld, yWorld];
}

// hit-test on env (airport/target/sam)
function hitTestWorld(x, y) {
  if (!state.env) return null;
  const airports = state.env.airports || [];
  const targets = state.env.targets || [];
  const sams = state.env.sams || [];
  const R = 4.0; // threshold in world units (approx)

  function dist2(ax, ay, bx, by) {
    const dx = ax - bx;
    const dy = ay - by;
    return dx * dx + dy * dy;
  }

  let best = null;
  let bestD2 = Infinity;

  airports.forEach((a, idx) => {
    const d2 = dist2(x, y, a.x, a.y);
    if (d2 < bestD2 && d2 < R * R) {
      bestD2 = d2;
      best = { kind: "airport", index: idx };
    }
  });

  targets.forEach((t, idx) => {
    const d2 = dist2(x, y, t.x, t.y);
    if (d2 < bestD2 && d2 < R * R) {
      bestD2 = d2;
      best = { kind: "target", index: idx };
    }
  });

  sams.forEach((s, idx) => {
    const pos = s.pos || s.position;
    if (!pos || pos.length !== 2) return;
    const d2 = dist2(x, y, pos[0], pos[1]);
    const thr = 3.0;
    if (d2 < bestD2 && d2 < thr * thr) {
      bestD2 = d2;
      best = { kind: "sam", index: idx };
    }
  });

  return best;
}

function attachCanvasEditing() {
  const canvas = $("env-canvas");
  if (!canvas) return;

  canvas.addEventListener("mousedown", (evt) => {
    if (!state.env || !state.editMode) return;

    const [wx, wy] = canvasToWorld(canvas, evt.clientX, evt.clientY);

    // If we are in "add mode", create an object here
    if (state.addMode) {
      if (state.addMode === "airport") {
        const airports = state.env.airports || (state.env.airports = []);
        const nextNum =
          airports
            .map((a) => {
              const m = String(a.id).match(/^A(\d+)$/i);
              return m ? parseInt(m[1], 10) : 0;
            })
            .reduce((a, b) => Math.max(a, b), 0) + 1;
        airports.push({ id: `A${nextNum}`, x: wx, y: wy });
        appendDebugLine(`Added Airport A${nextNum} at (${wx.toFixed(1)}, ${wy.toFixed(1)})`);
        updateAirportDropdowns();
      } else if (state.addMode === "target") {
        const targets = state.env.targets || (state.env.targets = []);

        // Find max target number across current env AND all segmented mission segments
        // This prevents ID collisions when merging edits forward
        let maxTargetNum = 0;

        // Check current env targets
        targets.forEach((t) => {
          const m = String(t.id).match(/^T(\d+)$/i);
          if (m) maxTargetNum = Math.max(maxTargetNum, parseInt(m[1], 10));
        });

        // Check all segments in segmented mission (if any)
        if (missionState.segmentedMission && missionState.segmentedMission.segments) {
          missionState.segmentedMission.segments.forEach((seg) => {
            (seg.env?.targets || []).forEach((t) => {
              const m = String(t.id).match(/^T(\d+)$/i);
              if (m) maxTargetNum = Math.max(maxTargetNum, parseInt(m[1], 10));
            });
          });
        }

        const nextNum = maxTargetNum + 1;

        // Read type and priority from dropdowns
        const typeSelect = $("new-target-type");
        const prioritySelect = $("new-target-priority");
        const targetType = typeSelect ? typeSelect.value : "a";
        const targetPriority = prioritySelect ? parseInt(prioritySelect.value, 10) : 5;

        targets.push({
          id: `T${nextNum}`,
          x: wx,
          y: wy,
          priority: targetPriority,
          type: targetType,
        });
        appendDebugLine(`Added Target T${nextNum} (type=${targetType.toUpperCase()}, priority=${targetPriority}) at (${wx.toFixed(1)}, ${wy.toFixed(1)})`);
      } else if (state.addMode === "sam") {
        const sams = state.env.sams || (state.env.sams = []);

        // Read range from dropdown
        const rangeSelect = $("new-sam-range");
        const samRange = rangeSelect ? parseInt(rangeSelect.value, 10) : 15;

        sams.push({
          pos: [wx, wy],
          range: samRange,
        });
        appendDebugLine(`Added SAM (range=${samRange}) at (${wx.toFixed(1)}, ${wy.toFixed(1)})`);

        // Calculate wrapping immediately (client-side) and draw
        updateSamWrappingClientSide();
        state.addMode = null;
        drawEnvironment();
        return;
      }
      // After a single click, clear addMode (or keep it; your choice)
      state.addMode = null;
      drawEnvironment();
      return;
    }

    // Otherwise: hit-test and start dragging if something is hit
    const hit = hitTestWorld(wx, wy);
    state.selectedObject = hit;
    state.dragging = null;

    if (hit) {
      appendDebugLine(`Selected ${hit.kind.toUpperCase()} index ${hit.index}`);
      // Start dragging
      state.dragging = {
        kind: hit.kind,
        index: hit.index,
        offsetX: 0,
        offsetY: 0,
      };
    }

    drawEnvironment();
  });

  canvas.addEventListener("mousemove", (evt) => {
    if (!state.env || !state.editMode) return;
    if (!state.dragging) return;

    const [wx, wy] = canvasToWorld(canvas, evt.clientX, evt.clientY);
    const { kind, index } = state.dragging;

    if (kind === "airport") {
      const airports = state.env.airports || [];
      if (airports[index]) {
        airports[index].x = wx;
        airports[index].y = wy;
      }
    } else if (kind === "target") {
      const targets = state.env.targets || [];
      if (targets[index]) {
        targets[index].x = wx;
        targets[index].y = wy;
      }
    } else if (kind === "sam") {
      const sams = state.env.sams || [];
      if (sams[index]) {
        sams[index].pos = [wx, wy];
        // Update wrapping instantly using client-side calculation
        updateSamWrappingClientSide();
      }
    }

    drawEnvironment();
  });

  // Use window-level mouseup so we ALWAYS end drag, even if the mouse
  // leaves the canvas or some other UI element grabs focus.
  window.addEventListener("mouseup", () => {
    // Always end drag
    const wasDragging = state.dragging !== null;
    state.dragging = null;

    // Redraw if we were dragging something (wrapping already updated client-side during drag)
    if (wasDragging) {
      drawEnvironment();
    }
  });

  // If the window loses focus (e.g. you alt-tab), make sure we are not dragging.
  window.addEventListener("blur", () => {
    state.dragging = null;
  });

  // Note: We intentionally do NOT cancel dragging on mouseleave.
  // The window-level mouseup handler will properly end the drag
  // even if the mouse leaves the canvas, allowing wrapping to update.
}

// ----------------------------------------------------
// Drone config + sequence bar
// ----------------------------------------------------

// Update airport dropdowns when airports are added/deleted
function updateAirportDropdowns() {
  const airports = (state.env && state.env.airports) || [];
  const airportIds = airports.map((a) => String(a.id));

  for (let did = 1; did <= 5; did++) {
    const startSel = $(`cfg-d${did}-start`);
    const endSel = $(`cfg-d${did}-end`);

    // Store current selections
    const currentStart = startSel ? startSel.value : null;
    const currentEnd = endSel ? endSel.value : null;

    // Repopulate start dropdown (no flexible option for start)
    if (startSel) {
      startSel.innerHTML = "";
      airportIds.forEach((aid) => {
        const opt = document.createElement("option");
        opt.value = aid;
        opt.textContent = aid;
        startSel.appendChild(opt);
      });
    }

    // Repopulate end dropdown WITH flexible "-" option
    if (endSel) {
      endSel.innerHTML = "";
      // Add flexible endpoint option first
      const flexOpt = document.createElement("option");
      flexOpt.value = "-";
      flexOpt.textContent = "Any";
      endSel.appendChild(flexOpt);
      // Add airport options
      airportIds.forEach((aid) => {
        const opt = document.createElement("option");
        opt.value = aid;
        opt.textContent = aid;
        endSel.appendChild(opt);
      });
    }

    // Restore selections if they still exist, otherwise use first available
    if (startSel) {
      if (airportIds.includes(currentStart)) {
        startSel.value = currentStart;
      } else if (airportIds.length > 0) {
        startSel.value = airportIds[0];
        state.droneConfigs[String(did)].start_airport = airportIds[0];
      }
    }

    if (endSel) {
      // Check if current selection is valid (flexible "-" or a known airport)
      if (currentEnd === "-" || airportIds.includes(currentEnd)) {
        endSel.value = currentEnd;
      } else if (airportIds.length > 0) {
        endSel.value = airportIds[0];
        state.droneConfigs[String(did)].end_airport = airportIds[0];
      }
    }
  }
}

/**
 * Refresh the drone config UI from state.droneConfigs.
 * Called during segment switches to update the UI to match the segment's stored configs.
 */
function refreshDroneConfigUI() {
  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const cfg = state.droneConfigs[idStr];
    if (!cfg) continue;

    const cbEnabled = $(`cfg-d${did}-enabled`);
    const fuelInput = $(`cfg-d${did}-fuel`);

    if (cbEnabled) cbEnabled.checked = cfg.enabled;
    if (fuelInput) fuelInput.value = cfg.fuel_budget || cfg.fuel_capacity || 150;
  }
}

/**
 * Read drone configs directly from the UI table.
 * This is the source of truth at accept time - what the user sees is what gets saved.
 * @returns {object} drone_configs object with enabled, fuel_budget, etc.
 */
function readDroneConfigsFromUI() {
  const configs = {};
  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const cbEnabled = $(`cfg-d${did}-enabled`);
    const fuelInput = $(`cfg-d${did}-fuel`);
    const startSel = $(`cfg-d${did}-start`);
    const endSel = $(`cfg-d${did}-end`);

    // Start with existing config as base (for fields not in UI)
    const baseConfig = state.droneConfigs[idStr] || {};

    configs[idStr] = {
      ...baseConfig,
      enabled: cbEnabled ? cbEnabled.checked : true,
      fuel_budget: fuelInput ? parseFloat(fuelInput.value) || 150 : 150,
      fuel_capacity: fuelInput ? parseFloat(fuelInput.value) || 150 : 150,
      start_airport: startSel ? startSel.value : baseConfig.start_airport,
      end_airport: endSel ? endSel.value : baseConfig.end_airport,
    };
  }
  return configs;
}

function initDroneConfigsFromEnv() {
  const airports = (state.env && state.env.airports) || [];
  const airportIds = airports.map((a) => String(a.id));
  const defaultAirport = airportIds[0] || "A1";

  // Check if drone_configs are saved in the environment
  const savedConfigs = (state.env && state.env.drone_configs) || {};

  function populateStartSelect(selectId) {
    const sel = $(selectId);
    if (!sel) return;
    sel.innerHTML = "";
    airportIds.forEach((aid) => {
      const opt = document.createElement("option");
      opt.value = aid;
      opt.textContent = aid;
      sel.appendChild(opt);
    });
  }

  function populateEndSelect(selectId) {
    const sel = $(selectId);
    if (!sel) return;
    sel.innerHTML = "";
    // Add flexible endpoint option first
    const flexOpt = document.createElement("option");
    flexOpt.value = "-";
    flexOpt.textContent = "Any";
    sel.appendChild(flexOpt);
    // Add airport options
    airportIds.forEach((aid) => {
      const opt = document.createElement("option");
      opt.value = aid;
      opt.textContent = aid;
      sel.appendChild(opt);
    });
  }

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const saved = savedConfigs[idStr] || {};

    // Use saved config if available, otherwise use defaults
    const enabledDefault = saved.enabled !== undefined ? saved.enabled : true;
    const fuelDefault = saved.fuel_budget !== undefined ? saved.fuel_budget : 150;
    const startDefault = saved.start_airport || (airportIds.includes(`A${did}`) ? `A${did}` : defaultAirport);
    const endDefault = saved.end_airport || (airportIds.includes(`A${did}`) ? `A${did}` : defaultAirport);
    const accessDefault = saved.target_access || { a: true, b: true, c: true, d: true, e: true };

    state.droneConfigs[idStr] = {
      enabled: enabledDefault,
      fuel_budget: fuelDefault,
      start_airport: startDefault,
      end_airport: endDefault,
      target_access: accessDefault,
    };

    populateStartSelect(`cfg-d${did}-start`);
    populateEndSelect(`cfg-d${did}-end`);

    const cfg = state.droneConfigs[idStr];
    const cbEnabled = $(`cfg-d${did}-enabled`);
    const fuelInput = $(`cfg-d${did}-fuel`);
    const startSel = $(`cfg-d${did}-start`);
    const endSel = $(`cfg-d${did}-end`);

    if (cbEnabled) cbEnabled.checked = cfg.enabled;
    if (fuelInput) fuelInput.value = cfg.fuel_budget;
    if (startSel && cfg.start_airport) startSel.value = cfg.start_airport;
    if (endSel && cfg.end_airport) endSel.value = cfg.end_airport;

    // Update target access checkboxes
    ["a", "b", "c", "d", "e"].forEach((t) => {
      const cb = $(`cfg-d${did}-type-${t}`);
      if (cb && cfg.target_access[t] !== undefined) {
        cb.checked = cfg.target_access[t];
      }
    });
  }
}

function attachConfigListeners() {
  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);

    const cbEnabled = $(`cfg-d${did}-enabled`);
    const fuelInput = $(`cfg-d${did}-fuel`);
    const startSel = $(`cfg-d${did}-start`);
    const endSel = $(`cfg-d${did}-end`);

    if (cbEnabled) {
      cbEnabled.addEventListener("change", () => {
        state.droneConfigs[idStr].enabled = cbEnabled.checked;
        invalidateMission(`Drone ${idStr} enabled changed`);
      });
    }

    if (fuelInput) {
      fuelInput.addEventListener("change", () => {
        const v = parseFloat(fuelInput.value || "0");
        state.droneConfigs[idStr].fuel_budget = isNaN(v) ? 0 : v;
        // Update stats to reflect new fuel budget
        updateStatsFromRoutes();
        invalidateMission(`Drone ${idStr} fuel_budget changed`);
      });
    }

    if (startSel) {
      startSel.addEventListener("change", () => {
        state.droneConfigs[idStr].start_airport = startSel.value;
        invalidateMission(`Drone ${idStr} start_airport changed`);
      });
    }

    if (endSel) {
      endSel.addEventListener("change", () => {
        state.droneConfigs[idStr].end_airport = endSel.value;
        invalidateMission(`Drone ${idStr} end_airport changed`);
      });
    }

    ["a", "b", "c", "d", "e"].forEach((t) => {
      const cb = $(`cfg-d${did}-type-${t}`);
      if (!cb) return;
      cb.addEventListener("change", () => {
        state.droneConfigs[idStr].target_access[t] = cb.checked;
        invalidateMission(`Drone ${idStr} target_access changed`);
      });
    });
  }
}


function attachSequenceBar() {
  const droneSel = $("sequence-drone-select");
  const seqInput = $("sequence-input");
  const applyBtn = $("sequence-apply-btn");

  if (!droneSel || !seqInput || !applyBtn) return;

  droneSel.addEventListener("change", () => {
    state.currentDroneForSeq = droneSel.value;
    const seq = state.sequences[state.currentDroneForSeq] || "";
    seqInput.value = seq;
  });

  applyBtn.addEventListener("click", async () => {
    const seqText = seqInput.value.trim().toUpperCase();
    const droneId = state.currentDroneForSeq;

    if (!seqText) {
      appendDebugLine(`Empty sequence for D${droneId}`);
      return;
    }

    if (!state.env) {
      appendDebugLine(`No environment loaded - cannot apply sequence`);
      return;
    }

    // Get fuel budget from drone config
    const droneConfig = state.droneConfigs[droneId] || {};
    const fuelBudget = droneConfig.fuel_budget || 300;

    appendDebugLine(`Applying sequence for D${droneId}: ${seqText}`);

    try {
      const res = await fetch("/api/apply_sequence", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          drone_id: droneId,
          sequence: seqText,
          env: state.env,
          fuel_budget: fuelBudget
        })
      });

      const data = await res.json();

      if (data.success) {
        // Update state with new route data
        state.sequences[droneId] = seqText;
        state.routes[droneId] = {
          route: data.route,
          sequence: seqText,
          points: data.points,
          distance: data.distance,
          fuel_budget: fuelBudget,
          trajectory: data.trajectory
        };

        appendDebugLine(
          `Applied D${droneId}: ${data.route.length} waypoints, ${Math.round(data.distance)} fuel, ${data.points} pts`
        );

        // Redraw canvas with new trajectory
        drawEnvironment();

        // Update stats display
        updateStatsFromRoutes();
      } else {
        appendDebugLine(`Error applying sequence: ${data.error || "Unknown error"}`);
      }
    } catch (err) {
      appendDebugLine(`Failed to apply sequence: ${err.message}`);
    }
  });
}

// ----------------------------------------------------
// Stats update
// ----------------------------------------------------
function updateStatsFromRoutes() {
  const envTargets = (state.env && state.env.targets) || [];
  const totalTargets = envTargets.length; // reserved if you need it

  let missionPoints = 0;
  let missionFuel = 0;
  let missionBudget = 0;
  let missionVisited = 0;

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const routeInfo = state.routes[idStr] || {};
    const droneConfig = state.droneConfigs[idStr] || {};
    const route = routeInfo.route || [];
    const points = Number(routeInfo.points || 0);
    const distance = Number(routeInfo.distance || 0);
    // Use fuel budget from droneConfigs (Config tab) as the source of truth
    // Fall back to route info if config not available
    const budget = Number(droneConfig.fuel_budget || routeInfo.fuel_budget || 0);

    const targetCount = route.filter((label) =>
      String(label).toUpperCase().startsWith("T")
    ).length;

    const fuelUsed = distance; // for now, assume distance == fuel
    const pf = fuelUsed > 0 ? points / fuelUsed : 0;

    setText(`stat-d${did}-tp`, `${targetCount} / ${points}`);
    setText(`stat-d${did}-fuel`, `${Math.round(fuelUsed)} / ${Math.round(budget)}`);
    setText(`stat-d${did}-pf`, pf.toFixed(2));

    if (route && route.length > 0) {
      missionPoints += points;
      missionFuel += fuelUsed;
      missionBudget += budget;
      missionVisited += targetCount;
    }
  }

  const missionPf = missionFuel > 0 ? missionPoints / missionFuel : 0;
  setText("stat-mission-tp", `${missionVisited} / ${missionPoints}`);
  setText(
    "stat-mission-fuel",
    `${Math.round(missionFuel)} / ${Math.round(missionBudget)}`
  );
  setText("stat-mission-pf", missionPf.toFixed(2));
}

/**
 * Update stats from actual animation state (real-time distance traveled)
 * This shows the actual fuel/distance used during animation, not the planned values
 */
function updateStatsFromAnimation() {
  if (!state.animation || !state.animation.drones) return;

  let missionDistanceTraveled = 0;
  let missionBudget = 0;
  let missionPoints = 0;
  let missionVisited = 0;

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const droneState = state.animation.drones[idStr];
    const droneConfig = state.droneConfigs[idStr] || {};
    const routeInfo = state.routes[idStr] || {};
    const route = routeInfo.route || [];
    const points = Number(routeInfo.points || 0);
    const budget = Number(droneConfig.fuel_budget || routeInfo.fuel_budget || 0);

    // Get actual distance traveled (use animation state if available, else route distance)
    let distanceTraveled = 0;
    if (droneState) {
      distanceTraveled = droneState.distanceTraveled || 0;
    } else {
      // Fallback to planned distance if no animation state
      distanceTraveled = Number(routeInfo.distance || 0);
    }

    // Count visited targets for this drone
    const visitedCount = route.filter((label) =>
      String(label).toUpperCase().startsWith("T") && state.visitedTargets.includes(label)
    ).length;

    const pf = distanceTraveled > 0 ? points / distanceTraveled : 0;

    // Update individual drone stats with actual distance
    setText(`stat-d${did}-tp`, `${visitedCount} / ${points}`);
    setText(`stat-d${did}-fuel`, `${Math.round(distanceTraveled)} / ${Math.round(budget)}`);
    setText(`stat-d${did}-pf`, pf.toFixed(2));

    if (route && route.length > 0) {
      missionPoints += points;
      missionDistanceTraveled += distanceTraveled;
      missionBudget += budget;
      missionVisited += visitedCount;
    }
  }

  const missionPf = missionDistanceTraveled > 0 ? missionPoints / missionDistanceTraveled : 0;
  setText("stat-mission-tp", `${missionVisited} / ${missionPoints}`);
  setText("stat-mission-fuel", `${Math.round(missionDistanceTraveled)} / ${Math.round(missionBudget)}`);
  setText("stat-mission-pf", missionPf.toFixed(2));
}

function updateAllocationDisplay(allocations, strategy) {
  const container = $("allocation-display");
  if (!container) return;

  // Handle null/undefined allocations
  const allocs = allocations || {};
  const totalTargets = (state.env && state.env.targets) ? state.env.targets.length : 0;

  // Build allocation from routes if allocations is empty but routes exist
  let effectiveAllocs = allocs;
  if (Object.keys(allocs).length === 0 && state.routes && Object.keys(state.routes).length > 0) {
    effectiveAllocs = {};
    for (const [did, routeData] of Object.entries(state.routes)) {
      const route = routeData.route || [];
      effectiveAllocs[did] = route.filter(wp => String(wp).startsWith('T'));
    }
  }

  let html = '';
  let totalAllocated = 0;

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const targets = effectiveAllocs[idStr] || [];
    const cfg = state.droneConfigs[idStr] || {};

    if (cfg.enabled === false) continue;

    totalAllocated += targets.length;
    const color = targets.length > 0 ? '#4ade80' : '#f87171';
    html += `<div style="color: ${color};">D${did}: ${targets.length > 0 ? targets.join(', ') : '(none)'}</div>`;
  }

  if (totalAllocated === 0 && totalTargets === 0) {
    container.innerHTML = '<div style="color: #888;">No targets in environment</div>';
    return;
  }

  // Add strategy in parentheses if available
  const strategyText = strategy ? ` (${strategy})` : '';
  html += `<div style="color: #60a5fa; margin-top: 4px;">Total: ${totalAllocated}/${totalTargets} targets${strategyText}</div>`;
  container.innerHTML = html;
}

// ----------------------------------------------------
// Import / Export
// ----------------------------------------------------

/**
/**
 * Check if JSON data is a segmented mission file (vs plain environment)
 * Detection: data.segments is array and data.segments[0].env exists
 */
function isSegmentedMissionJson(data, filename = "") {
  const hasSegmentsArray = data && Array.isArray(data.segments) && data.segments.length > 0 && data.segments[0].env;
  const isSegmented = isSegmentedByFilename(filename);
  console.log(`[isSegmentedMissionJson] hasSegmentsArray=${hasSegmentsArray}, isSegmentedByFilename=${isSegmented}, filename=${filename}`);
  return hasSegmentsArray && isSegmented;
}

/**
 * Extract segment count from filename pattern _Nx_ where x is the number
 * Returns the segment count (x) or 1 if not found (non-segmented)
 */
function getSegmentCountFromFilename(filename) {
  if (!filename) return 1;
  // Match pattern _Nx_ where x is one or more digits
  const match = filename.match(/_N(\d+)_/);
  if (match) {
    return parseInt(match[1], 10);
  }
  return 1; // Default to non-segmented
}

/**
 * Check if file should be treated as segmented based on filename pattern _Nx_ where x > 1
 */
function isSegmentedByFilename(filename) {
  return getSegmentCountFromFilename(filename) > 1;
}

/**
 * Check if JSON data has segmentInfo with cut points (replay format)
 * This format contains the base env + segmentInfo.segmentCuts for replay
 * Detects by: type="segmented" AND segmentCuts array with entries
 * OR filename has _Nx_ pattern with x > 1
 */
function isSegmentInfoJson(data, filename = "") {
  const hasSegmentInfoStructure = data && data.type === "segmented" && data.segmentInfo && Array.isArray(data.segmentInfo.segmentCuts);
  if (!hasSegmentInfoStructure) return false;

  // If segmentCuts has entries, it's definitely a segmented file
  const hasSegmentCuts = data.segmentInfo.segmentCuts.length > 0;
  const hasFilenamePattern = isSegmentedByFilename(filename);

  return hasSegmentCuts || hasFilenamePattern;
}

/**
 * Load a segmented mission from segmentInfo format (replay format)
 * Uses SegmentedImportManager as single source of truth
 */
function loadSegmentInfoFromJson(data, filename = "") {
  appendDebugLine(`📥 Loading segmentInfo format from ${filename}`);

  // 1. Load into the segmented import manager (single source of truth)
  segmentedImport.loadFromSegmentInfo(data);

  // 2. Set up global state from the manager
  const fullEnv = segmentedImport.getFullEnv();
  state.env = fullEnv;
  state.droneConfigs = JSON.parse(JSON.stringify(fullEnv.drone_configs || {}));
  state.env.drone_configs = state.droneConfigs;

  // Store initial snapshot (ALL targets - never overwrite this)
  state.initialEnvSnapshot = JSON.parse(JSON.stringify(fullEnv));

  // 3. Reset mission state
  missionReplay.clear();
    segmentStore.clear();    // Clear segmentStore for new import
  missionState.committedSegments = [];
  missionState.draftSolution = null;
  state.visitedTargets = [];  // Segment 0 starts with no visited targets
  state.importedSegmentCuts = null;  // We use segmentedImport now, not this
  state.wrappedPolygons = [];  // Clear ghost SAM boundaries from previous imports

  // 4. Create MissionReplay segments (placeholder solutions to be filled on Solve)
  const totalSegments = segmentedImport.getTotalSegments();
  appendDebugLine(`📊 Creating ${totalSegments} segments`);

  // Check if the imported JSON has solutions (for replay) or not (requires re-solve)
  const hasSolutions = segmentedImport.hasSolutions();
  if (hasSolutions) {
    appendDebugLine(`📦 Imported mission has solutions - replay ready`);
  } else {
    appendDebugLine(`📦 Imported mission has no solutions - requires solving each segment`);
  }

  for (let i = 0; i < totalSegments; i++) {
    const cutDistance = segmentedImport.getCutDistanceForSegment(i);
    const cutPositions = segmentedImport.getCutPositionForSegment(i);
    const storedSolution = segmentedImport.getSolutionForSegment(i);

    // Log segment info with more detail
    const visited = segmentedImport.getVisitedTargetsForSegment(i);
    const hasRoutes = storedSolution && Object.keys(storedSolution.routes || {}).length > 0;
    const cutDistStr = cutDistance !== null ? cutDistance.toFixed(1) : 'null';
    const cutPosStr = cutPositions ? Object.entries(cutPositions).map(([d,p]) => `D${d}:[${Array.isArray(p) ? p.map(v=>v.toFixed(1)).join(',') : 'obj'}]`).join(' ') : 'null';
    appendDebugLine(`   Segment ${i}: cutDist=${cutDistStr}, cutPos=${cutPosStr}, visited=[${visited.join(",")}], hasRoutes=${hasRoutes}`);

    missionReplay.addSegment({
      solution: storedSolution || { routes: {}, sequences: {} },
      env: fullEnv,  // All segments see all targets; filtering happens in solver
      cutDistance: i === 0 ? null : cutDistance,
      cutPositions: hasSolutions ? cutPositions : null,  // Only use cutPositions if we have solutions
      isCheckpointReplan: i > 0,
    });
  }

  // 5. Reset to start
  missionReplay.resetToStart();
  missionState.currentBuildSegmentIndex = 0;

  // 6. NO cut marker on import - markers appear after you Accept solutions
  state.pendingCutPositions = null;
  state.pendingLostDrones = [];

  // 7. CRITICAL: If we have solutions, populate missionState.fullSolutionsPerSegment
  // This is needed for Reset and animation to work correctly with FULL trajectories
  if (hasSolutions) {
    missionState.fullSolutionsPerSegment = {};
    for (let i = 0; i < totalSegments; i++) {
      const storedSolution = segmentedImport.getSolutionForSegment(i);
      if (storedSolution && Object.keys(storedSolution.routes || {}).length > 0) {
        missionState.fullSolutionsPerSegment[i] = JSON.parse(JSON.stringify(storedSolution));
        if (i === 0) {
          missionState.seg0FullSolution = JSON.parse(JSON.stringify(storedSolution));
          appendDebugLine(`📦 Saved seg-0 FULL solution for Reset (${Object.keys(storedSolution.routes).length} drones)`);
        }
      }
    }
  }

  if (hasSolutions) {
    // Has solutions - ready to animate
    appendDebugLine(`✅ Loaded ${segmentedImport.getTotalSegments()} segments with solutions. Ready to Reset & Animate.`);
    setMissionMode(MissionMode.IDLE, "segmentInfo loaded with solutions - ready to animate");
  } else {
    // No solutions - needs solving
    appendDebugLine(`✅ Loaded ${segmentedImport.getTotalSegments()} segments. Solve segment 0 first.`);
    setMissionMode(MissionMode.IDLE, "segmentInfo loaded - solve segment 0");
  }
}

/**
 * Load a segmented mission from JSON - strips solutions, keeps only env + cut distances
 * User must solve each segment sequentially before animating
 * NOW USES SegmentedImportManager for clean state management
 */
function loadSegmentedMissionFromJson(data, filename = "") {
  appendDebugLine(`📥 Loading segmented mission from ${filename}`);
  console.log(`[loadSegmentedMissionFromJson] Starting load for ${filename}`);

  // 1. Load into the segmented import manager (single source of truth)
  const loadResult = segmentedImport.loadFromSegmentsArray(data);
  console.log(`[loadSegmentedMissionFromJson] loadFromSegmentsArray returned ${loadResult}`);
  console.log(`[loadSegmentedMissionFromJson] segmentedImport.isActive()=${segmentedImport.isActive()}`);
  console.log(`[loadSegmentedMissionFromJson] segmentedImport.getTotalSegments()=${segmentedImport.getTotalSegments()}`);

  // 2. Set up global state - show only segment 0's targets on import
  const displayEnv = segmentedImport.getEnvForDisplay();  // Only segment 0's targets
  const fullEnv = segmentedImport.getFullEnv();           // All targets for snapshot
  state.env = displayEnv;

  // Derive segment 0's droneConfigs from C1 cutPositions (which drones actually flew segment 0)
  // The JSON's per-segment drone_configs may be buggy (often shows wrong enabled drones)
  const baseConfigs = JSON.parse(JSON.stringify(displayEnv.drone_configs || {}));
  const c1Positions = segmentedImport.getCutPositionForSegment(1);  // C1 = end of segment 0

  if (c1Positions && Object.keys(c1Positions).length > 0) {
    // Disable all, then enable only drones that have C1 positions
    Object.keys(baseConfigs).forEach(did => {
      if (baseConfigs[did]) baseConfigs[did].enabled = false;
    });
    Object.keys(c1Positions).forEach(did => {
      if (baseConfigs[did]) baseConfigs[did].enabled = true;
    });
  } else {
  }
  state.droneConfigs = baseConfigs;
  state.env.drone_configs = state.droneConfigs;

  // Store initial snapshot (ALL targets - never overwrite this)
  state.initialEnvSnapshot = JSON.parse(JSON.stringify(fullEnv));
  // 3. Reset mission state
  missionReplay.clear();
    segmentStore.clear();    // Clear segmentStore for new import
  missionState.committedSegments = [];
  missionState.draftSolution = null;
  state.visitedTargets = [];  // Segment 0 starts with no visited targets
  state.missionId = null;
  state.importedSegmentCuts = null;  // We use segmentedImport now, not this
  state.wrappedPolygons = [];  // Clear ghost SAM boundaries from previous imports
  missionState.segmentedMission = null;

  // 4. Create MissionReplay segments (placeholder solutions to be filled on Solve)
  const totalSegments = segmentedImport.getTotalSegments();
  appendDebugLine(`📊 Creating ${totalSegments} segments`);

  // First, collect all segment drone_configs so we can derive lostDrones
  const segmentDroneConfigs = [];
  for (let i = 0; i < totalSegments; i++) {
    const configs = segmentedImport.getDroneConfigsForSegment(i) || {};
    segmentDroneConfigs.push(configs);
  }

  for (let i = 0; i < totalSegments; i++) {
    const cutDistance = segmentedImport.getCutDistanceForSegment(i);
    // Don't load cutPositions on import - they're calculated fresh when you Accept
    // const cutPositions = segmentedImport.getCutPositionForSegment(i);

    // Derive lostDrones: drones enabled in segment i-1 but disabled in segment i
    let lostDrones = [];
    if (i > 0) {
      const prevConfigs = segmentDroneConfigs[i - 1];
      const currConfigs = segmentDroneConfigs[i];
      Object.entries(prevConfigs).forEach(([did, cfg]) => {
        if (cfg.enabled && !currConfigs[did]?.enabled) {
          lostDrones.push(did);
        }
      });
      if (lostDrones.length > 0) {
      }
    }

    // Log segment info
    const visited = segmentedImport.getVisitedTargetsForSegment(i);
    const cutDistStr = cutDistance !== null ? cutDistance.toFixed(1) : 'null';
    appendDebugLine(`   Segment ${i}: cutDist=${cutDistStr}, visited=[${visited.join(",")}], lostDrones=[${lostDrones.join(",")}]`);

    missionReplay.addSegment({
      solution: { routes: {}, sequences: {} },
      env: fullEnv,  // All segments see all targets; filtering happens in solver
      cutDistance: i === 0 ? null : cutDistance,
      cutPositions: null,  // Don't copy cutPositions on import - they're set when you Accept
      isCheckpointReplan: i > 0,
      lostDrones: lostDrones,  // Drones that were lost at this checkpoint
    });
  }

  // 5. Reset to start
  missionReplay.resetToStart();
  missionState.currentBuildSegmentIndex = 0;

  // 6. NO cut marker on import - it appears after Solve
  state.pendingCutPositions = null;
  state.pendingLostDrones = [];

  appendDebugLine(`✅ Loaded ${segmentedImport.getTotalSegments()} segments. Solve segment 0 first.`);

  // Set mode to IDLE - user must solve segment 0 first
  setMissionMode(MissionMode.IDLE, `segmented mission loaded - solve segment 1`);
}

/**
 * Load environment or segmented mission from JSON data
 * Centralizes all import logic for both plain env and segmented mission files
 */
function loadFromJson(data, filename = "") {
  // Full reset for any import
  state.envFilename = filename || "";
  state.missionId = null;
  state.routes = {};
  state.sequences = {};
  state.trajectoryVisible = {};
  state.visitedTargets = [];
  state.initialEnvSnapshot = null;
  state.missionHistory = [];
  state.targetSegmentMap = {};
  state.currentCutSegment = 1;
  state.previouslyAssignedTargets = [];
  state.importedSegmentCuts = [];  // Clear segmented import state
  state.wrappedPolygons = [];  // Clear any ghost SAM polygons from previous imports
  state.pendingCutPositions = null;  // Clear any ghost markers from previous imports
  state.pendingCheckpoints = null;  // Clear checkpoints from previous imports
  state.checkpointSegmentCounter = 0;  // Reset checkpoint counter on import
  state.pendingLostDrones = [];  // Clear lost drones from previous imports

  // Stop animation if running
  if (state.animation) {
    if (state.animation.animationId) {
      cancelAnimationFrame(state.animation.animationId);
      state.animation.animationId = null;
    }
    state.animation.active = false;
    state.animation.drones = {};
  }

  // Reset replay + draft + segmented mission state
  missionReplay.clear();
    segmentedImport.clear();  // CRITICAL: Clear segmented import manager to avoid ghost targets/SAMs
  segmentStore.clear();    // Clear segmentStore for new mission
  state.wrappedPolygons = [];  // Clear ghost SAM boundaries from previous imports
  missionState.draftSolution = null;
  missionState.checkpointSource = null;
  missionState.segmentedMission = null;
  missionState.currentBuildSegmentIndex = 0;

  // --- Case 1: SegmentInfo format (type: "segmented" with segmentInfo.segmentCuts) ---
  // Check this FIRST because some files may have both segments array AND segmentInfo
  // IMPORTANT: Now requires filename pattern _Nx_ where x > 1 to be treated as segmented
  const segCountFromFilename = getSegmentCountFromFilename(filename);
  if (isSegmentInfoJson(data, filename)) {
    loadSegmentInfoFromJson(data, filename);
    initDroneConfigsFromEnv();
    updateSamWrappingClientSide();
    drawEnvironment();
    const envNameEl = $("env-filename");
    if (envNameEl) envNameEl.textContent = filename || "(imported)";
    return;
  }

  // --- Case 1b: Segmented mission JSON (segments array format) ---
  if (isSegmentedMissionJson(data, filename)) {
    loadSegmentedMissionFromJson(data, filename);
    initDroneConfigsFromEnv();
    updateSamWrappingClientSide();
    drawEnvironment();
    const envNameEl = $("env-filename");
    if (envNameEl) envNameEl.textContent = filename || "(imported)";
    return;
  }

  // --- Case 2: New non-segmented format (schema: isr_env_v1, is_segmented: false) ---
  if (data && data.schema === "isr_env_v1" && data.is_segmented === false && data.env) {
    state.env = JSON.parse(JSON.stringify(data.env));
    setMissionMode(MissionMode.IDLE, "new environment loaded");
    appendDebugLine(`Imported environment from ${filename}`);

  } else if (data && data.environment) {
    // --- Case 3: Wrapped format (data.environment) ---
    state.env = data.environment;
    setMissionMode(MissionMode.IDLE, "new environment loaded");
    appendDebugLine(`Imported environment from ${filename}`);

  } else {
    // --- Case 4: Plain env JSON (legacy/backward compatible) ---
    state.env = data;
    setMissionMode(MissionMode.IDLE, "new environment loaded");
    appendDebugLine(`Imported environment from ${filename}`);
  }

  // Initialize drone configs (from env if present)
  initDroneConfigsFromEnv();
  Object.entries(state.droneConfigs || {}).forEach(([did, cfg]) => {
    if (cfg.enabled) {
      appendDebugLine(`   D${did}: ${cfg.start_airport}→${cfg.end_airport}, fuel=${cfg.fuel_budget}`);
    }
  });

  // Recompute SAM wrapping and redraw
  updateSamWrappingClientSide();
  drawEnvironment();

  // Update filename label
  const envNameEl = $("env-filename");
  if (envNameEl) envNameEl.textContent = filename || "(imported)";
}

function attachIOHandlers() {
  const fileInput = $("file-input");
  const btnImport = $("btn-import");
  const btnExport = $("btn-export");

  // If there's no file input, we can't import anything
  if (!fileInput) {
    appendDebugLine("attachIOHandlers: no file-input element found; import disabled.");
    return;
  }

  // Wire up the Import button if present
  if (btnImport) {
    btnImport.addEventListener("click", () => {
      fileInput.value = "";
      fileInput.click();
    });
  } else {
    appendDebugLine("attachIOHandlers: no btn-import element found; import button disabled.");
  }

  // File selection / JSON import
  fileInput.addEventListener("change", (evt) => {
    const file = evt.target.files && evt.target.files[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const text = e.target.result;
        const data = JSON.parse(text);
        loadFromJson(data, file.name);
      } catch (err) {
        appendDebugLine("Error parsing JSON: " + err);
        alert("Error parsing JSON: " + err);
      }
    };

    reader.readAsText(file);
  });

  // Wire up Export button if present
  if (btnExport) {
    btnExport.addEventListener("click", exportEnvironment);
  } else {
    appendDebugLine("attachIOHandlers: no btn-export element found; export disabled.");
  }

  // Agent tab Import/Export buttons (duplicate functionality for convenience)
  const agentImportBtn = $("agent-import-btn");
  const agentExportBtn = $("agent-export-btn");

  if (agentImportBtn) {
    agentImportBtn.addEventListener("click", () => {
      fileInput.value = "";
      fileInput.click();
    });
  }

  if (agentExportBtn) {
    agentExportBtn.addEventListener("click", exportEnvironment);
  }
}

// ----------------------------------------------------
// Optimizer helpers for frozen trajectory respect
// ----------------------------------------------------

/**
 * Get the frozen trajectory length for each drone.
 * Works for both segmented import and checkpoint workflow.
 * The frozen portion is everything up to the cut marker.
 * Returns: { droneId: frozenPointCount }
 */
function getFrozenTrajectoryCounts() {
  const counts = {};

  // Determine cut distance from either segmented import or checkpoint workflow
  let cutDistance = null;

  if (segmentedImport.isActive()) {
    const segIdx = segmentedImport.getCurrentSegmentIndex();
    if (segIdx === 0) {
      // First segment - nothing frozen yet
      return counts;
    }
    cutDistance = segmentedImport.getCutDistanceForSegment(segIdx);
  } else if (state.checkpoint?.active && state.checkpoint.segments) {
    // Checkpoint workflow - get cut distance from checkpoint data
    // Use the first drone's checkpoint distance as the cut distance
    const firstDroneData = Object.values(state.checkpoint.segments)[0];
    cutDistance = firstDroneData?.checkpointDist || state.pendingCutDistance;
  } else if (missionState.draftSolution?.isCheckpointReplan) {
    // Draft solution has checkpoint data (after solve but before accept)
    // Try direct cutDistance first, then fall back to checkpointSegments
    cutDistance = missionState.draftSolution.cutDistance;
    if (!cutDistance && missionState.draftSolution.checkpointSegments) {
      const firstDroneData = Object.values(missionState.draftSolution.checkpointSegments)[0];
      cutDistance = firstDroneData?.checkpointDist;
    }
  } else if (state.pendingCutDistance) {
    // Fallback to pending cut distance
    cutDistance = state.pendingCutDistance;
  }

  if (!cutDistance || cutDistance <= 0) {
    return counts;
  }

  // Calculate frozen point count for each drone based on cut distance
  Object.entries(state.routes || {}).forEach(([did, routeData]) => {
    const traj = routeData.trajectory || [];
    if (traj.length < 2) {
      counts[did] = 0;
      return;
    }

    // Find how many points are within the cut distance
    let cumulativeDist = 0;
    let frozenCount = 1; // At least the first point
    for (let i = 1; i < traj.length; i++) {
      const dx = traj[i][0] - traj[i-1][0];
      const dy = traj[i][1] - traj[i-1][1];
      cumulativeDist += Math.sqrt(dx * dx + dy * dy);
      if (cumulativeDist <= cutDistance) {
        frozenCount = i + 1;
      } else {
        break;
      }
    }
    counts[did] = frozenCount;
  });

  return counts;
}

/**
 * Calculate which targets are visited within a given distance on a set of routes.
 * This uses the ACTUAL trajectory and cut distance, NOT the JSON's visitedTargets.
 * @param {Object} routes - { droneId: { trajectory: [...], route: [...] } }
 * @param {number} cutDistance - Distance threshold for considering targets visited
 * @returns {Array} Array of target IDs that are visited within the cut distance
 */
function calculateVisitedTargetsFromRoutes(routes, cutDistance) {
  console.log(`[calculateVisitedTargets] cutDistance=${cutDistance}, routes=${Object.keys(routes || {}).length}`);
  if (!cutDistance || cutDistance <= 0) {
    console.log(`[calculateVisitedTargets] Early return: cutDistance is ${cutDistance}`);
    return [];
  }

  const visitedTargets = new Set();

  // Get all target positions - try segmentedImport first, fall back to state.env
  let allTargets = segmentedImport.getAllTargets();
  if (!allTargets || allTargets.length === 0) {
    allTargets = state.env?.targets || [];
    console.log(`[calculateVisitedTargets] Using state.env.targets (${allTargets.length} targets)`);
  } else {
    console.log(`[calculateVisitedTargets] Using segmentedImport.getAllTargets() (${allTargets.length} targets)`);
  }
  const targetPositions = {};
  allTargets.forEach(t => {
    targetPositions[t.id] = [t.x, t.y];
  });

  // For each drone, find which targets are visited within the cut distance
  // IMPORTANT: Check ALL targets against the trajectory, not just the sequence
  // This is because frozenRoutes may have cumulative trajectory but only last segment's sequence
  Object.entries(routes || {}).forEach(([droneId, routeData]) => {
    const traj = routeData.trajectory || [];

    if (traj.length < 2) return;

    // Calculate cumulative distance at each trajectory point
    const distances = [0];
    for (let i = 1; i < traj.length; i++) {
      const dx = traj[i][0] - traj[i-1][0];
      const dy = traj[i][1] - traj[i-1][1];
      distances.push(distances[i-1] + Math.sqrt(dx * dx + dy * dy));
    }

    // For each target, check if it's on the trajectory within the cut distance
    // We check ALL targets (not just sequence) because frozenRoutes may have
    // cumulative trajectory from multiple segments but only last segment's route
    allTargets.forEach(target => {
      const targetId = target.id;
      const targetPos = targetPositions[targetId];
      if (!targetPos) return;

      // Find if this target is on the trajectory within the cut distance
      for (let i = 0; i < traj.length; i++) {
        const dx = traj[i][0] - targetPos[0];
        const dy = traj[i][1] - targetPos[1];
        const dist = Math.sqrt(dx * dx + dy * dy);

        // If trajectory point is close to target (within tolerance)
        if (dist < 1.0) {  // 1.0 unit tolerance
          // Check if this trajectory point is within the cut distance
          if (distances[i] <= cutDistance) {
            visitedTargets.add(targetId);
            console.log(`[calculateVisitedTargets] D${droneId}: Found ${targetId} at traj[${i}], dist=${distances[i].toFixed(1)} <= cutDist=${cutDistance.toFixed(1)}`);
          } else {
            console.log(`[calculateVisitedTargets] D${droneId}: Found ${targetId} at traj[${i}], dist=${distances[i].toFixed(1)} > cutDist=${cutDistance.toFixed(1)} (NOT visited)`);
          }
          break;
        }
      }
    });
  });

  console.log(`[calculateVisitedTargets] Result: [${Array.from(visitedTargets).join(",")}]`);
  return Array.from(visitedTargets);
}

/**
 * Calculate which targets are in the frozen portion of the trajectory.
 * This is based on the actual trajectory and cut distance, NOT the JSON's visitedTargets.
 * Returns: array of target IDs that are within the frozen trajectory portion.
 */
function getActualFrozenTargets() {
  if (!segmentedImport.isActive()) {
    return [];
  }

  const segIdx = segmentedImport.getCurrentSegmentIndex();
  if (segIdx === 0) {
    return [];
  }

  const cutDistance = segmentedImport.getCutDistanceForSegment(segIdx);
  if (!cutDistance || cutDistance <= 0) return [];

  const frozenTargets = new Set();

  // Get all target positions - try segmentedImport first, fall back to state.env
  let allTargets = segmentedImport.getAllTargets();
  if (!allTargets || allTargets.length === 0) {
    allTargets = state.env?.targets || [];
  }
  const targetPositions = {};
  allTargets.forEach(t => {
    targetPositions[t.id] = [t.x, t.y];
  });

  // For each drone, find which targets are visited within the frozen trajectory portion
  // IMPORTANT: Check ALL targets against the trajectory, not just the sequence
  // This is because state.routes may have cumulative trajectory but only last segment's sequence
  Object.entries(state.routes || {}).forEach(([droneId, routeData]) => {
    const traj = routeData.trajectory || [];

    if (traj.length < 2) return;

    // Calculate cumulative distance at each trajectory point
    const distances = [0];
    for (let i = 1; i < traj.length; i++) {
      const dx = traj[i][0] - traj[i-1][0];
      const dy = traj[i][1] - traj[i-1][1];
      distances.push(distances[i-1] + Math.sqrt(dx * dx + dy * dy));
    }

    // For each target, check if it's on the trajectory within the frozen distance
    // We check ALL targets (not just sequence) because state.routes may have
    // cumulative trajectory from multiple segments but only last segment's route
    allTargets.forEach(target => {
      const targetId = target.id;
      const targetPos = targetPositions[targetId];
      if (!targetPos) return;

      // Find if this target is on the trajectory within the cut distance
      for (let i = 0; i < traj.length; i++) {
        const dx = traj[i][0] - targetPos[0];
        const dy = traj[i][1] - targetPos[1];
        const dist = Math.sqrt(dx * dx + dy * dy);

        // If trajectory point is close to target (within tolerance)
        if (dist < 1.0) {  // 1.0 unit tolerance
          // Check if this trajectory point is within the frozen distance
          if (distances[i] <= cutDistance) {
            frozenTargets.add(targetId);
          }
          break;
        }
      }
    });
  });

  return Array.from(frozenTargets);
}

/**
 * Get routes with only the unfrozen portion (for optimizers).
 * This extracts only the current segment's trajectory, excluding frozen prefix.
 */
function getUnfrozenRoutes() {
  const frozenCounts = getFrozenTrajectoryCounts();
  const unfrozenRoutes = {};

  Object.entries(state.routes || {}).forEach(([did, routeData]) => {
    const frozenCount = frozenCounts[did] || 0;
    const fullTraj = routeData.trajectory || [];
    const fullRoute = routeData.route || [];

    if (frozenCount === 0) {
      // No frozen portion - use full route
      unfrozenRoutes[did] = JSON.parse(JSON.stringify(routeData));
    } else {
      // Extract only the unfrozen portion
      // The trajectory after the frozen prefix
      const unfrozenTraj = fullTraj.slice(frozenCount - 1); // Include the cut point as new start

      // For route sequence, we need to find which targets are after the cut
      // This is complex - route contains node IDs, not trajectory points
      // For now, we'll use the full route and let the optimizer handle it
      // But mark the frozen targets so they're excluded
      unfrozenRoutes[did] = {
        ...routeData,
        trajectory: unfrozenTraj,
        route: fullRoute, // Keep full route for now, optimizer will filter by visitedTargets
        _frozenPointCount: frozenCount,
        _originalTrajectory: fullTraj,
      };
    }
  });

  return unfrozenRoutes;
}

/**
 * Get environment with only unfrozen targets (for optimizers).
 * Uses ACTUAL frozen targets based on cut distance, not JSON's visitedTargets.
 *
 * The key insight: state.visitedTargets comes from the JSON and may include targets
 * that are AFTER the cut marker in the current trajectory. We need to calculate
 * which targets are actually in the frozen portion based on the trajectory distance.
 */
function getUnfrozenEnv() {
  const env = JSON.parse(JSON.stringify(state.env || {}));

  // CRITICAL: Merge state.droneConfigs to preserve UI-configured values (end_airport, fuel_budget, enabled)
  // state.env.drone_configs comes from the imported JSON, but user may have changed these in the UI
  const droneConfigs = JSON.parse(JSON.stringify(env.drone_configs || {}));
  Object.entries(state.droneConfigs || {}).forEach(([did, cfg]) => {
    if (!droneConfigs[did]) {
      // Drone not in original config - add it
      droneConfigs[did] = JSON.parse(JSON.stringify(cfg));
    } else {
      // Merge UI config over imported config
      if (cfg.end_airport) droneConfigs[did].end_airport = cfg.end_airport;
      if (cfg.start_airport) droneConfigs[did].start_airport = cfg.start_airport;
      if (cfg.fuel_budget !== undefined) droneConfigs[did].fuel_budget = cfg.fuel_budget;
      if (cfg.enabled !== undefined) droneConfigs[did].enabled = cfg.enabled;
      if (cfg.target_access) droneConfigs[did].target_access = cfg.target_access;
    }
  });
  env.drone_configs = droneConfigs;

  // Determine frozen targets:
  // - For segmented import: use actual trajectory-based calculation
  // - For checkpoint workflow: use state.visitedTargets directly
  let frozenTargets;
  if (segmentedImport.isActive() && segmentedImport.getCurrentSegmentIndex() > 0) {
    frozenTargets = getActualFrozenTargets();
  } else if (state.visitedTargets && state.visitedTargets.length > 0) {
    frozenTargets = state.visitedTargets;
  } else {
    frozenTargets = [];
  }
  const frozenSet = new Set(frozenTargets);

  // Filter out frozen targets
  env.targets = (env.targets || []).filter(t => !frozenSet.has(t.id));

  return env;
}

/**
 * Merge optimized routes back with frozen prefix.
 * Takes the optimizer output and prepends the frozen trajectory.
 * IMPORTANT: Also preserves any existing routes that the optimizer didn't process.
 */
function mergeWithFrozenRoutes(optimizedRoutes) {
  const frozenCounts = getFrozenTrajectoryCounts();
  const mergedRoutes = {};

  // First, preserve ALL existing routes from state.routes
  // This ensures we don't lose routes for drones that the optimizer didn't touch
  Object.entries(state.routes || {}).forEach(([did, routeData]) => {
    mergedRoutes[did] = JSON.parse(JSON.stringify(routeData));
  });

  // Then process routes from the optimizer, merging with frozen portions
  Object.entries(optimizedRoutes || {}).forEach(([did, optimizedData]) => {
    const frozenCount = frozenCounts[did] || 0;
    const originalRoute = state.routes[did];

    if (frozenCount === 0 || !originalRoute) {
      // No frozen portion - use optimized route directly
      mergedRoutes[did] = JSON.parse(JSON.stringify(optimizedData));
    } else {
      // Prepend frozen trajectory
      const frozenTraj = originalRoute.trajectory.slice(0, frozenCount);
      const optimizedTraj = optimizedData.trajectory || [];

      // Check for duplicate at junction
      const lastFrozen = frozenTraj[frozenTraj.length - 1];
      const firstOptimized = optimizedTraj[0];
      const isDuplicate = lastFrozen && firstOptimized &&
        Math.abs(lastFrozen[0] - firstOptimized[0]) < 0.001 &&
        Math.abs(lastFrozen[1] - firstOptimized[1]) < 0.001;

      const mergedTraj = isDuplicate
        ? frozenTraj.concat(optimizedTraj.slice(1))
        : frozenTraj.concat(optimizedTraj);

      mergedRoutes[did] = {
        ...optimizedData,
        trajectory: mergedTraj,
        distance: (originalRoute.distance || 0), // Keep original distance calc
      };

    }
  });

  return mergedRoutes;
}

// ----------------------------------------------------
// Optimization buttons
// ----------------------------------------------------
function attachOptimizationHandlers() {
  const btnInsert = $("btn-optimize-insert");
  const btnCrossRemove = $("btn-optimize-cross");

  if (btnInsert) {
    btnInsert.addEventListener("click", async () => {
      if (!state.routes || Object.keys(state.routes).length === 0) {
        appendDebugLine("No routes to optimize. Run planner first.");
        return;
      }
      appendDebugLine("Running Insert Missed optimization...");
      btnInsert.disabled = true;
      btnInsert.textContent = "Working...";

      try {
        const droneConfigs = state.droneConfigs;
        // FROZEN TRAJECTORY RESPECT: Use only unfrozen routes and targets
        const unfrozenEnv = getUnfrozenEnv();
        // Check for frozen targets from EITHER segmented import OR checkpoint replan
        const segmentedFrozen = segmentedImport.isActive() && segmentedImport.getCurrentSegmentIndex() > 0;
        const checkpointFrozen = (state.visitedTargets && state.visitedTargets.length > 0);
        const hasFrozen = segmentedFrozen || checkpointFrozen;
        // For segmented import, use trajectory-based frozen targets; for checkpoint, use state.visitedTargets
        const actualFrozenTargets = segmentedFrozen ? getActualFrozenTargets() : (state.visitedTargets || []);
        const resp = await fetch("/api/insert_missed_optimize", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            solution: {
              routes: state.routes,
              distance_matrix: state.distanceMatrix
            },
            env: unfrozenEnv, // Only unfrozen targets
            drone_configs: droneConfigs,
            visited_targets: actualFrozenTargets // Tell backend which targets are frozen
          })
        });
        const data = await resp.json();

        if (data.success) {
          const insertions = data.insertions || [];
          if (insertions.length > 0) {
            // FROZEN TRAJECTORY RESPECT: Merge optimized routes with frozen prefix
            if (hasFrozen) {
              state.routes = mergeWithFrozenRoutes(data.routes);
            } else {
              // Deep clone to avoid read-only property errors from frozen fetch response
              state.routes = JSON.parse(JSON.stringify(data.routes));
            }
            // Deep clone sequences to avoid read-only property errors
            state.sequences = JSON.parse(JSON.stringify(data.sequences || {}));

            // Update sequence display
            for (const [did, seq] of Object.entries(state.sequences)) {
              state.sequences[did] = seq;
            }

            appendDebugLine(`Insert optimization: ${insertions.length} targets inserted.`);
            insertions.forEach(ins => {
              appendDebugLine(`  ${ins.target} -> D${ins.drone}`);
            });

            // Recalculate trajectories for modified routes
            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();

            // Update the committed segment so Reset preserves optimizer changes
            missionReplay.updateCurrentSegmentSolution(state.routes, state.sequences);
          } else {
            appendDebugLine("Insert optimization: No insertions possible (all visited or fuel exhausted).");
          }
        } else {
          appendDebugLine("Insert optimization error: " + (data.error || "Unknown error"));
        }
      } catch (err) {
        appendDebugLine("Insert optimization error: " + err.message);
      } finally {
        btnInsert.disabled = false;
        btnInsert.textContent = "Insert";
      }
    });
  }

  // "Swap" button - runs cascade swap optimization until no more beneficial swaps
  const btnSwapUntil = $("btn-optimize-swap-until");
  if (btnSwapUntil) {
    btnSwapUntil.addEventListener("click", async () => {
      if (!state.routes || Object.keys(state.routes).length === 0) {
        appendDebugLine("No routes to optimize. Run planner first.");
        return;
      }

      btnSwapUntil.disabled = true;
      const originalText = btnSwapUntil.textContent;
      btnSwapUntil.textContent = "Running...";

      try {
        const droneConfigs = state.droneConfigs;
        const unfrozenEnv = getUnfrozenEnv();
        const segmentedFrozen = segmentedImport.isActive() && segmentedImport.getCurrentSegmentIndex() > 0;
        const checkpointFrozen = (state.visitedTargets && state.visitedTargets.length > 0);
        const hasFrozen = segmentedFrozen || checkpointFrozen;
        const actualFrozenTargets = segmentedFrozen ? getActualFrozenTargets() : (state.visitedTargets || []);

        const autoRegen = $("chk-auto-regen") ? $("chk-auto-regen").checked : false;

        // If user requested auto-regeneration cascade, do a single server-side call
        // which will auto-iterate/regenerate and return the full sequence. Otherwise
        // run the client-side loop that applies a single-pass repeatedly.
        if (autoRegen) {
          appendDebugLine("Swap optimization: requesting server-side cascade (auto_regen)");
          const resp = await fetch("/api/trajectory_swap_optimize", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              solution: { routes: state.routes, distance_matrix: state.distanceMatrix },
              env: unfrozenEnv,
              drone_configs: droneConfigs,
              visited_targets: actualFrozenTargets,
              auto_iterate: true,
              auto_regen: true
            })
          });
          const data = await resp.json();
          if (!data || !data.success) {
            appendDebugLine("Swap optimization error during server-side cascade: " + (data && data.error ? data.error : "Unknown error"));
          } else {
            const swaps = data.swaps_made || [];
            appendDebugLine(`Server-side cascade applied ${swaps.length} swaps, iterations=${data.iterations || 1}`);

            // Merge returned routes (preserve frozen prefixes)
            let savedFrozenTrajectories = null;
            if (hasFrozen) {
              const frozenCounts = getFrozenTrajectoryCounts();
              savedFrozenTrajectories = {};
              Object.entries(state.routes || {}).forEach(([did, routeData]) => {
                const frozenCount = frozenCounts[did] || 0;
                if (frozenCount > 0 && routeData.trajectory && routeData.trajectory.length >= frozenCount) {
                  savedFrozenTrajectories[did] = routeData.trajectory.slice(0, frozenCount);
                }
              });
            }

            state.routes = JSON.parse(JSON.stringify(data.routes));
            if (savedFrozenTrajectories) {
              Object.entries(savedFrozenTrajectories).forEach(([did, frozenTraj]) => {
                if (state.routes[did]) state.routes[did].trajectory = frozenTraj;
              });
            }
            state.sequences = JSON.parse(JSON.stringify(data.sequences || {}));

            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();
            missionReplay.updateCurrentSegmentSolution(state.routes, state.sequences);
          }
        } else {
          let totalSwaps = 0;
          const maxLoops = 200; // safety cap to avoid accidental infinite loops
          let loop = 0;

          while (loop < maxLoops) {
            loop += 1;
            appendDebugLine(`Swap loop iteration ${loop}...`);

            const resp = await fetch("/api/trajectory_swap_optimize", {
              method: "POST",
              headers: { "Content-Type": "application/json" },
              body: JSON.stringify({
                solution: { routes: state.routes, distance_matrix: state.distanceMatrix },
                env: unfrozenEnv,
                drone_configs: droneConfigs,
                visited_targets: actualFrozenTargets
              })
            });
            const data = await resp.json();

            if (!data || !data.success) {
              appendDebugLine("Swap optimization error during loop: " + (data && data.error ? data.error : "Unknown error"));
              break;
            }

            const swaps = data.swaps_made || [];
            if (swaps.length === 0) {
              appendDebugLine(`No more swaps found after ${loop - 1} iterations. Stopping.`);
              break;
            }

            // Apply the returned routes and sequences (preserves frozen handling similar to single-click)
            let savedFrozenTrajectories = null;
            if (hasFrozen) {
              const frozenCounts = getFrozenTrajectoryCounts();
              savedFrozenTrajectories = {};
              Object.entries(state.routes || {}).forEach(([did, routeData]) => {
                const frozenCount = frozenCounts[did] || 0;
                if (frozenCount > 0 && routeData.trajectory && routeData.trajectory.length >= frozenCount) {
                  savedFrozenTrajectories[did] = routeData.trajectory.slice(0, frozenCount);
                }
              });
            }

            // Use returned routes (deep clone)
            state.routes = JSON.parse(JSON.stringify(data.routes));
            if (savedFrozenTrajectories) {
              Object.entries(savedFrozenTrajectories).forEach(([did, frozenTraj]) => {
                if (state.routes[did]) state.routes[did].trajectory = frozenTraj;
              });
            }
            state.sequences = JSON.parse(JSON.stringify(data.sequences || {}));

            // Show the swaps from this iteration
            appendDebugLine(`  Iter ${loop}: ${swaps.length} swap(s)`);
            swaps.slice(0, 20).forEach(swap => {
              appendDebugLine(`    ${swap.target}: D${swap.from_drone} -> D${swap.to_drone} (savings=${(swap.savings||swap.savings||0).toFixed? (swap.savings||0).toFixed(1) : swap.savings})`);
            });
            if (swaps.length > 20) appendDebugLine(`    ... and ${swaps.length - 20} more`);

            totalSwaps += swaps.length;

            // Regenerate trajectories and update UI after each iteration so user can observe progress
            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();

            // Also update mission replay committed solution so Reset preserves progress
            missionReplay.updateCurrentSegmentSolution(state.routes, state.sequences);

            // small yield to keep UI responsive
            await new Promise(r => setTimeout(r, 50));
          }

          appendDebugLine(`Swap finished: total swaps applied = ${totalSwaps}, iterations = ${loop}`);
        }
      } catch (err) {
        appendDebugLine("Swap error: " + err.message);
      } finally {
        btnSwapUntil.disabled = false;
        btnSwapUntil.textContent = originalText;
      }
    });
  }

  if (btnCrossRemove) {
    btnCrossRemove.addEventListener("click", async () => {
      if (!state.routes || Object.keys(state.routes).length === 0) {
        appendDebugLine("No routes to optimize. Run planner first.");
        return;
      }
      appendDebugLine("Running Crossing Removal optimization...");
      btnCrossRemove.disabled = true;
      btnCrossRemove.textContent = "Working...";

      try {
        const droneConfigs = state.droneConfigs;
        // FROZEN TRAJECTORY RESPECT: Use only unfrozen targets
        const unfrozenEnv = getUnfrozenEnv();
        // Check for frozen targets from EITHER segmented import OR checkpoint replan
        const segmentedFrozen = segmentedImport.isActive() && segmentedImport.getCurrentSegmentIndex() > 0;
        const checkpointFrozen = (state.visitedTargets && state.visitedTargets.length > 0);
        const hasFrozen = segmentedFrozen || checkpointFrozen;
        // For segmented import, use trajectory-based frozen targets; for checkpoint, use state.visitedTargets
        const actualFrozenTargets = segmentedFrozen ? getActualFrozenTargets() : (state.visitedTargets || []);
        const resp = await fetch("/api/crossing_removal_optimize", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            solution: { routes: state.routes },
            env: unfrozenEnv, // Only unfrozen targets
            drone_configs: droneConfigs,
            visited_targets: actualFrozenTargets // Tell backend which targets are frozen
          })
        });
        const data = await resp.json();

        if (data.success) {
          const fixes = data.fixes_made || [];
          if (fixes.length > 0) {
            // FROZEN TRAJECTORY RESPECT: Merge optimized routes with frozen prefix
            if (hasFrozen) {
              state.routes = mergeWithFrozenRoutes(data.routes);
            } else {
              // Deep clone to avoid read-only property errors from frozen fetch response
              state.routes = JSON.parse(JSON.stringify(data.routes));
            }
            // Deep clone sequences to avoid read-only property errors
            state.sequences = JSON.parse(JSON.stringify(data.sequences || {}));

            // Update sequence display
            for (const [did, seq] of Object.entries(state.sequences)) {
              state.sequences[did] = seq;
            }

            appendDebugLine(`Crossing removal: ${fixes.length} crossings fixed.`);
            fixes.forEach(fix => {
              appendDebugLine(`  Drone ${fix.drone}: reversed segment ${fix.segment_i}-${fix.segment_j}`);
            });

            // Recalculate trajectories for modified routes
            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();

            // Update the committed segment so Reset preserves optimizer changes
            missionReplay.updateCurrentSegmentSolution(state.routes, state.sequences);
          } else {
            appendDebugLine("Crossing removal: No crossings found.");
          }
        } else {
          appendDebugLine("Crossing removal error: " + (data.error || "Unknown error"));
        }
      } catch (err) {
        appendDebugLine("Crossing removal error: " + err.message);
      } finally {
        btnCrossRemove.disabled = false;
        btnCrossRemove.textContent = "No Cross";
      }
    });
  }
}

async function regenerateTrajectories() {
  // Regenerate SAM-avoiding trajectories for all routes after swap optimization
  // FROZEN TRAJECTORY RESPECT: Only regenerate unfrozen portions
  const droneConfigs = state.droneConfigs;
  const frozenCounts = getFrozenTrajectoryCounts();
  // Check for frozen from EITHER segmented import OR checkpoint workflow
  const segmentedFrozen = segmentedImport.isActive() && segmentedImport.getCurrentSegmentIndex() > 0;
  const checkpointFrozen = state.checkpoint?.active || (state.visitedTargets && state.visitedTargets.length > 0);
  const hasFrozen = segmentedFrozen || checkpointFrozen;

  // Build env with merged drone configs from UI
  const envWithConfigs = JSON.parse(JSON.stringify(state.env || {}));
  const mergedDroneConfigs = JSON.parse(JSON.stringify(envWithConfigs.drone_configs || {}));
  Object.entries(state.droneConfigs || {}).forEach(([did, cfg]) => {
    if (!mergedDroneConfigs[did]) {
      mergedDroneConfigs[did] = JSON.parse(JSON.stringify(cfg));
    } else {
      if (cfg.end_airport) mergedDroneConfigs[did].end_airport = cfg.end_airport;
      if (cfg.start_airport) mergedDroneConfigs[did].start_airport = cfg.start_airport;
      if (cfg.fuel_budget !== undefined) mergedDroneConfigs[did].fuel_budget = cfg.fuel_budget;
      if (cfg.enabled !== undefined) mergedDroneConfigs[did].enabled = cfg.enabled;
    }
  });
  envWithConfigs.drone_configs = mergedDroneConfigs;

  // For checkpoint workflow: add synthetic starts from frozen trajectory endpoints
  // or from draftSolution.checkpointSegments
  // IMPORTANT: Add to AIRPORTS array (not just synthetic_starts) so /api/apply_sequence can find them
  if (hasFrozen) {
    envWithConfigs.synthetic_starts = envWithConfigs.synthetic_starts || {};
    envWithConfigs.airports = envWithConfigs.airports || [];

    // Try to get synthetic starts from draftSolution first
    if (missionState.draftSolution?.checkpointSegments) {
      Object.entries(missionState.draftSolution.checkpointSegments).forEach(([did, seg]) => {
        if (seg.splitPoint) {
          const nodeId = `D${did}_START`;
          envWithConfigs.synthetic_starts[nodeId] = { id: nodeId, x: seg.splitPoint[0], y: seg.splitPoint[1] };
          // Also add as airport so /api/apply_sequence can find it
          // IMPORTANT: Mark as is_synthetic: true so backend filters it from valid_end_airports
          if (!envWithConfigs.airports.some(a => a.id === nodeId)) {
            envWithConfigs.airports.push({ id: nodeId, x: seg.splitPoint[0], y: seg.splitPoint[1], is_synthetic: true });
          }
        }
      });
    }

    // Fallback: derive from frozen trajectory endpoints
    Object.entries(state.routes || {}).forEach(([did, routeData]) => {
      const frozenCount = frozenCounts[did] || 0;
      const nodeId = `D${did}_START`;
      if (frozenCount > 0 && routeData.trajectory && routeData.trajectory.length >= frozenCount && !envWithConfigs.synthetic_starts[nodeId]) {
        const lastFrozenPt = routeData.trajectory[frozenCount - 1];
        envWithConfigs.synthetic_starts[nodeId] = { id: nodeId, x: lastFrozenPt[0], y: lastFrozenPt[1] };
        // Also add as airport so /api/apply_sequence can find it
        // IMPORTANT: Mark as is_synthetic: true so backend filters it from valid_end_airports
        if (!envWithConfigs.airports.some(a => a.id === nodeId)) {
          envWithConfigs.airports.push({ id: nodeId, x: lastFrozenPt[0], y: lastFrozenPt[1], is_synthetic: true });
        }
      }
    });
  }

  for (const [did, routeData] of Object.entries(state.routes)) {
    if (!routeData.route || routeData.route.length < 2) {
      continue;
    }

    try {
      // If we have frozen trajectory, we need to preserve it
      const frozenCount = frozenCounts[did] || 0;
      const frozenTraj = hasFrozen && frozenCount > 0
        ? routeData.trajectory?.slice(0, frozenCount)
        : null;

      const resp = await fetch("/api/apply_sequence", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          drone_id: did,
          sequence: routeData.route.join(","),
          env: envWithConfigs,
          fuel_budget: droneConfigs[did]?.fuel_budget || 300
        })
      });
      const data = await resp.json();
      if (data.success) {
        // Ensure state.routes[did] is a fresh writable object to avoid read-only errors
        if (!state.routes[did] || Object.isFrozen(state.routes[did])) {
          state.routes[did] = JSON.parse(JSON.stringify(state.routes[did] || {}));
        }

        if (frozenTraj && frozenTraj.length > 0) {
          // FROZEN TRAJECTORY RESPECT: Merge frozen trajectory with regenerated
          // The new trajectory starts from the synthetic start (cut position),
          // so we simply append it to the frozen trajectory
          const newTraj = data.trajectory || [];

          // Check for duplicate at junction - new trajectory starts from synthetic start
          // which should be at or near the last frozen point
          const lastFrozen = frozenTraj[frozenTraj.length - 1];
          const firstNew = newTraj[0];
          const isDuplicate = lastFrozen && firstNew &&
            Math.abs(lastFrozen[0] - firstNew[0]) < 1.0 &&
            Math.abs(lastFrozen[1] - firstNew[1]) < 1.0;

          state.routes[did].trajectory = isDuplicate
            ? frozenTraj.concat(newTraj.slice(1))
            : frozenTraj.concat(newTraj);

        } else {
          // No frozen trajectory - use regenerated directly
          state.routes[did].trajectory = data.trajectory;
        }
        state.routes[did].distance = data.distance;
        state.routes[did].points = data.points;
      }
    } catch (err) {
    }
  }
}

// ----------------------------------------------------
// Planner integration
// ----------------------------------------------------
async function fetchWrappedPolygons(recalculate = false) {
  // Abort any pending wrapping request
  if (state.wrappingAbortController) {
    state.wrappingAbortController.abort();
  }

  // Create new abort controller for this request
  state.wrappingAbortController = new AbortController();
  const signal = state.wrappingAbortController.signal;

  try {
    let resp, data;

    if (recalculate && state.env && state.env.sams && state.env.sams.length > 0) {
      // Recalculate wrapping with current SAM positions
      resp = await fetch("/api/calculate_wrapping", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ sams: state.env.sams }),
        signal: signal
      });
      data = await resp.json();
    } else {
      // Get cached wrapping from last solve
      resp = await fetch("/api/wrapped_polygons", { signal: signal });
      data = await resp.json();
    }

    if (data && data.success && data.polygons) {
      state.wrappedPolygons = data.polygons;
    } else {
      state.wrappedPolygons = [];
    }
  } catch (err) {
    if (err.name === 'AbortError') {
      return; // Don't clear polygons on abort
    }
    state.wrappedPolygons = [];
  }
}

// ----------------------------------------------------
// Mission History & Segmented Replanning Helpers
// ----------------------------------------------------

/**
 * Save the initial environment snapshot (called on first load or reset)
 */
function saveInitialEnvSnapshot() {
  if (state.env) {
    const targetIds = (state.env.targets || []).map(t => t.id);
    state.initialEnvSnapshot = JSON.parse(JSON.stringify(state.env));
    state.missionHistory = [];
    state.visitedTargets = [];
    state.currentCutSegment = 1;  // Reset segment counter
    state.previouslyAssignedTargets = [];  // Clear previously assigned targets
    state.targetSegmentMap = {};  // Clear segment assignments
    appendDebugLine("Initial environment snapshot saved");
  }
}

/**
 * Push current segment to mission history
 */
function pushMissionSegment(note = "") {
  const segment = {
    envSnapshot: JSON.parse(JSON.stringify(state.env)),
    droneConfigsSnapshot: JSON.parse(JSON.stringify(state.droneConfigs)),
    startStates: {},  // { did: { x, y } } - where each drone started this segment
    trajectoriesByDrone: {},  // { did: [[x,y], ...] }
    routesByDrone: {},  // { did: ["A1", "T1", ...] }
    visitedTargetsThisSegment: [],
    meta: {
      createdAt: new Date().toISOString(),
      note: note,
      segmentIndex: state.missionHistory.length,
    }
  };

  // Capture start states and trajectories from current routes
  Object.entries(state.routes || {}).forEach(([did, routeData]) => {
    const traj = routeData.trajectory || [];
    const route = routeData.route || [];

    // Start state is first point of trajectory (or start airport position)
    if (traj.length > 0) {
      segment.startStates[did] = { x: traj[0][0], y: traj[0][1] };
      segment.trajectoriesByDrone[did] = JSON.parse(JSON.stringify(traj));
    }
    segment.routesByDrone[did] = [...route];

    // Extract visited targets (anything starting with T)
    route.forEach(wp => {
      if (String(wp).startsWith("T") && !state.visitedTargets.includes(wp)) {
        segment.visitedTargetsThisSegment.push(wp);
        state.visitedTargets.push(wp);
      }
    });
  });

  state.missionHistory.push(segment);
  appendDebugLine(`Segment ${segment.meta.segmentIndex} saved (${segment.visitedTargetsThisSegment.length} targets visited)`);
  return segment;
}

/**
 * Get drone configs from the last committed segment.
 * The segment is the source of truth for drone configuration.
 * Returns null only for the very first solve (segment 0) when no segments exist yet.
 *
 * @returns {object|null} The segment's drone_configs, or null for first solve
 */
function getPreviousSegmentDroneConfigs() {
  // Check missionReplay for previous segment drone configs
  const segIdx = missionReplay.getCurrentSegmentIndex();
  if (segIdx >= 0) {
    const seg = missionReplay.getSegment(segIdx);
    if (seg && seg.drone_configs) {
      appendDebugLine(`[SOLVER] Using drone_configs from legacy segment ${segIdx}`);
      return JSON.parse(JSON.stringify(seg.drone_configs));
    } else {
      appendDebugLine(`[SOLVER] Legacy segment ${segIdx} exists but no drone_configs (seg=${!!seg}, drone_configs=${!!seg?.drone_configs})`);
    }
  }

  // First solve - no segments yet
  appendDebugLine(`[SOLVER] No previous segment drone_configs found - using UI state`);
  return null;
}

/**
 * Build env2 for replanning after checkpoint freeze
 * - Creates synthetic start nodes at frozen positions
 * - Marks visited targets
 * - Returns modified env and droneConfigs
 */
function buildCheckpointEnv() {
  appendDebugLine(`buildCheckpointEnv: checkpoint.active=${state.checkpoint?.active}, segments=${Object.keys(state.checkpoint?.segments || {}).join(",") || "NONE"}`);
  appendDebugLine(`buildCheckpointEnv: visitedTargets=[${state.visitedTargets.join(", ")}]`);

  // ========================================
  // NEW CLEAN PATH: Use SegmentedImportManager
  // ========================================
  if (segmentedImport.isActive()) {
    const segIdx = segmentedImport.getCurrentSegmentIndex();
    appendDebugLine(`buildCheckpointEnv: NEW MANAGER, segment ${segIdx}`);

    // Check if we have a checkpoint cut during animation (state.checkpoint.active)
    // In this case, we need to use the checkpoint split positions, not the original segment cut positions
    const hasCheckpointCut = state.checkpoint?.active && Object.keys(state.checkpoint.segments || {}).length > 0;
    appendDebugLine(`   hasCheckpointCut=${hasCheckpointCut}`);

    // Get env ready for solver (filtered targets, synthetic starts)
    const solverEnv = segmentedImport.getEnvForSolver();
    // Read drone configs directly from UI - what user sees is what solver uses
    const droneConfigs = readDroneConfigsFromUI();
    appendDebugLine(`   UI drone_configs: ${Object.entries(droneConfigs).map(([d,c]) => `D${d}:${c.enabled?'✓':'✗'}`).join(' ')}`);

    const unfrozen = segmentedImport.getUnfrozenTargets();
    const visited = segmentedImport.getVisitedTargetsForSegment(segIdx);

    // IMPORTANT: Also include any targets added during edit mode that aren't in the manager
    // The manager only knows about targets from the original import, not edits
    const managerTargetIds = new Set(unfrozen.map(t => t.id));
    const stateTargets = state.env?.targets || [];
    const newlyAddedTargets = stateTargets.filter(t =>
      !managerTargetIds.has(t.id) && !visited.includes(t.id)
    );

    if (newlyAddedTargets.length > 0) {
      appendDebugLine(`   Newly added targets from edits: ${newlyAddedTargets.map(t => t.id).join(",")}`);
      // Add newly added targets to solver env
      solverEnv.targets = [...solverEnv.targets, ...newlyAddedTargets];
    }

    // Also sync SAMs from state.env (user may have added/removed SAMs during editing)
    solverEnv.sams = JSON.parse(JSON.stringify(state.env?.sams || []));

    appendDebugLine(`   Targets for solver: ${solverEnv.targets.map(t => t.id).join(",")}`);
    appendDebugLine(`   Frozen (excluded): ${visited.join(",")}`);

    // If there's a checkpoint cut during animation, use those positions instead of original segment cuts
    if (hasCheckpointCut) {
      appendDebugLine(`   Using CHECKPOINT cut positions (mid-animation cut)`);
      solverEnv.synthetic_starts = solverEnv.synthetic_starts || {};

      Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
        if (!seg?.splitPoint) return;
        const nodeId = `D${did}_START`;
        solverEnv.synthetic_starts[nodeId] = { id: nodeId, x: seg.splitPoint[0], y: seg.splitPoint[1] };
        // Update ALL start fields to use synthetic start
        if (droneConfigs[did]) {
          droneConfigs[did].start_airport = nodeId;
          droneConfigs[did].start_id = nodeId;
          droneConfigs[did].home_airport = nodeId;
        }
        // Also update in solverEnv.drone_configs
        if (solverEnv.drone_configs && solverEnv.drone_configs[did]) {
          solverEnv.drone_configs[did].start_airport = nodeId;
          solverEnv.drone_configs[did].start_id = nodeId;
          solverEnv.drone_configs[did].home_airport = nodeId;
        }
        appendDebugLine(`   Drone ${did}: start from checkpoint [${seg.splitPoint[0].toFixed(2)}, ${seg.splitPoint[1].toFixed(2)}]`);
      });
    } else if (segIdx > 0) {
      // CRITICAL: For segment > 0, we must use synthetic starts from getEnvForSolver()
      // The solverEnv.drone_configs already has start_airport set to D{did}_START
      // We need to copy that to droneConfigs (which was based on state.droneConfigs)
      const cutPos = segmentedImport.getCutPositionForSegment(segIdx);
      if (cutPos) {
        Object.entries(cutPos).forEach(([did, pos]) => {
          const nodeId = `D${did}_START`;
          // Update ALL start fields to use synthetic start
          if (droneConfigs[did]) {
            droneConfigs[did].start_airport = nodeId;
            droneConfigs[did].start_id = nodeId;
            droneConfigs[did].home_airport = nodeId;
          }
          appendDebugLine(`   Drone ${did}: start from segment cut [${pos[0].toFixed(2)}, ${pos[1].toFixed(2)}] → start=${nodeId}`);
        });
      }
    }

    // Log end_airport for each drone to help debug
    Object.entries(droneConfigs).forEach(([did, cfg]) => {
      if (cfg.enabled) {
        appendDebugLine(`   Drone ${did}: start=${cfg.start_airport}, end=${cfg.end_airport}`);
      }
    });

    return {
      env: solverEnv,
      droneConfigs: droneConfigs,
      isCheckpointReplan: segIdx > 0 || hasCheckpointCut,
    };
  }

  // ========================================
  // LEGACY PATH: Old segmentInfo workflow
  // ========================================
  const isSegmentInfoWorkflow = state.importedSegmentCuts && state.importedSegmentCuts.length > 0;
  const currentSegIdx = missionReplay.getCurrentSegmentIndex();
  const currentSeg = missionReplay.getSegment(currentSegIdx);

  if (isSegmentInfoWorkflow && currentSegIdx > 0 && currentSeg?.cutPositions) {
    appendDebugLine(`buildCheckpointEnv: OLD segmentInfo workflow, segment ${currentSegIdx}, using cutPositions as start`);

    const baseEnv = state.env;
    const env2 = JSON.parse(JSON.stringify(baseEnv));
    env2.targets = (env2.targets || []).filter(t => !state.visitedTargets.includes(t.id));

    env2.synthetic_starts = env2.synthetic_starts || {};
    // Read drone configs directly from UI - what user sees is what solver uses
    const newDroneConfigs = readDroneConfigsFromUI();
    appendDebugLine(`   UI drone_configs: ${Object.entries(newDroneConfigs).map(([d,c]) => `D${d}:${c.enabled?'✓':'✗'}`).join(' ')}`);

    Object.entries(currentSeg.cutPositions).forEach(([did, pos]) => {
      if (!pos || pos.length !== 2) return;
      const nodeId = `D${did}_START`;
      env2.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };
      if (newDroneConfigs[did]) {
        newDroneConfigs[did].start_airport = nodeId;
        newDroneConfigs[did].start_id = nodeId;
        newDroneConfigs[did].home_airport = nodeId;
      }
    });

    return { env: env2, droneConfigs: newDroneConfigs, isCheckpointReplan: true };
  }

  // ========================================
  // LIVE CREATION PATH: Check MissionReplay for committed segments with cut positions
  // This handles the case where we've done cut → accept → animate → cut again
  // ========================================
  const liveSegmentIdx = missionReplay.getCurrentSegmentIndex();
  const liveCurrentSeg = missionReplay.getSegment(liveSegmentIdx);
  const hasLiveCutPositions = liveCurrentSeg?.cutPositions && Object.keys(liveCurrentSeg.cutPositions).length > 0;

  if (!state.checkpoint?.active && hasLiveCutPositions && liveSegmentIdx > 0) {
    appendDebugLine(`buildCheckpointEnv: LIVE CREATION path, segment ${liveSegmentIdx}, using saved cutPositions`);

    const baseEnv = state.env;
    const env2 = JSON.parse(JSON.stringify(baseEnv));

    // Filter out visited targets from the solver's target list
    env2.targets = (env2.targets || []).filter(t => !state.visitedTargets.includes(t.id));
    appendDebugLine(`   Filtered targets: ${env2.targets.map(t => t.id).join(",")}`);
    appendDebugLine(`   Visited (excluded): ${state.visitedTargets.join(",")}`);

    // Create synthetic starts from the saved cut positions
    env2.synthetic_starts = env2.synthetic_starts || {};
    // Read drone configs directly from UI - what user sees is what solver uses
    const newDroneConfigs = readDroneConfigsFromUI();
    appendDebugLine(`   UI drone_configs: ${Object.entries(newDroneConfigs).map(([d,c]) => `D${d}:${c.enabled?'✓':'✗'}`).join(' ')}`);

    Object.entries(liveCurrentSeg.cutPositions).forEach(([did, pos]) => {
      if (!pos || pos.length !== 2) return;
      const nodeId = `D${did}_START`;
      env2.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };
      if (newDroneConfigs[did]) {
        newDroneConfigs[did].start_airport = nodeId;
        newDroneConfigs[did].start_id = nodeId;
        newDroneConfigs[did].home_airport = nodeId;
        appendDebugLine(`   Drone ${did}: start from live cut [${pos[0].toFixed(2)}, ${pos[1].toFixed(2)}]`);
      }
    });

    return { env: env2, droneConfigs: newDroneConfigs, isCheckpointReplan: true };
  }

  if (!state.checkpoint?.active || !state.checkpoint.segments) {
    appendDebugLine("buildCheckpointEnv: NOT a checkpoint replan (checkpoint.active is false or no segments)");
    // Read drone configs directly from UI - what user sees is what solver uses
    const uiConfigs = readDroneConfigsFromUI();
    appendDebugLine(`   UI drone_configs: ${Object.entries(uiConfigs).map(([d,c]) => `D${d}:${c.enabled?'✓':'✗'}`).join(' ')}`);
    return { env: state.env, droneConfigs: uiConfigs, isCheckpointReplan: false };
  }

  // Deep copy current env
  const env2 = JSON.parse(JSON.stringify(state.env));

  // Read drone configs directly from UI - what user sees is what solver uses
  const newDroneConfigs = readDroneConfigsFromUI();
  appendDebugLine(`buildCheckpointEnv: CHECKPOINT path - using UI drone_configs`);
  appendDebugLine(`   UI drone_configs: ${Object.entries(newDroneConfigs).map(([d,c]) => `D${d}:${c.enabled?'✓':'✗'}`).join(' ')}`);

  // NEW: Add per-drone checkpoints to environment (C1-1, C1-2, etc.)
  // Each drone gets its own checkpoint at its cut position
  env2.checkpoints = env2.checkpoints || [];
  if (state.pendingCheckpoints && Object.keys(state.pendingCheckpoints).length > 0) {
    // Add each drone's checkpoint to the environment
    Object.entries(state.pendingCheckpoints).forEach(([did, cp]) => {
      // Only add if not already in the list
      if (!env2.checkpoints.some(c => c.id === cp.id)) {
        env2.checkpoints.push({ id: cp.id, x: cp.x, y: cp.y });
      }
      appendDebugLine(`   Added checkpoint ${cp.id} at [${cp.x.toFixed(2)}, ${cp.y.toFixed(2)}]`);

      // Update drone config to start from its checkpoint
      if (newDroneConfigs[did]) {
        newDroneConfigs[did].start_airport = cp.id;
        newDroneConfigs[did].start_id = cp.id;
        newDroneConfigs[did].home_airport = cp.id;

        // Calculate remaining fuel (original budget minus distance traveled)
        const seg = state.checkpoint.segments[did];
        const originalBudget = newDroneConfigs[did].fuel_budget || 150;
        const distanceTraveled = seg?.checkpointDist || 0;
        newDroneConfigs[did].fuel_budget = Math.max(0, originalBudget - distanceTraveled);

        appendDebugLine(`   Drone ${did}: start from checkpoint ${cp.id}, remaining fuel: ${newDroneConfigs[did].fuel_budget.toFixed(1)}`);
      }
    });
  } else {
    // Fallback to per-drone synthetic starts (legacy)
    env2.synthetic_starts = env2.synthetic_starts || {};
    Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
      if (!seg?.splitPoint) return;

      const nodeId = `D${did}_START`;
      env2.synthetic_starts[nodeId] = {
        id: nodeId,
        x: seg.splitPoint[0],
        y: seg.splitPoint[1],
      };

      if (newDroneConfigs[did]) {
        newDroneConfigs[did].start_airport = nodeId;
        newDroneConfigs[did].start_id = nodeId;
        newDroneConfigs[did].home_airport = nodeId;

        const originalBudget = newDroneConfigs[did].fuel_budget || 150;
        const distanceTraveled = seg.checkpointDist || 0;
        newDroneConfigs[did].fuel_budget = Math.max(0, originalBudget - distanceTraveled);
      }
    });
    appendDebugLine(`   Using legacy synthetic_starts: ${Object.keys(env2.synthetic_starts).join(",")}`);
  }

  // Mark visited targets (exclude them from planning or mark as completed)
  env2.visited_targets = [...state.visitedTargets];

  appendDebugLine(`buildCheckpointEnv: IS checkpoint replan. checkpoints=${(env2.checkpoints || []).map(c => c.id).join(",")}, visited=${env2.visited_targets.length}`);

  return {
    env: env2,
    droneConfigs: newDroneConfigs,
    isCheckpointReplan: true,
  };
}

/**
 * Get joined trajectories for full mission replay
 * Concatenates all segment trajectories per drone
 */
function getJoinedTrajectories() {
  const joined = {};

  state.missionHistory.forEach((segment) => {
    Object.entries(segment.trajectoriesByDrone || {}).forEach(([did, traj]) => {
      if (!joined[did]) {
        joined[did] = [];
      }

      if (joined[did].length > 0 && traj.length > 0) {
        // Check if we need to bridge (avoid duplicate points)
        const lastPoint = joined[did][joined[did].length - 1];
        const firstPoint = traj[0];
        if (Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
            Math.abs(lastPoint[1] - firstPoint[1]) < 0.001) {
          // Skip duplicate point
          joined[did] = joined[did].concat(traj.slice(1));
        } else {
          joined[did] = joined[did].concat(traj);
        }
      } else {
        joined[did] = joined[did].concat(traj);
      }
    });
  });

  return joined;
}

/**
 * Build combined routes from MissionReplay segments
 * For multi-segment missions, concatenates trajectories from frozen solutions
 * @param {number} [maxSegmentIndex] - If provided, only include segments 0 through maxSegmentIndex (inclusive)
 * Returns combined routes object ready for display/animation
 */
function buildCombinedRoutesFromSegments(maxSegmentIndex) {
  const totalSegments = missionReplay.getSegmentCount();
  // If maxSegmentIndex provided, limit to that; otherwise use all segments
  const segmentCount = (maxSegmentIndex !== undefined && maxSegmentIndex >= 0)
    ? Math.min(maxSegmentIndex + 1, totalSegments)
    : totalSegments;

  if (segmentCount <= 1 && maxSegmentIndex === undefined) {
    // Single segment or no segments - return state.routes as-is
    return state.routes;
  }

  appendDebugLine(`Building combined routes from segments 0-${segmentCount - 1} (of ${totalSegments} total)...`);

  const combinedRoutes = {};

  // Build a map of which drones are lost at which segment
  // lostDrones in segment N = drones that died at the END of segment N-1 (at cut CN)
  const droneLostAtSegment = {};
  for (let i = 1; i < totalSegments; i++) {
    const seg = missionReplay.getSegment(i);
    if (seg?.lostDrones) {
      seg.lostDrones.forEach(did => {
        droneLostAtSegment[String(did)] = i;
      });
    }
  }
  if (Object.keys(droneLostAtSegment).length > 0) {
    appendDebugLine(`  Lost drones map: ${Object.entries(droneLostAtSegment).map(([d, s]) => `D${d}@seg${s}`).join(', ')}`);
  }

  for (let i = 0; i < segmentCount; i++) {
    const seg = missionReplay.getSegment(i);
    if (!seg) {
      appendDebugLine(`  Seg ${i}: No segment`);
      continue;
    }

    // Get the FULL trajectory for this segment from the ORIGINAL JSON import.
    // segmentedImport stores the unmodified trajectories from the imported JSON file.
    // These are the complete trajectories (start → airport) that need truncation.
    // DO NOT use missionState.fullSolutionsPerSegment - it may contain re-solved
    // trajectories that are different from the original import.
    let solutionToUse = null;
    let solutionSource = 'none';
    if (segmentedImport.isActive()) {
      solutionToUse = segmentedImport.getSolutionForSegment(i);
      if (solutionToUse?.routes && Object.keys(solutionToUse.routes).length > 0) {
        solutionSource = 'segmentedImport';
      }
    }
    if (!solutionToUse || !solutionToUse.routes || Object.keys(solutionToUse.routes).length === 0) {
      solutionToUse = seg.solution;
      solutionSource = 'missionReplay';
    }

    if (!solutionToUse || !solutionToUse.routes) {
      appendDebugLine(`  Seg ${i}: No solution/routes`);
      continue;
    }

    // Check if NEXT segment has a cutDistance - if so, truncate THIS segment's trajectory there
    // IMPORTANT: cutDistance is CUMULATIVE from mission start, but each segment's trajectory
    // starts at distance 0. So we need segment-relative truncation distance.
    const nextSeg = missionReplay.getSegment(i + 1);
    const nextCutDistance = nextSeg?.cutDistance || null;
    // Get THIS segment's start distance (which is the cutDistance stored in THIS segment, or 0 for seg0)
    const thisCutDistance = seg.cutDistance || 0;
    // Segment-relative truncation = next cut - this segment's start
    const segmentRelativeTruncation = (nextCutDistance !== null && nextCutDistance > 0)
      ? nextCutDistance - thisCutDistance
      : null;

    // Debug: Log segment info with trajectory distances
    const routeCount = Object.keys(solutionToUse.routes).length;
    const firstDrone = Object.keys(solutionToUse.routes)[0];
    const firstTraj = solutionToUse.routes[firstDrone]?.trajectory || [];
    const firstTrajLen = firstTraj.length;
    let firstTrajDist = 0;
    for (let j = 1; j < firstTraj.length; j++) {
      const dx = firstTraj[j][0] - firstTraj[j-1][0];
      const dy = firstTraj[j][1] - firstTraj[j-1][1];
      firstTrajDist += Math.sqrt(dx*dx + dy*dy);
    }
    const usingFull = (i === 0 && solutionToUse === missionState.seg0FullSolution);
    appendDebugLine(`  Seg ${i} [${solutionSource}]: cutDist=${thisCutDistance?.toFixed(1)}, nextCut=${nextCutDistance?.toFixed(1) || 'null'}, truncAt=${segmentRelativeTruncation?.toFixed(1) || 'null'}, drones=${routeCount}, D${firstDrone}:${firstTrajLen}pts/${firstTrajDist.toFixed(1)}dist${usingFull ? ' (FULL)' : ''}`);

    Object.entries(solutionToUse.routes).forEach(([droneId, routeData]) => {
      // Check if this drone was lost at the START of the NEXT segment (i+1)
      // If so, this is the last segment for this drone - truncate at the kill position
      const lostAtSeg = droneLostAtSegment[droneId];
      if (lostAtSeg !== undefined && lostAtSeg <= i) {
        // Drone was already lost before or at this segment - skip adding more trajectory
        return;
      }

      let trajectory = routeData.trajectory || [];
      if (trajectory.length === 0) return;

      // Check if this drone is KILLED at the end of this segment (lost at segment i+1)
      // If so, truncate at the kill position (red diamond) - trajectory after that should vanish
      const droneKilledHere = (lostAtSeg === i + 1);
      const originalLen = trajectory.length;

      if (droneKilledHere) {
        // Drone is killed at the end of this segment - truncate at kill position
        // Try missionReplay segment first, then segmentedImport
        let killCutPositions = nextSeg?.cutPositions;
        if ((!killCutPositions || !killCutPositions[droneId]) && segmentedImport.isActive()) {
          const importedCutPos = segmentedImport.getCutPositionForSegment(i + 1);
          if (importedCutPos && importedCutPos[droneId]) {
            killCutPositions = importedCutPos;
          }
        }

        if (killCutPositions && killCutPositions[droneId]) {
          const killPos = killCutPositions[droneId];
          // Truncate trajectory at the kill position
          const truncated = truncateTrajectoryAtPosition(trajectory, killPos);
          trajectory = truncated;
          appendDebugLine(`    D${droneId}: KILLED at seg ${i+1}, truncated at [${killPos[0].toFixed(1)},${killPos[1].toFixed(1)}]: ${originalLen}→${trajectory.length} pts`);
        } else if (segmentRelativeTruncation !== null && segmentRelativeTruncation > 0) {
          // Fallback: truncate at distance
          const splitResult = split_polyline_at_distance(trajectory, segmentRelativeTruncation);
          trajectory = splitResult?.prefixPoints || trajectory;
          appendDebugLine(`    D${droneId}: KILLED at seg ${i+1}, truncated at dist ${segmentRelativeTruncation.toFixed(1)}: ${originalLen}→${trajectory.length} pts`);
        }
      } else {
        // Truncate trajectory at the NEXT segment's cut position (where the drone was when cut happened)
        // CRITICAL: Only truncate segments BEFORE the last one being built.
        // The last segment (the one we're currently animating) should keep its FULL trajectory.
        const isLastSegmentBeingBuilt = (i === segmentCount - 1);

        // Get cut positions from missionReplay segment first, then fallback to segmentedImport
        // For imported JSON files, cutPositions may be in segmentedImport but not in missionReplay
        let nextSegCutPositions = nextSeg?.cutPositions;
        if ((!nextSegCutPositions || !nextSegCutPositions[droneId]) && segmentedImport.isActive()) {
          // nextSeg is segment i+1, so its cut position is C{i+1}
          const importedCutPos = segmentedImport.getCutPositionForSegment(i + 1);
          if (importedCutPos && importedCutPos[droneId]) {
            nextSegCutPositions = importedCutPos;
          }
        }

        if (!isLastSegmentBeingBuilt && nextSegCutPositions && nextSegCutPositions[droneId]) {
          // USE POSITION-BASED TRUNCATION: truncate at the exact cut position
          // This is more accurate than distance-based truncation because each segment's
          // trajectory in the JSON goes from start to AIRPORT (full solver output),
          // not from start to next checkpoint.
          const cutPos = nextSegCutPositions[droneId];
          const truncated = truncateTrajectoryAtPosition(trajectory, cutPos);

          appendDebugLine(`    D${droneId}: Position-truncate at [${cutPos[0].toFixed(1)},${cutPos[1].toFixed(1)}]: ${originalLen}→${truncated.length} pts`);
          trajectory = truncated;
        } else if (!isLastSegmentBeingBuilt && segmentRelativeTruncation !== null && segmentRelativeTruncation > 0) {
          // Fallback: distance-based truncation if no cut positions available
          const splitResult = split_polyline_at_distance(trajectory, segmentRelativeTruncation);
          trajectory = splitResult?.prefixPoints || trajectory;

          if (droneId === Object.keys(solutionToUse.routes)[0]) {
            appendDebugLine(`    D${droneId}: Distance-truncate at ${segmentRelativeTruncation.toFixed(1)}: ${originalLen}→${trajectory.length} pts`);
          }
        }
      }

      // Determine the checkpoint ID for the END of this segment (if there's a next segment)
      // Format: C{nextSegIdx}-{droneId}
      const isLastSegmentBeingBuilt = (i === segmentCount - 1);
      const checkpointAtEnd = (!isLastSegmentBeingBuilt && nextSeg) ? `C${i + 1}-${droneId}` : null;

      // For killed drones, the checkpoint is their final waypoint
      const killCheckpoint = droneKilledHere ? `C${i + 1}-${droneId}` : null;

      // Get the route from solver - this has the targets in order for THIS drone
      const originalRoute = routeData.route || [];

      // Extract targets only (filter out synthetic starts and airports)
      const segTargetsInOrder = originalRoute.filter(wp => {
        const wpStr = String(wp);
        if (wpStr.match(/^D\d+_START$/)) return false;
        if (wpStr.match(/^A\d+$/)) return false;
        if (wpStr.match(/^C\d+-\d+$/)) return false; // Also filter existing checkpoints
        return true;
      });

      // Find start airport (first airport in route that's not a synthetic start)
      const startAirport = originalRoute.find(wp => {
        const wpStr = String(wp);
        return wpStr.match(/^A\d+$/) && originalRoute.indexOf(wp) === 0;
      });

      // Find end airport (last item if it's an airport)
      const lastItem = originalRoute[originalRoute.length - 1];
      const endAirport = (lastItem && String(lastItem).match(/^A\d+$/)) ? lastItem : null;

      if (!combinedRoutes[droneId]) {
        // First segment for this drone - copy route data
        // Build fullRoute: start airport + targets + checkpoint
        let fullRouteForSeg = [];

        // Add start airport (for first segment only)
        if (startAirport) {
          fullRouteForSeg.push(startAirport);
        }

        // Add targets in solver order
        fullRouteForSeg = fullRouteForSeg.concat(segTargetsInOrder);

        // Add checkpoint or kill checkpoint at end (NOT the airport yet)
        if (killCheckpoint) {
          fullRouteForSeg.push(killCheckpoint);
        } else if (checkpointAtEnd) {
          fullRouteForSeg.push(checkpointAtEnd);
        } else if (isLastSegmentBeingBuilt && endAirport) {
          // Only single segment mission - add end airport
          fullRouteForSeg.push(endAirport);
        }

        combinedRoutes[droneId] = {
          ...routeData,
          trajectory: [...trajectory],
          distance: routeData.distance || 0,
          fullRoute: fullRouteForSeg,
        };
      } else {
        // Subsequent segment - concatenate trajectory AND route
        const existing = combinedRoutes[droneId].trajectory;
        if (existing.length > 0 && trajectory.length > 0) {
          const lastPoint = existing[existing.length - 1];
          const firstPoint = trajectory[0];
          // Check for duplicate point at junction
          const isDuplicate = Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
                             Math.abs(lastPoint[1] - firstPoint[1]) < 0.001;
          if (isDuplicate) {
            // Skip duplicate
            combinedRoutes[droneId].trajectory = existing.concat(trajectory.slice(1));
          } else {
            combinedRoutes[droneId].trajectory = existing.concat(trajectory);
          }
        } else {
          combinedRoutes[droneId].trajectory = existing.concat(trajectory);
        }
        // Accumulate distance
        combinedRoutes[droneId].distance += (routeData.distance || 0);

        // CRITICAL: Also concatenate the route array so targets from later segments get checked
        // Filter out synthetic starts (D1_START, D2_START, etc.), checkpoints (C1-1, C1-2, etc.), and duplicates
        const existingRoute = combinedRoutes[droneId].route || [];
        const newRoute = routeData.route || [];
        const filteredNewRoute = newRoute.filter(wp => {
          const wpStr = String(wp);
          // Skip synthetic starts
          if (wpStr.match(/^D\d+_START$/)) return false;
          // Skip checkpoints (C{seg}-{drone} format) - they're segment boundaries, not mission waypoints
          if (wpStr.match(/^C\d+-\d+$/)) return false;
          // Skip if already in existing route
          if (existingRoute.includes(wp)) return false;
          return true;
        });
        combinedRoutes[droneId].route = existingRoute.concat(filteredNewRoute);

        // fullRoute: add this segment's NEW targets in order + checkpoint at end
        const existingFullRoute = combinedRoutes[droneId].fullRoute || [];

        // Add targets that were newly visited in this segment (in solver order)
        // Filter out any targets already in fullRoute (avoid duplicates)
        let filteredFullRoute = segTargetsInOrder.filter(t => !existingFullRoute.includes(t));

        // Add checkpoint at end if applicable
        if (killCheckpoint) {
          filteredFullRoute.push(killCheckpoint);
        } else if (checkpointAtEnd) {
          filteredFullRoute.push(checkpointAtEnd);
        } else if (isLastSegmentBeingBuilt && endAirport) {
          // Final segment - add the end airport
          filteredFullRoute.push(endAirport);
        }

        combinedRoutes[droneId].fullRoute = existingFullRoute.concat(filteredFullRoute);
      }
    });
  }

  const droneCount = Object.keys(combinedRoutes).length;
  // Log combined trajectory lengths AND distances
  const trajInfo = Object.entries(combinedRoutes).map(([did, rd]) => {
    const traj = rd.trajectory || [];
    let dist = 0;
    for (let j = 1; j < traj.length; j++) {
      const dx = traj[j][0] - traj[j-1][0];
      const dy = traj[j][1] - traj[j-1][1];
      dist += Math.sqrt(dx*dx + dy*dy);
    }
    return `D${did}:${traj.length}pts/${dist.toFixed(1)}dist`;
  }).join(', ');
  appendDebugLine(`Combined routes: ${droneCount} drones from ${segmentCount} segs → [${trajInfo}]`);
  return combinedRoutes;
}

/**
 * Regenerate trajectories for combined routes using /api/apply_sequence.
 * This ensures we get fresh SAM-aware trajectories from the combined route sequences
 * instead of concatenating pre-existing (and potentially overlapping) trajectories.
 *
 * @param {Object} combinedRoutes - Routes object with route arrays for each drone
 * @param {Object} env - Environment with airports, targets, sams, checkpoints
 * @returns {Promise<Object>} - Updated routes with regenerated trajectories
 */
async function regenerateTrajectoriesFromSequences(combinedRoutes, env) {
  if (!combinedRoutes || Object.keys(combinedRoutes).length === 0) {
    appendDebugLine(`[REGEN] No combined routes to regenerate`);
    return combinedRoutes;
  }

  appendDebugLine(`[REGEN] Regenerating trajectories for ${Object.keys(combinedRoutes).length} drones...`);

  // Build a map of ALL waypoints (airports, targets, checkpoints) with their coordinates
  const waypointMap = {}; // id -> {x, y}

  // Add airports
  (env.airports || []).forEach(a => {
    waypointMap[String(a.id)] = { x: a.x, y: a.y };
  });

  // Add targets
  (env.targets || []).forEach(t => {
    waypointMap[String(t.id)] = { x: t.x, y: t.y };
  });

  // Add checkpoints from all segment cut positions
  const totalSegs = missionReplay.getSegmentCount();
  for (let segIdx = 1; segIdx < totalSegs; segIdx++) {
    const seg = missionReplay.getSegment(segIdx);
    if (seg?.cutPositions) {
      Object.entries(seg.cutPositions).forEach(([droneId, pos]) => {
        if (pos && pos.length === 2) {
          const cpId = `C${segIdx}-${droneId}`;
          waypointMap[cpId] = { x: pos[0], y: pos[1] };
        }
      });
    }
  }

  appendDebugLine(`[REGEN] Waypoint map has ${Object.keys(waypointMap).length} entries`);

  // Create env with checkpoints added for /api/apply_sequence
  const checkpoints = [];
  for (let segIdx = 1; segIdx < totalSegs; segIdx++) {
    const seg = missionReplay.getSegment(segIdx);
    if (seg?.cutPositions) {
      Object.entries(seg.cutPositions).forEach(([droneId, pos]) => {
        if (pos && pos.length === 2) {
          checkpoints.push({ id: `C${segIdx}-${droneId}`, x: pos[0], y: pos[1] });
        }
      });
    }
  }
  const envWithCheckpoints = { ...env, checkpoints };

  const updatedRoutes = JSON.parse(JSON.stringify(combinedRoutes));
  const promises = [];

  for (const [droneId, routeData] of Object.entries(combinedRoutes)) {
    const trajectory = routeData.trajectory || [];
    if (trajectory.length < 2) {
      appendDebugLine(`  D${droneId}: Skipping (trajectory too short: ${trajectory.length})`);
      continue;
    }

    // Build sequence by walking trajectory and detecting waypoints
    // A waypoint is "visited" when trajectory passes within a small threshold of its coordinates
    const sequence = [];
    const visited = new Set();
    const THRESHOLD = 0.5; // coordinate match threshold

    // Get checkpoints for THIS drone
    const droneCheckpoints = [];
    for (let segIdx = 1; segIdx < totalSegs; segIdx++) {
      const cpId = `C${segIdx}-${droneId}`;
      if (waypointMap[cpId]) {
        droneCheckpoints.push(cpId);
      }
    }

    // Walk through trajectory points
    for (const pt of trajectory) {
      const px = pt[0];
      const py = pt[1];

      // Check all waypoints
      for (const [wpId, wpPos] of Object.entries(waypointMap)) {
        if (visited.has(wpId)) continue;

        // Only check checkpoints that belong to THIS drone
        if (wpId.match(/^C\d+-\d+$/)) {
          const cpDroneId = wpId.split('-')[1];
          if (cpDroneId !== droneId) continue;
        }

        const dx = Math.abs(px - wpPos.x);
        const dy = Math.abs(py - wpPos.y);
        if (dx < THRESHOLD && dy < THRESHOLD) {
          sequence.push(wpId);
          visited.add(wpId);
        }
      }
    }

    if (sequence.length < 2) {
      appendDebugLine(`  D${droneId}: Skipping (detected sequence too short: ${sequence.length})`);
      continue;
    }

    const sequenceStr = sequence.join(', ');
    appendDebugLine(`  D${droneId}: Sequence = ${sequenceStr}`);

    const droneConfig = state.droneConfigs[droneId] || {};
    const fuelBudget = droneConfig.fuel_budget || 300;

    // Call /api/apply_sequence to regenerate trajectory
    const promise = fetch("/api/apply_sequence", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        drone_id: droneId,
        sequence: sequenceStr,
        env: envWithCheckpoints,
        fuel_budget: fuelBudget
      })
    })
      .then(res => res.json())
      .then(data => {
        if (data.success && data.trajectory && data.trajectory.length > 0) {
          updatedRoutes[droneId].trajectory = data.trajectory;
          updatedRoutes[droneId].distance = data.distance;
          updatedRoutes[droneId].points = data.points;
          appendDebugLine(`  D${droneId}: ✓ Regenerated ${data.trajectory.length} pts, dist=${data.distance?.toFixed(1)}`);
        } else {
          appendDebugLine(`  D${droneId}: ✗ Failed to regenerate: ${data.error || 'empty trajectory'}`);
        }
      })
      .catch(err => {
        appendDebugLine(`  D${droneId}: ✗ Error: ${err.message}`);
      });

    promises.push(promise);
  }

  // Wait for all trajectory regenerations to complete
  await Promise.all(promises);

  const trajInfo = Object.entries(updatedRoutes).map(([did, rd]) => `D${did}:${rd.trajectory?.length || 0}`).join(', ');
  appendDebugLine(`[REGEN] Complete: [${trajInfo}]`);

  return updatedRoutes;
}

// resetMission() is now defined in the Mission Mode Action Handlers section

async function runPlanner() {
  if (!state.env) {
    alert("No environment loaded yet.");
    return;
  }

  // Check if we can solve in current state
  const perms = getUiPermissions();
  if (!perms.canSolve) {
    appendDebugLine("Cannot run planner in current state");
    return;
  }

  // Cancel any existing solve request
  if (state.solveAbortController) {
    state.solveAbortController.abort();
  }

  // Create new abort controller
  state.solveAbortController = new AbortController();

  // Grey out the Run Planner button while solving
  const btnRun = $("btn-run-planner");
  if (btnRun) {
    btnRun.disabled = true;
    btnRun.textContent = "Solving...";
    btnRun.style.opacity = "0.5";
    btnRun.style.cursor = "not-allowed";
  }

  // Get the selected allocation strategy
  const strategySelect = $("allocation-strategy");
  const strategy = strategySelect ? strategySelect.value : "efficient";

  // Check if this is a checkpoint replan (solve from frozen positions)
  const isCheckpointMode = missionState.mode === MissionMode.CHECKPOINT;
  const { env: envToSolve, droneConfigs: configsToSolve, isCheckpointReplan } = buildCheckpointEnv();

  // Detect lost drones: drones that have cut positions but are now disabled
  // These are drones that were "lost" at this checkpoint
  state.pendingLostDrones = [];
  if (isCheckpointReplan && state.pendingCutPositions) {
    Object.keys(state.pendingCutPositions).forEach(did => {
      const cfg = configsToSolve[did] || state.droneConfigs[did];
      if (cfg && !cfg.enabled) {
        state.pendingLostDrones.push(did);
        appendDebugLine(`🔴 Drone ${did} LOST at checkpoint (had cut position but now disabled)`);
      }
    });
  }

  // If this is the first solve (no history), save the initial snapshot
  if (missionState.committedSegments.length === 0 && !isCheckpointMode) {
    saveInitialEnvSnapshot();
    // Also set the accepted env if not already set
    if (!missionState.acceptedEnv) {
      missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));
    }
  }

  const payload = {
    env: envToSolve,
    drone_configs: configsToSolve,
    allocation_strategy: strategy,
    is_checkpoint_replan: isCheckpointReplan,
    visited_targets: state.visitedTargets,
  };

  const modeLabel = isCheckpointReplan ? "CHECKPOINT REPLAN" : "FRESH SOLVE";
  appendDebugLine(`➡ [${modeLabel}] Sending /api/solve_with_allocation (strategy: ${strategy})...`);
  appendDebugLine(`   Payload targets: ${(envToSolve.targets||[]).map(t=>t.id).join(",")}`);
  appendDebugLine(`   Visited targets: ${state.visitedTargets.join(",") || "none"}`);
  appendDebugLine(`   state.env targets: ${(state.env.targets||[]).map(t=>t.id).join(",")}`);
  // Show fuel budgets for each drone
  const fuelInfo = Object.entries(configsToSolve).map(([did, cfg]) =>
    `D${did}:${cfg.fuel_budget?.toFixed(0) || '?'}`
  ).join(", ");
  appendDebugLine(`   Fuel budgets: ${fuelInfo}`);
  // Show which drones are enabled/disabled (critical for debugging lost drone persistence)
  const enabledInfo = Object.entries(configsToSolve).map(([did, cfg]) =>
    `D${did}:${cfg.enabled ? '✓' : '✗'}`
  ).join(", ");
  appendDebugLine(`   Enabled drones: ${enabledInfo}`);
  // Show start/end airports for each drone (for debugging end_airport issue)
  const airportInfo = Object.entries(configsToSolve).map(([did, cfg]) =>
    `D${did}:${cfg.start_airport || '?'}→${cfg.end_airport || '?'}`
  ).join(", ");
  appendDebugLine(`   Start→End airports: ${airportInfo}`);
  let bodyStr;
  try {
    bodyStr = JSON.stringify(payload);
  } catch (jsonErr) {
    appendDebugLine(`❌ Failed to serialize payload: ${jsonErr}`);
    $("debug-output").textContent = "Error serializing payload: " + jsonErr;
    return;
  }

  try {
    const resp = await fetch("/api/solve_with_allocation", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: bodyStr,
      signal: state.solveAbortController.signal,
    });

    // Check if response is OK before parsing
    if (!resp.ok) {
      const errorText = await resp.text();
      appendDebugLine(`❌ Server error (${resp.status}): ${errorText.substring(0, 200)}`);
      $("debug-output").textContent = `Server returned ${resp.status}: ${errorText.substring(0, 500)}`;
      return;
    }

    const data = await resp.json();
    $("debug-output").textContent = JSON.stringify(data, null, 2);

    if (!data || data.success === false) {
      appendDebugLine(`❌ Planner reported an error: ${JSON.stringify(data?.errors || [])}`);
      return;
    }

    // Log raw solver response for debugging trajectory issues
    if (isCheckpointReplan) {
      Object.entries(data.routes || {}).forEach(([did, r]) => {
      });
    }

    // For checkpoint replans: DON'T splice frozen prefix into the new trajectory!
    // Each segment stores its own trajectory independently:
    // - Segment 0: A → ... → c1 (truncated at cut)
    // - Segment 1: c1 → targets → end (solver's result, starting from synthetic start)
    // The combination happens during animation via buildCombinedRoutesFromSegments
    if (isCheckpointReplan) {
      appendDebugLine(`📏 Cut distance for this segment: ${state.pendingCutDistance?.toFixed(1) || "null"}`);
      // Log the solver's trajectory (should start from synthetic start, NOT airport)
      Object.entries(data.routes || {}).forEach(([did, routeData]) => {
        const traj = routeData.trajectory || [];
        if (traj.length > 0) {
          const firstPt = `(${traj[0][0].toFixed(1)},${traj[0][1].toFixed(1)})`;
          const lastPt = `(${traj[traj.length-1][0].toFixed(1)},${traj[traj.length-1][1].toFixed(1)})`;
          appendDebugLine(`📎 D${did}: solver trajectory ${traj.length}pts [${firstPt}→${lastPt}]`);
        }
      });
    }

    // Store as draft solution (two-phase commit)
    // IMPORTANT: Deep copy routes to prevent modifications to state.routes from affecting the draft
    // Also save checkpoint state so it can be restored if the draft is discarded
    // CRITICAL: Save drone_configs at Solve time so Accept uses the actual config used for solving
    // DEBUG: Log cutDistance capture at solve time
    const solverCutDistance = isCheckpointReplan ? state.pendingCutDistance : null;
    appendDebugLine(`[SOLVE] Creating draftSolution: isCheckpointReplan=${isCheckpointReplan}, cutDistance=${solverCutDistance?.toFixed(1) || 'null'}, pendingCutDistance=${state.pendingCutDistance?.toFixed(1) || 'null'}`);

    missionState.draftSolution = {
      sequences: JSON.parse(JSON.stringify(data.sequences || {})),
      routes: JSON.parse(JSON.stringify(data.routes || {})),
      wrappedPolygons: JSON.parse(JSON.stringify(data.wrapped_polygons || [])),
      allocations: JSON.parse(JSON.stringify(data.allocations || {})),
      allocationStrategy: data.allocation_strategy || null,
      distanceMatrix: data.distance_matrix ? JSON.parse(JSON.stringify(data.distance_matrix)) : null,
      isCheckpointReplan: isCheckpointReplan,
      checkpointSegments: isCheckpointReplan ? JSON.parse(JSON.stringify(state.checkpoint.segments)) : null,
      cutDistance: solverCutDistance,
      // Save cut positions so they can be restored on discard
      pendingCutPositions: state.pendingCutPositions ? JSON.parse(JSON.stringify(state.pendingCutPositions)) : null,
      // Save drone_configs from solver (includes previous segment's disabled drones)
      // Use configsToSolve (from buildCheckpointEnv) which carries forward disabled drones from previous segments
      droneConfigs: JSON.parse(JSON.stringify(configsToSolve || {})),
      // Save lost drones so applyDraftSolutionToUI can purge them
      lostDrones: state.pendingLostDrones?.length > 0 ? [...state.pendingLostDrones] : [],
    };

    // Apply the draft solution visually (but not committed yet)
    applyDraftSolutionToUI(missionState.draftSolution);

    // For checkpoint replans: check if any routes are missing proper trajectories
    // and regenerate them if needed
    if (isCheckpointReplan) {
      const routesNeedingRegeneration = [];
      Object.entries(state.routes || {}).forEach(([did, routeData]) => {
        if (routeData.route && routeData.route.length > 2) {
          // Check if trajectory seems incomplete (only has frozen portion)
          const frozenCount = missionState.draftSolution.checkpointSegments?.[did]?.prefix?.length || 0;
          const trajLen = routeData.trajectory?.length || 0;
          if (trajLen <= frozenCount + 1) {
            routesNeedingRegeneration.push(did);
          }
        }
      });

      if (routesNeedingRegeneration.length > 0) {
        // Regenerate trajectories for incomplete routes
        await regenerateTrajectories();
      }
    }

    // Clear checkpoint state after successful replan (now in draft)
    state.checkpoint = {
      active: false,
      pct: 0.5,
      segments: {},
    };

    // Transition to DRAFT_READY
    setMissionMode(MissionMode.DRAFT_READY, isCheckpointReplan ? "checkpoint replan ready" : "solution ready");

    const resultLabel = isCheckpointReplan ? "✅ Checkpoint replan ready for review." : "✅ Solution ready for review.";
    appendDebugLine(resultLabel);
  } catch (err) {
    if (err.name === 'AbortError') {
      appendDebugLine("⚠️ Solve request canceled.");
      $("debug-output").textContent = "Solve request was canceled.";
    } else {
      $("debug-output").textContent = "Error calling /api/solve_with_allocation: " + err;
      appendDebugLine("❌ Error calling /api/solve_with_allocation: " + err);
    }
  } finally {
    state.solveAbortController = null;
    // Re-enable the Run Planner button
    const btnRun = $("btn-run-planner");
    if (btnRun) {
      btnRun.disabled = false;
      btnRun.textContent = "Run Planner";
      btnRun.style.opacity = "1";
      btnRun.style.cursor = "pointer";
    }
    updateButtonStates();
  }
}

/**
 * Apply a draft solution to the UI for visualization
 * (without committing it to missionState.committedSegments)
 */
function applyDraftSolutionToUI(draft) {
  if (!draft) return;

  // Debug: Log what's in the draft routes before applying
  Object.entries(draft.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
    }
  });

  // Deep copy to allow UI modifications without affecting the stored draft
  state.sequences = JSON.parse(JSON.stringify(draft.sequences || {}));

  // For checkpoint replans: MERGE frozen trajectory with new segment trajectory
  // - state.routes[did].trajectory = frozen prefix (A → c1) from the cut
  // - draft.routes[did].trajectory = new segment (c1 → end) from solver
  // - combined = frozen + new for display
  if (draft.isCheckpointReplan) {
    const mergedRoutes = { ...state.routes };
    Object.entries(draft.routes || {}).forEach(([did, routeData]) => {
      // Use the frozen prefix captured at cut time, NOT whatever is in state.routes
      // This ensures checkpoint visual splice is based on authoritative frozen data
      let frozenTraj = draft.checkpointSegments?.[did]?.prefix
        || mergedRoutes[did]?.trajectory
        || [];
      const newTraj = routeData.trajectory || [];

      // If frozenTraj came from state.routes (not checkpointSegments), it may be FULL trajectory
      // We need to truncate it at the cut position before merging
      const usedStateRoutes = !draft.checkpointSegments?.[did]?.prefix && mergedRoutes[did]?.trajectory;
      if (usedStateRoutes && frozenTraj.length > 0) {
        // Try to get cut position from pendingCutPositions or newTraj start point
        const cutPos = draft.pendingCutPositions?.[did] || (newTraj.length > 0 ? newTraj[0] : null);
        if (cutPos) {
          // Truncate frozenTraj at the cut position
          const truncated = truncateTrajectoryAtPosition(frozenTraj, cutPos);
          if (truncated.length < frozenTraj.length) {
            appendDebugLine(`[applyDraft] D${did}: Truncated frozen traj at cut pos [${cutPos[0].toFixed(1)},${cutPos[1].toFixed(1)}]: ${frozenTraj.length} → ${truncated.length} points`);
            frozenTraj = truncated;
          }
        }
      }

      // DEBUG: Log what we're combining
      const frozenFirst = frozenTraj.length > 0 ? `(${frozenTraj[0][0].toFixed(1)},${frozenTraj[0][1].toFixed(1)})` : "none";
      const frozenLast = frozenTraj.length > 0 ? `(${frozenTraj[frozenTraj.length-1][0].toFixed(1)},${frozenTraj[frozenTraj.length-1][1].toFixed(1)})` : "none";
      const newFirst = newTraj.length > 0 ? `(${newTraj[0][0].toFixed(1)},${newTraj[0][1].toFixed(1)})` : "none";
      // Concatenate frozen + new, avoiding duplicate point at junction
      let combinedTraj;
      if (frozenTraj.length > 0 && newTraj.length > 0) {
        const lastFrozen = frozenTraj[frozenTraj.length - 1];
        const firstNew = newTraj[0];
        const isDuplicate = Math.abs(lastFrozen[0] - firstNew[0]) < 0.001 &&
                           Math.abs(lastFrozen[1] - firstNew[1]) < 0.001;
        combinedTraj = isDuplicate ? frozenTraj.concat(newTraj.slice(1)) : frozenTraj.concat(newTraj);
      } else {
        combinedTraj = frozenTraj.concat(newTraj);
      }

      mergedRoutes[did] = {
        ...routeData,
        trajectory: combinedTraj,
        // Use the new segment's distance only - don't accumulate across segments
        // The display should show current segment distance, not cumulative
        distance: routeData.distance || 0,
      };
    });
    // Preserve routes for drones NOT in the draft (completed drones)
    state.routes = mergedRoutes;

    // Ensure trajectoryVisible is set for ALL drones in merged routes
    Object.keys(mergedRoutes).forEach(did => {
      state.trajectoryVisible[did] = true;
    });
  } else {
    // Not a checkpoint replan - just replace as before
    state.routes = JSON.parse(JSON.stringify(draft.routes || {}));
  }

  state.wrappedPolygons = JSON.parse(JSON.stringify(draft.wrappedPolygons || []));
  state.allocations = JSON.parse(JSON.stringify(draft.allocations || {}));
  state.allocationStrategy = draft.allocationStrategy || null;
  state.distanceMatrix = draft.distanceMatrix ? JSON.parse(JSON.stringify(draft.distanceMatrix)) : null;

  // After checkpoint replan, update animation drones to show at their new starting positions
  // Use draft.cutDistance (captured at solve time) instead of state.pendingCutDistance (may be stale/cleared)
  const cutDist = (draft.cutDistance != null) ? draft.cutDistance : state.pendingCutDistance;
  if (draft.isCheckpointReplan && state.animation.drones && cutDist != null) {
    Object.keys(state.animation.drones).forEach((did) => {
      if (state.routes[did]) {
        const routeInfo = state.routes[did];
        const trajectory = routeInfo.trajectory || [];

        // Recalculate cumulative distances for the new spliced trajectory
        let cumulativeDistances = [0];
        let totalDistance = 0;
        if (trajectory.length >= 2) {
          for (let i = 1; i < trajectory.length; i++) {
            const dx = trajectory[i][0] - trajectory[i - 1][0];
            const dy = trajectory[i][1] - trajectory[i - 1][1];
            totalDistance += Math.sqrt(dx * dx + dy * dy);
            cumulativeDistances.push(totalDistance);
          }
        }

        state.animation.drones[did] = {
          progress: totalDistance > 0 ? cutDist / totalDistance : 0,
          distanceTraveled: cutDist,
          animating: false,  // Not animating yet, just showing position
          cumulativeDistances,
          totalDistance,
        };
      }
    });
  }

  // Debug: Log what allocations we received
  appendDebugLine("🎯 Allocations received: " + JSON.stringify(draft.allocations));

  // Display allocations in Env tab
  updateAllocationDisplay(state.allocations, state.allocationStrategy);

  const cur = state.currentDroneForSeq;
  const curSeq = state.sequences[cur] || "";
  const seqInput = $("sequence-input");
  if (seqInput) seqInput.value = curSeq;

  updateStatsFromRoutes();
  drawEnvironment();
}

// Cancel any running solve request
function cancelSolve() {
  if (state.solveAbortController) {
    state.solveAbortController.abort();
    appendDebugLine("⚠️ Canceling solve request...");
  } else {
    appendDebugLine("No solve request in progress.");
  }
}

// ----------------------------------------------------
// Initial environment load from backend (optional)
// ----------------------------------------------------
async function initialLoadEnv() {
  try {
    const resp = await fetch("/api/environment");
    if (!resp.ok) {
      appendDebugLine("No /api/environment available, start by importing JSON.");
      drawEnvironment();
      return;
    }
    const data = await resp.json();
    if (!data || data.success === false) {
      appendDebugLine("Backend /api/environment responded but not success.");
      drawEnvironment();
      return;
    }
    state.env = data.environment || null;
    state.envFilename = data.filename || null;
    if (state.envFilename) {
      $("env-filename").textContent = state.envFilename;
    }

    // Reset mission when loading a new environment from backend
    state.missionId = null;
    state.routes = {};
    state.sequences = {};
    state.trajectoryVisible = {};

    appendDebugLine("Loaded environment from backend.");
    initDroneConfigsFromEnv();

    // Fetch SAM wrapping if there are SAMs in the environment
    if (state.env && state.env.sams && state.env.sams.length > 0) {
      fetchWrappedPolygons(true).then(() => drawEnvironment());
    } else {
      drawEnvironment();
    }
  } catch (err) {
    appendDebugLine("Failed to fetch /api/environment: " + err);
    drawEnvironment();
  }
}

// ----------------------------------------------------
// Tab switching
// ----------------------------------------------------
function attachTabSwitching() {
  const tabBtns = document.querySelectorAll(".tab-btn");
  const tabPanes = document.querySelectorAll(".tab-pane");

  tabBtns.forEach((btn) => {
    btn.addEventListener("click", () => {
      const tabId = btn.getAttribute("data-tab");

      // Remove active from all buttons and panes
      tabBtns.forEach((b) => b.classList.remove("active"));
      tabPanes.forEach((p) => p.classList.remove("active"));

      // Add active to clicked button and corresponding pane
      btn.classList.add("active");
      const pane = document.getElementById(`tab-${tabId}`);
      if (pane) pane.classList.add("active");
    });
  });
}

// ----------------------------------------------------
// Trajectory visibility controls
// ----------------------------------------------------
function updateTrajectoryButtonStates() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`traj-d${did}`);
    if (btn) {
      if (state.trajectoryVisible[String(did)]) {
        btn.classList.add("active");
      } else {
        btn.classList.remove("active");
      }
    }
  }

  // Update "All" button - active if all are visible
  const allBtn = $("traj-all");
  if (allBtn) {
    const allVisible = Object.values(state.trajectoryVisible).every((v) => v);
    if (allVisible) {
      allBtn.classList.add("active");
    } else {
      allBtn.classList.remove("active");
    }
  }
}

function toggleTrajectory(did) {
  state.trajectoryVisible[did] = !state.trajectoryVisible[did];
  updateTrajectoryButtonStates();
  drawEnvironment();
  appendDebugLine(`Trajectory D${did}: ${state.trajectoryVisible[did] ? "shown" : "hidden"}`);
}

function toggleAllTrajectories() {
  const visibleValues = Object.values(state.trajectoryVisible);
  // If no trajectories set yet, or none visible, show all; otherwise toggle off
  const allVisible = visibleValues.length > 0 && visibleValues.every((v) => v);
  const newState = !allVisible;

  for (let did = 1; did <= 5; did++) {
    state.trajectoryVisible[String(did)] = newState;
  }

  updateTrajectoryButtonStates();
  drawEnvironment();
  appendDebugLine(`All trajectories: ${newState ? "shown" : "hidden"}`);
}

function attachTrajectoryControls() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`traj-d${did}`);
    if (btn) {
      btn.addEventListener("click", () => toggleTrajectory(String(did)));
    }
  }

  const allBtn = $("traj-all");
  if (allBtn) {
    allBtn.addEventListener("click", toggleAllTrajectories);
  }

  // Initialize button states
  updateTrajectoryButtonStates();
}

function freezeAtCheckpoint() {
  // Freeze drones at their CURRENT position (distanceTraveled), not at a percentage

  if (!state.animation || !state.animation.drones || Object.keys(state.animation.drones).length === 0) {
    appendDebugLine("No animation data to freeze (drones: " + JSON.stringify(state.animation?.drones) + ")");
    return;
  }

  // Debug: confirm which drones are being frozen
  appendDebugLine(`freezeAtCheckpoint: drones=${Object.keys(state.animation?.drones || {}).join(",")}`);

  // Ensure checkpoint container exists
  state.checkpoint = state.checkpoint || { active: false, pct: 0, segments: {} };
  state.checkpoint.active = true;
  state.checkpoint.segments = {};

  // Initialize per-drone cut positions map (kept for backward compatibility)
  state.pendingCutPositions = {};

  // NEW: Capture single cut distance (same for all drones since they fly at same speed)
  // Use missionDistance from animation state (authoritative source for cumulative distance)
  state.pendingCutDistance = state.animation.missionDistance || 0;
  appendDebugLine(`✂️ Cut at distance: ${state.pendingCutDistance.toFixed(1)} (missionDistance=${state.animation.missionDistance?.toFixed(1) || 'null'})`);

  // Create checkpoint segment number for this cut (C1, C2, C3, ...)
  // Each drone gets its own checkpoint: C1-1, C1-2, etc.
  state.checkpointSegmentCounter = (state.checkpointSegmentCounter || 0) + 1;
  const checkpointSegNum = state.checkpointSegmentCounter;

  // Track which targets are visited in the current (frozen) routes
  const targetsVisitedThisSegment = [];

  // Initialize per-drone checkpoints map
  state.pendingCheckpoints = {};

  Object.entries(state.animation.drones).forEach(([did, droneState]) => {
    const traj = state.routes[did]?.trajectory || [];
    if (traj.length < 2 || droneState.totalDistance <= 0) return;

    // Use the drone's CURRENT distanceTraveled (actual position during animation)
    const currentDist = droneState.distanceTraveled || 0;

    // Compute whether this drone has finished its route
    const totalDist = droneState.totalDistance || 0;
    const finished = totalDist > 0 && currentDist >= (totalDist - 1e-6); // epsilon
    appendDebugLine(`✂️ D${did}: currentDist=${currentDist.toFixed(1)}, totalDist=${totalDist.toFixed(1)}, finished=${finished}`);

    const { prefixPoints, suffixPoints, splitPoint } =
      split_polyline_at_distance(traj, currentDist);

    // Always store checkpoint segment info (so UI can show "where it is")
    // But for finished drones we treat splitPoint as the last point and do not "cut" them.
    state.checkpoint.segments[did] = {
      prefix: prefixPoints,
      suffix: suffixPoints,
      splitPoint,
      checkpointDist: currentDist,
      finishedAtCut: finished, // flag for debugging / later logic
    };

    // Only record cut positions for drones that are actually still moving.
    if (!finished && splitPoint && splitPoint.length === 2) {
      state.pendingCutPositions[did] = [splitPoint[0], splitPoint[1]];
      // Create per-drone checkpoint: C{segment}-{drone} (e.g., C1-1, C1-2)
      const checkpointId = `C${checkpointSegNum}-${did}`;
      state.pendingCheckpoints[did] = {
        id: checkpointId,
        x: splitPoint[0],
        y: splitPoint[1],
      };
      appendDebugLine(`✂️ Created checkpoint ${checkpointId} at [${splitPoint[0].toFixed(2)}, ${splitPoint[1].toFixed(2)}]`);
    }

    // Mark ONLY targets that have been passed (are in the prefix trajectory)
    // Targets ahead of the frozen position should remain available for replanning
    const route = state.routes[did]?.route || [];
    const env = state.env;

    // Build cumulative distances for the FULL trajectory to find target positions
    const fullTraj = traj;
    const cumDist = [0];
    for (let i = 1; i < fullTraj.length; i++) {
      const dx = fullTraj[i][0] - fullTraj[i - 1][0];
      const dy = fullTraj[i][1] - fullTraj[i - 1][1];
      cumDist.push(cumDist[i - 1] + Math.sqrt(dx * dx + dy * dy));
    }

    route.forEach(wp => {
      if (!String(wp).startsWith("T")) return;
      if (state.visitedTargets.includes(wp)) return;

      // Find target position
      const target = env.targets?.find(t => t.id === wp);
      if (!target) return;

      // Find the trajectory point closest to this target (same logic as animation loop)
      let minDist = Infinity;
      let closestIdx = -1;
      fullTraj.forEach((pt, idx) => {
        const dist = Math.sqrt(
          Math.pow(pt[0] - target.x, 2) +
          Math.pow(pt[1] - target.y, 2)
        );
        if (dist < minDist) {
          minDist = dist;
          closestIdx = idx;
        }
      });

      // Target is visited if:
      // 1. The closest trajectory point is within 20 units (same threshold as animation)
      // 2. We've traveled past that point (currentDist >= cumDist[closestIdx])
      const targetDist = closestIdx >= 0 ? cumDist[closestIdx] : Infinity;
      const isPassed = closestIdx >= 0 && minDist < 20.0 && currentDist >= targetDist;

      if (isPassed) {
        targetsVisitedThisSegment.push(wp);
        state.visitedTargets.push(wp);
      } else {
      }
    });

    // Stop animating but keep current position
    droneState.animating = false;

    // Only truncate displayed trajectory for drones still in-flight.
    // For finished drones, preserve the full trajectory exactly as-is.
    if (!finished) {
      state.routes[did]._fullTrajectory = state.routes[did]._fullTrajectory || traj;
      state.routes[did].trajectory = prefixPoints;
    } else {
      // Make sure full trajectory is preserved for later redraws/replays
      state.routes[did]._fullTrajectory = state.routes[did]._fullTrajectory || traj;
      // leave state.routes[did].trajectory unchanged
    }
  });

  if (targetsVisitedThisSegment.length > 0) {
    appendDebugLine(`🎯 Marked ${targetsVisitedThisSegment.length} targets as visited (in frozen trajectory): ${targetsVisitedThisSegment.join(', ')}`);
    appendDebugLine(`   These targets will show GREEN X and be excluded from replanning`);
  } else {
    appendDebugLine(`   No targets in frozen trajectories - all remain available for replanning`);
  }

  // Stop the RAF loop cleanly
  state.animation.active = false;
  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }

  const avgPct = Object.values(state.animation.drones)
    .filter(d => d.totalDistance > 0)
    .map(d => (d.distanceTraveled || 0) / d.totalDistance)
    .reduce((a, b, _, arr) => a + b / arr.length, 0) * 100;

  // Debug: log captured cut positions and checkpoints
  appendDebugLine(`✂️ pendingCutPositions=${JSON.stringify(state.pendingCutPositions || {})}`);
  const checkpointIds = Object.values(state.pendingCheckpoints || {}).map(cp => cp.id);
  appendDebugLine(`✂️ pendingCheckpoints: ${checkpointIds.join(", ") || "none"}`);

  updateAnimationButtonStates();
  drawEnvironment();
  appendDebugLine(`Frozen at ${avgPct.toFixed(0)}% progress`);
}
// ----------------------------------------------------
// Animation controls
// ----------------------------------------------------
function startAnimation(droneIds) {
  const perms = getUiPermissions();

  // Debug: log which drones are being passed to animation
  appendDebugLine(`startAnimation called with droneIds=[${(droneIds || []).join(",")}]`);

  // Check if we can animate or resume
  if (!perms.canAnimate && !perms.canResume) {
    appendDebugLine("⚠️ Cannot animate in current state. Accept a solution first.");
    return;
  }

  // If we're resuming from pause, restore context
  if (perms.canResume && missionState.pauseContext) {
    resumeAnimation();
    return;
  }

  // Stop any existing animation
  stopAnimation();

  // Check if we're in a frozen checkpoint state WITHOUT a replan
  // If checkpoint is active (frozen) but no cut distance exists, block animation
  const hasFrozenCheckpoint = state.checkpoint?.active ||
    Object.keys(state.checkpoint?.segments || {}).length > 0;
  const hasReplanData = state.pendingCutDistance !== null && state.pendingCutDistance > 0;

  if (hasFrozenCheckpoint && !hasReplanData) {
    appendDebugLine("⚠️ Cannot animate: checkpoint is frozen. Run Planner to replan from checkpoint, or press R to reset.");
    return;
  }

  // Clear checkpoint state so frozen drones don't persist
  if (state.checkpoint) {
    state.checkpoint.active = false;
    state.checkpoint.segments = {};
  }

  // Restore full trajectories if they were truncated by a previous freeze
  // BUT NOT if this is a checkpoint replan (we want to keep the spliced trajectory)
  Object.keys(state.routes || {}).forEach((did) => {
    if (state.routes[did]._fullTrajectory) {
      state.routes[did].trajectory = state.routes[did]._fullTrajectory;
      delete state.routes[did]._fullTrajectory;
    }
  });

  // Debug: Log trajectories at animation start
  Object.entries(state.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
    }
  });

  // Initialize animation state for selected drones
  state.animation.active = true;
  state.animation.drones = {};

  // NEW: Single mission distance tracker (same for all drones since they fly at same speed)
  // Get starting distance from MissionReplay (0 for segment 0, cutDistance for checkpoint replans)
  // Exception: After Reset, we force start at 0 even though missionReplay index is 1 (for marker drawing)
  if (state.animation.forceStartDistanceZero) {
    state.animation.missionDistance = 0;
    state.animation.forceStartDistanceZero = false; // Clear the flag
    appendDebugLine(`🎬 Mission starting at distance: 0 (forced after Reset)`);
  } else {
    state.animation.missionDistance = missionReplay.getStartingDistance();
    appendDebugLine(`🎬 Mission starting at distance: ${state.animation.missionDistance.toFixed(1)}`);
  }

  // Enable debug stop at splice point if this is a checkpoint replan
  const isCheckpointReplan = state.animation.missionDistance > 0;
  state.debugStopAtSplice = isCheckpointReplan;  // Set true to stop at splice point

  // Debug: Log visited targets and MissionReplay state at animation start
  appendDebugLine(`🎬 Animation starting. Visited targets: [${state.visitedTargets.join(", ")}]`);
  const replayInfo = missionReplay.getDebugInfo();
  appendDebugLine(`🎬 MissionReplay: ${replayInfo.segmentCount} segments, current=${replayInfo.currentIndex}`);
  if (isCheckpointReplan) {
    appendDebugLine(`🛑 DEBUG: Will stop at splice point (prefix distance)`);
  }

  droneIds.forEach((did) => {
    const routeInfo = state.routes[did];

    // Debug: Log why a drone might be skipped
    if (!routeInfo) {
      appendDebugLine(`⚠️ D${did} skipped: no routeInfo`);
      return;
    }

    // Debug: Log trajectory info at animation start
    const trajLen = routeInfo.trajectory ? routeInfo.trajectory.length : 0;
    const routeStr = (routeInfo.route || []).join("-");
    const firstPt = trajLen > 0 ? `(${routeInfo.trajectory[0][0].toFixed(1)},${routeInfo.trajectory[0][1].toFixed(1)})` : "N/A";
    const lastPt = trajLen > 0 ? `(${routeInfo.trajectory[trajLen-1][0].toFixed(1)},${routeInfo.trajectory[trajLen-1][1].toFixed(1)})` : "N/A";
    appendDebugLine(`🎬 D${did}: ${trajLen} pts, route: ${routeStr}, from ${firstPt} to ${lastPt}`);
    if (!routeInfo.route) {
      appendDebugLine(`⚠️ D${did} skipped: no route array`);
      return;
    }
    if (routeInfo.route.length < 2) {
      appendDebugLine(`⚠️ D${did} skipped: route too short (${routeInfo.route.length} waypoints)`);
      return;
    }

    // Pre-calculate cumulative distances for uniform speed animation
    const trajectory = routeInfo.trajectory || [];
    let cumulativeDistances = [0];
    let totalDistance = 0;

    if (trajectory.length >= 2) {
      for (let i = 1; i < trajectory.length; i++) {
        const dx = trajectory[i][0] - trajectory[i - 1][0];
        const dy = trajectory[i][1] - trajectory[i - 1][1];
        totalDistance += Math.sqrt(dx * dx + dy * dy);
        cumulativeDistances.push(totalDistance);
      }
    }

    if (totalDistance <= 0) {
      appendDebugLine(`⚠️ D${did} skipped: totalDistance is ${totalDistance} (trajectory has ${trajectory.length} points)`);
      return;
    }

    // Use single missionDistance for all drones (already set from missionReplay.getStartingDistance())
    const initialDistance = state.animation.missionDistance;

    state.animation.drones[did] = {
      progress: totalDistance > 0 ? initialDistance / totalDistance : 0,
      distanceTraveled: initialDistance,
      animating: initialDistance < totalDistance,  // Only animate if there's more to travel
      cumulativeDistances,
      totalDistance,
    };

    // TEMP TEST: verify split utility
    const traj = (state.routes[did] && state.routes[did].trajectory) ? state.routes[did].trajectory : [];
    if (traj.length >= 2) {
      const total = state.animation.drones[did].totalDistance;
      const checkpointDist = 0.5 * total;

      const res = split_polyline_at_distance(traj, checkpointDist);
    }

    // Make sure trajectory is visible for animated drones
    state.trajectoryVisible[did] = true;
  });

  // Note: missionDistance is already set from missionReplay.getStartingDistance()
  // and will persist across the animation lifecycle

  updateTrajectoryButtonStates();
  updateAnimationButtonStates();

  // Check if any drones were added for animation
  const activeDrones = Object.keys(state.animation.drones);
  if (activeDrones.length === 0) {
    appendDebugLine("No valid drones to animate (all routes too short). Transitioning to CHECKPOINT for re-planning.");
    state.animation.active = false;

    // Transition to CHECKPOINT mode so user can edit or try again
    // This prevents getting stuck in READY_TO_ANIMATE with nothing to animate
    setMissionMode(MissionMode.CHECKPOINT, "no routes to animate - ready for edit/replan");
    alert("No targets could be reached with the remaining fuel. You can:\n• Edit the environment to move targets\n• Increase drone fuel budget\n• Remove blocking SAMs");
    return;
  }

  // Start animation loop
  const speedUnitsPerSec = 20; // tune this (map units per second)
  let lastTime = null;

  function animate(currentTime) {
    // Check if animation was stopped/paused externally
    if (!state.animation.active) {
      return;
    }

    if (lastTime === null) lastTime = currentTime;

    const deltaTime = Math.min(currentTime - lastTime, 100);
    lastTime = currentTime;

    const dt = deltaTime / 1000; // seconds
    let anyAnimating = false;

    // NEW: Only advance mission distance if at least one drone is still animating
    // Check FIRST if any drone can advance (has trajectory left to travel)
    const anyDroneCanAdvance = Object.values(state.animation.drones).some(ds =>
      ds.animating && ds.distanceTraveled < ds.totalDistance
    );

    if (anyDroneCanAdvance) {
      state.animation.missionDistance += speedUnitsPerSec * dt;
    }

    // LOST DRONES CHECK: Find the cut distance where each drone is lost and stop them there
    // This must happen BEFORE we sync distanceTraveled, so drones stop at their cut markers
    if (segmentedImport.isActive()) {
      const currentMissionDist = state.animation.missionDistance;

      Object.entries(state.animation.drones).forEach(([did, droneState]) => {
        if (!droneState.animating) return;
        if (droneState._lostAtCutDistance !== undefined) return; // Already handled

        // Check each segment to find where this drone is lost
        const segCount = missionReplay.getSegmentCount();
        for (let segIdx = 1; segIdx < segCount; segIdx++) {
          const seg = missionReplay.getSegment(segIdx);
          if (!seg || !seg.lostDrones) continue;

          // Is this drone lost at this segment?
          if (seg.lostDrones.includes(did)) {
            const cutDistance = seg.cutDistance;
            if (cutDistance && currentMissionDist >= cutDistance) {
              // Drone should stop at this cut distance
              droneState._lostAtCutDistance = cutDistance;
              droneState.distanceTraveled = cutDistance;
              droneState.animating = false;

              // TRUNCATE TRAJECTORY and get split point for lost drone position
              const routeInfo = state.routes[did];
              if (routeInfo && routeInfo.trajectory && routeInfo.trajectory.length > 0) {
                const fullTraj = routeInfo.trajectory;
                // split_polyline_at_distance returns { prefixPoints, suffixPoints, splitPoint, ... }
                const splitResult = split_polyline_at_distance(fullTraj, cutDistance);
                const truncated = splitResult?.prefixPoints;
                if (truncated && truncated.length > 0) {
                  routeInfo.trajectory = truncated;
                }
                // Save the cut position for drawing (so drone circle matches the marker)
                // Priority: seg.cutPositions if available, else use calculated split point
                if (seg.cutPositions && seg.cutPositions[did]) {
                  droneState._lostAtPosition = [...seg.cutPositions[did]];
                } else if (splitResult?.splitPoint) {
                  droneState._lostAtPosition = [...splitResult.splitPoint];
                } else if (truncated && truncated.length > 0) {
                  // Fallback: use end of truncated trajectory
                  droneState._lostAtPosition = [...truncated[truncated.length - 1]];
                }
              } else if (seg.cutPositions && seg.cutPositions[did]) {
                // No trajectory but have cutPositions
                droneState._lostAtPosition = [...seg.cutPositions[did]];
              }
              break;
            }
          }
        }
      });
    }

    Object.entries(state.animation.drones).forEach(([did, droneState]) => {
      if (!droneState.animating) return;

      // 1) Sync drone's distanceTraveled with mission distance
      droneState.distanceTraveled = state.animation.missionDistance;

      // DEBUG: Log distance periodically (every 50 units)
      const lastDistLog = droneState._lastDistLog || 0;
      if (droneState.distanceTraveled - lastDistLog > 50) {
        droneState._lastDistLog = droneState.distanceTraveled;
      }

      // 2) clamp and stop if THIS drone's trajectory is finished
      // (other drones may continue if their trajectories are longer)
      if (droneState.distanceTraveled >= droneState.totalDistance) {
        droneState.distanceTraveled = droneState.totalDistance;
        droneState.animating = false;
      } else {
        anyAnimating = true;
      }

      // 3) derive progress (for existing drawing/UI code)
      droneState.progress = (droneState.totalDistance > 0)
        ? (droneState.distanceTraveled / droneState.totalDistance)
        : 1;

      // 4) Check if drone's current position matches any unvisited target
      // Calculate drone position from trajectory using distance traveled
      const trajectory = state.routes[did]?.trajectory || [];
      const cumulativeDistances = droneState.cumulativeDistances || [];

      if (trajectory.length < 2 || cumulativeDistances.length !== trajectory.length) return;

      // Find current position based on distance traveled (same logic as drawDrone)
      const targetDistance = droneState.distanceTraveled;
      let segmentIdx = 0;
      for (let i = 1; i < cumulativeDistances.length; i++) {
        if (cumulativeDistances[i] >= targetDistance) {
          segmentIdx = i - 1;
          break;
        }
        segmentIdx = i - 1;
      }

      const fromPoint = trajectory[segmentIdx];
      const toPoint = trajectory[segmentIdx + 1] || fromPoint;
      const segmentStartDist = cumulativeDistances[segmentIdx];
      const segmentEndDist = cumulativeDistances[segmentIdx + 1] || segmentStartDist;
      const segmentLength = segmentEndDist - segmentStartDist;

      const segmentProgress = segmentLength > 0
        ? (targetDistance - segmentStartDist) / segmentLength
        : 0;

      const droneX = fromPoint[0] + (toPoint[0] - fromPoint[0]) * segmentProgress;
      const droneY = fromPoint[1] + (toPoint[1] - fromPoint[1]) * segmentProgress;

      const EPS = 0.5;  // Floating-point comparison tolerance

      // Get targets in this drone's route for the current segment (optimization)
      // This avoids checking all targets in the environment
      const currentSegIdx = missionReplay._currentSegmentIndex || 0;
      const currentSeg = missionReplay.getSegment(currentSegIdx);
      const routeTargetIds = new Set();

      // Collect target IDs from the current segment's route for this drone
      const segRoute = currentSeg?.solution?.routes?.[did]?.route || state.routes[did]?.route || [];
      for (const wp of segRoute) {
        if (wp && String(wp).startsWith('T')) {
          routeTargetIds.add(wp);
        }
      }

      // Check targets in this drone's route
      const allTargets = state.env.targets || [];
      for (const target of allTargets) {
        if (!target.id) continue;
        if (!routeTargetIds.has(target.id)) continue;  // Only check targets in route
        if (state.visitedTargets.includes(target.id)) continue;  // Already visited (stays checked until Reset)

        // Check if drone position equals target position (within epsilon)
        if (Math.abs(droneX - target.x) < EPS && Math.abs(droneY - target.y) < EPS) {
          state.visitedTargets.push(target.id);
          appendDebugLine(`🎯 D${did} visited ${target.id} at (${droneX.toFixed(1)}, ${droneY.toFixed(1)})`);
        }
      }
    });

    // MISSION REPLAY: Check segment boundary using single mission distance
    // This is simple: when missionDistance >= nextSegment.cutDistance, switch segments
    let switched = false;
    let newSegment = null;

    // Segment boundary check
    try {
      const nextSeg = missionReplay.getNextSegment();
      const nextCutDist = nextSeg?.getCutDistance?.() || nextSeg?.cutDistance || null;
      // DEBUG: Log segment check every 10 units of travel
      const lastSegCheckLog = state.animation._lastSegCheckLog || 0;
      if (state.animation.missionDistance - lastSegCheckLog > 10) {
        appendDebugLine(`[SEG CHECK] missionDist=${state.animation.missionDistance.toFixed(1)}, nextCutDist=${nextCutDist?.toFixed(1) || 'null'}, currentSeg=${missionReplay._currentSegmentIndex}, anyAnimating=${anyAnimating}`);
        state.animation._lastSegCheckLog = state.animation.missionDistance;
      }

      const result = missionReplay.checkSegmentBoundary(state.animation.missionDistance);
      switched = result.switched;
      newSegment = result.newSegment;
    } catch (err) {
      console.error('[SEG CHECK ERROR]', err);
      appendDebugLine(`[SEG CHECK ERROR] ${err.message}`);
    }

    if (switched && newSegment) {
      appendDebugLine(`🔄 === SEGMENT SWITCH to ${newSegment.index} (via MissionReplay) ===`);
      // DEBUG: Print segment table at switch to diagnose cutDistance issues
      appendDebugLine(missionReplay.printSegmentTable());

      // Also sync missionState for backward compatibility
      missionState.currentSegmentIndex = newSegment.index;

      // SEGMENTED IMPORT: Update environment and targets using SegmentedImportManager
      if (segmentedImport.isActive()) {
        // Sync SegmentedImportManager's current segment index
        segmentedImport._currentSegmentIndex = newSegment.index;

        // Get environment for display (cumulative targets from segments 0 to current)
        state.env = segmentedImport.getEnvForDisplay();

        // Load drone_configs from the SEGMENT (the source of truth)
        if (newSegment.drone_configs) {
          state.droneConfigs = JSON.parse(JSON.stringify(newSegment.drone_configs));
          state.env.drone_configs = state.droneConfigs;
          refreshDroneConfigUI();  // Update the UI table
        }

      } else {
        // LEGACY: Progressive target reveal for non-segmented imports
        const newSegmentRouteTargets = new Set();
        Object.values(newSegment.solution?.routes || {}).forEach(routeData => {
          const route = routeData.route || [];
          route.forEach(stop => {
            if (stop && stop.startsWith('T')) {
              newSegmentRouteTargets.add(stop);
            }
          });
        });

        if (newSegmentRouteTargets.size > 0 && state.initialEnvSnapshot?.targets) {
          // Add new segment's route targets to state.env.targets (if not already present)
          const currentTargetIds = new Set((state.env.targets || []).map(t => t.id));
          const targetsToAdd = state.initialEnvSnapshot.targets.filter(
            t => newSegmentRouteTargets.has(t.id) && !currentTargetIds.has(t.id)
          );
          if (targetsToAdd.length > 0) {
            state.env.targets = [...(state.env.targets || []), ...targetsToAdd];
          }
        } else if (!state.initialEnvSnapshot) {
          // Fallback: use segment env if no snapshot available
          state.env = JSON.parse(JSON.stringify(newSegment.env));
        }

        // Load drone_configs from the SEGMENT (the source of truth)
        if (newSegment.drone_configs) {
          state.droneConfigs = JSON.parse(JSON.stringify(newSegment.drone_configs));
          state.env.drone_configs = state.droneConfigs;
          refreshDroneConfigUI();  // Update the UI table
        }
      }

      // SEGMENTED IMPORT: Extend trajectories for ACTIVE drones only
      // Check lostDrones to identify which drones should stop animating
      if (segmentedImport.isActive()) {
        // Collect all lost drones from previous segments
        // Try multiple sources: segmentedImport, missionReplay, newSegment
        const allLostDrones = new Set();
        for (let i = 1; i <= newSegment.index; i++) {
          // First try segmentedImport (SegmentManager)
          const segFromImport = segmentedImport.getSegment?.(i);
          if (segFromImport?.lostDrones) {
            segFromImport.lostDrones.forEach(did => allLostDrones.add(did));
          }
          // Fallback to missionReplay
          const segFromReplay = missionReplay.getSegment(i);
          if (segFromReplay?.lostDrones) {
            segFromReplay.lostDrones.forEach(did => allLostDrones.add(did));
          }
        }

        // CRITICAL FIX: Apply lost drone state to state.droneConfigs and refresh UI
        // The segment's drone_configs should already have this, but enforce it here too
        if (allLostDrones.size > 0) {
          allLostDrones.forEach(did => {
            if (state.droneConfigs?.[did]) {
              state.droneConfigs[did].enabled = false;
            }
          });
          refreshDroneConfigUI();  // Update the UI table with lost drones
        }

        // Build combined trajectories from segment 0 to current segment
        // This is the SAME as creation workflow - progressive reveal
        appendDebugLine(`🔄 Segment switch: Building combined routes for seg 0-${newSegment.index}...`);
        const combinedRoutes = buildCombinedRoutesFromSegments(newSegment.index);

        // Log trajectory changes with distances
        const oldLens = Object.entries(state.routes).map(([did, rd]) => `D${did}:${rd.trajectory?.length || 0}`).join(', ');
        const newInfo = Object.entries(combinedRoutes).map(([did, rd]) => {
          const traj = rd.trajectory || [];
          let dist = 0;
          for (let j = 1; j < traj.length; j++) {
            const dx = traj[j][0] - traj[j-1][0];
            const dy = traj[j][1] - traj[j-1][1];
            dist += Math.sqrt(dx*dx + dy*dy);
          }
          return `D${did}:${traj.length}pts/${dist.toFixed(1)}dist`;
        }).join(', ');
        appendDebugLine(`  Old trajectories: [${oldLens}]`);
        appendDebugLine(`  New trajectories: [${newInfo}]`);

        Object.entries(combinedRoutes).forEach(([did, routeData]) => {
          const isLostDrone = allLostDrones.has(did);
          const trajectory = routeData.trajectory || [];

          if (trajectory.length > 0) {
            // Ensure state.routes[did] exists
            if (!state.routes[did]) {
              state.routes[did] = {};
            }
            state.routes[did].trajectory = JSON.parse(JSON.stringify(trajectory));
            state.routes[did].route = JSON.parse(JSON.stringify(routeData.route || []));

            // Update cumulative distances for the combined trajectory
            const droneState = state.animation.drones[did];
            if (droneState) {
              let cumulativeDistances = [0];
              let totalDistance = 0;
              for (let i = 1; i < trajectory.length; i++) {
                const dx = trajectory[i][0] - trajectory[i - 1][0];
                const dy = trajectory[i][1] - trajectory[i - 1][1];
                totalDistance += Math.sqrt(dx * dx + dy * dy);
                cumulativeDistances.push(totalDistance);
              }
              droneState.cumulativeDistances = cumulativeDistances;
              droneState.totalDistance = totalDistance;
              // Keep distanceTraveled as-is (drone continues from checkpoint)

              appendDebugLine(`    D${did}: distTraveled=${droneState.distanceTraveled.toFixed(1)}, newTotalDist=${totalDistance.toFixed(1)}, trajLen=${trajectory.length}, isLost=${isLostDrone}`);

              // Lost drones: keep stopped
              if (isLostDrone) {
                droneState.animating = false;
                state.trajectoryVisible[did] = true;
                appendDebugLine(`    D${did}: LOST - stopped`);
              } else if (droneState.distanceTraveled < totalDistance) {
                // Active drone with more to fly: re-enable animation
                droneState.animating = true;
                appendDebugLine(`    D${did}: Continuing animation`);
              } else {
                droneState.animating = false;
                state.trajectoryVisible[did] = true;
                appendDebugLine(`    D${did}: distTraveled >= totalDist - stopped`);
              }
            }
          }
        });
      } else {
        // LEGACY/CHECKPOINT: Build combined trajectories up to current segment only
        // This ensures progressive reveal - only show trajectories for segments reached so far

        // STEP 1: Apply lost drones FIRST (before applying routes)
        // Collect all lost drones from segments up to and including newSegment
        const allLostDrones = new Set();
        for (let i = 0; i <= newSegment.index; i++) {
          const seg = missionReplay.getSegment(i);
          if (seg?.lostDrones) {
            seg.lostDrones.forEach(did => allLostDrones.add(String(did)));
          }
        }
        // Apply lost drone state
        allLostDrones.forEach(did => {
          // Stop animation
          if (state.animation.drones[did]) {
            state.animation.drones[did].animating = false;
          }
          // Disable in config
          if (state.droneConfigs?.[did]) {
            state.droneConfigs[did].enabled = false;
          }
          // Hide trajectory (or keep visible but grayed - your choice)
          state.trajectoryVisible[did] = true; // Keep visible to show where it stopped
          appendDebugLine(`🔴 D${did}: Lost drone disabled at segment switch`);
        });

        // STEP 2: Update visited_targets from stored segment data (deterministic)
        // Union of visited_targets from all segments played so far
        const storedVisited = new Set();
        for (let i = 0; i <= newSegment.index; i++) {
          const seg = missionReplay.getSegment(i);
          if (seg?.visited_targets) {
            seg.visited_targets.forEach(t => storedVisited.add(t));
          }
        }
        if (storedVisited.size > 0) {
          // Merge with existing (in case some were marked during animation)
          storedVisited.forEach(t => {
            if (!state.visitedTargets.includes(t)) {
              state.visitedTargets.push(t);
            }
          });
          appendDebugLine(`🎯 Visited targets from stored data: ${[...storedVisited].join(',')}`);
        }

        // STEP 3: Build combined routes (frozen + new segment)
        const combinedRoutes = buildCombinedRoutesFromSegments(newSegment.index);

        Object.entries(combinedRoutes).forEach(([did, routeData]) => {
          const isLostDrone = allLostDrones.has(did);
          const trajectory = routeData.trajectory || [];
          if (trajectory.length > 0) {
            // Ensure state.routes[did] exists (critical for lost drones that may have been cleared)
            if (!state.routes[did]) {
              state.routes[did] = {};
            }
            state.routes[did].trajectory = JSON.parse(JSON.stringify(trajectory));
            state.routes[did].route = JSON.parse(JSON.stringify(routeData.route || []));

            // Update drone's cumulative distances and total distance for the new trajectory
            const droneState = state.animation.drones[did];
            if (droneState) {
              let cumulativeDistances = [0];
              let totalDistance = 0;
              for (let i = 1; i < trajectory.length; i++) {
                const dx = trajectory[i][0] - trajectory[i - 1][0];
                const dy = trajectory[i][1] - trajectory[i - 1][1];
                totalDistance += Math.sqrt(dx * dx + dy * dy);
                cumulativeDistances.push(totalDistance);
              }
              droneState.cumulativeDistances = cumulativeDistances;
              droneState.totalDistance = totalDistance;
              // Keep distanceTraveled as-is (we're continuing from checkpoint)

              // Lost drones: keep stopped, don't re-enable animation
              if (isLostDrone) {
                droneState.animating = false;
                state.trajectoryVisible[did] = true;
              } else if (droneState.distanceTraveled < totalDistance) {
                // Active drone with more to fly: re-enable animation
                droneState.animating = true;
              } else {
                // Drone is frozen/completed - ensure it stays stopped but visible
                droneState.animating = false;
                state.trajectoryVisible[did] = true;
              }
            }
          }
        });
      }

      appendDebugLine(`🔄 === SEGMENT SWITCH COMPLETE ===`);

      // Clear debug flags so targets get re-evaluated with new trajectory
      Object.entries(state.animation.drones).forEach(([did, ds]) => {
        // Clear _debugDist_ flags so we log distances for targets in new segment
        Object.keys(ds).filter(k => k.startsWith('_debug')).forEach(k => delete ds[k]);
      });

      // IMPORTANT: After segment switch, immediately mark any targets that should
      // have been passed based on current distanceTraveled. This catches targets
      // whose cumulative distance is <= distanceTraveled but weren't marked due
      // to the segment boundary.
      //
      // CRITICAL FIX: Only check targets from PREVIOUS segments (already in initialEnvSnapshot),
      // NOT newly added targets from the current segment. Newly added targets should only be
      // marked as the drone actually reaches them during continued animation.
      Object.entries(state.animation.drones).forEach(([did, droneState]) => {
        const routeInfo = state.routes[did];
        if (!routeInfo || !routeInfo.route || !routeInfo.trajectory) return;

        const trajectory = routeInfo.trajectory;
        const route = routeInfo.route;
        const cumulativeDistances = droneState.cumulativeDistances || [];
        const currentDistance = droneState.distanceTraveled;

        // Build set of targets that were in THIS drone's route in PREVIOUS segments
        // AND are still in this drone's route in the CURRENT segment
        // This prevents marking targets that were:
        // 1. Newly added to this drone (moved from another drone)
        // 2. Removed from this drone (reassigned to another drone)
        const prevSegTargetsForDrone = new Set();
        for (let segIdx = 0; segIdx < newSegment.index; segIdx++) {
          const prevSeg = missionReplay.getSegment(segIdx);
          const prevRoute = prevSeg?.solution?.routes?.[did]?.route || [];
          prevRoute.forEach(wp => {
            if (String(wp).startsWith("T")) {
              prevSegTargetsForDrone.add(wp);
            }
          });
        }

        // Get targets in the CURRENT segment's route for this drone
        const currentSegTargetsForDrone = new Set();
        const currentSegRoute = newSegment?.solution?.routes?.[did]?.route || [];
        currentSegRoute.forEach(wp => {
          if (String(wp).startsWith("T")) {
            currentSegTargetsForDrone.add(wp);
          }
        });

        route.forEach((wp, wpIdx) => {
          if (!String(wp).startsWith("T")) return;
          if (state.visitedTargets.includes(wp)) return;

          // CHECK 1: Target must be in the CURRENT segment's route for this drone
          // If a target was removed from this drone's route (reassigned to another drone),
          // this drone should NOT mark it - even if this drone's old trajectory passed near it
          if (!currentSegTargetsForDrone.has(wp)) {
            return; // Skip - target no longer assigned to this drone
          }

          // CHECK 2: Only mark targets that were in previous segments for this drone
          // If a target was just added to this drone's route (e.g., moved from D1 to D2),
          // it should NOT be marked at segment switch - only when drone actually reaches it
          if (!prevSegTargetsForDrone.has(wp)) {
            return; // Skip - this target is newly assigned to this drone
          }

          // Sequential order check: only mark if all previous targets visited
          // This prevents newly added targets from being marked prematurely
          let canMark = true;
          for (let i = 0; i < wpIdx; i++) {
            const prevWp = route[i];
            if (String(prevWp).startsWith("T") && !state.visitedTargets.includes(prevWp)) {
              canMark = false;
              break;
            }
          }
          if (!canMark) return;

          // Find target position - check current env first, then all segments
          let target = state.env.targets?.find(t => t.id === wp);
          if (!target) {
            for (let segIdx = 0; segIdx < missionReplay.getSegmentCount(); segIdx++) {
              const seg = missionReplay.getSegment(segIdx);
              const segTarget = seg?.env?.targets?.find(t => t.id === wp);
              if (segTarget) {
                target = segTarget;
                break;
              }
            }
          }
          if (!target) return;

          // Find closest trajectory point to this target
          let minDist = Infinity;
          let closestIdx = -1;
          trajectory.forEach((pt, idx) => {
            const dist = Math.sqrt(Math.pow(pt[0] - target.x, 2) + Math.pow(pt[1] - target.y, 2));
            if (dist < minDist) {
              minDist = dist;
              closestIdx = idx;
            }
          });

          // If trajectory passes close enough and we've already traveled past it, mark it
          // Use threshold of 5.0 - must be on actual path, not just near cut point
          if (closestIdx >= 0 && minDist < 5.0 && cumulativeDistances[closestIdx] !== undefined) {
            const targetDistance = cumulativeDistances[closestIdx];
            if (currentDistance >= targetDistance) {
              state.visitedTargets.push(wp);
              appendDebugLine(`🎯 D${did} visited ${wp} (at segment switch)`);
            }
          }
        });
      });

      // AUTO-CONTINUE through checkpoints with 1 second pause
      drawEnvironment(true);  // fromAnimationLoop = true
      updateStatsFromAnimation();

      // Pause animation for 1 second, then auto-resume
      state.animation.active = false;
      setTimeout(() => {
        if (missionState.mode === MissionMode.ANIMATING ||
            missionState.mode === MissionMode.READY_TO_ANIMATE) {
          state.animation.active = true;
          state.animation.animationId = requestAnimationFrame(animate);
        }
      }, 1000);
      return;  // Exit current frame, setTimeout will resume
    }

    drawEnvironment(true);  // fromAnimationLoop = true

    // Update stats with real-time distance traveled
    updateStatsFromAnimation();

    // Check if there are more segments to switch to (pending checkpoints)
    const hasMoreSegments = missionReplay.getNextSegment() !== null;

    // DEBUG: Log when all drones stop animating
    if (!anyAnimating && state.animation._wasAnimating) {
      appendDebugLine(`[ANIM END] All drones stopped. hasMoreSegments=${hasMoreSegments}, missionDist=${state.animation.missionDistance.toFixed(1)}`);
    }
    state.animation._wasAnimating = anyAnimating;

    if (anyAnimating && state.animation.active) {
      state.animation.animationId = requestAnimationFrame(animate);
    } else if (!state.animation.active) {
      // Stopped externally (pause/cut) - don't change mode
      appendDebugLine(`[ANIM] Stopped externally`);
      return;
    } else if (hasMoreSegments) {
      // No drones animating, but there are more segments pending
      // This can happen if a drone finishes early but other drones need segment switches
      // Keep the animation loop running to process segment switches
      state.animation.animationId = requestAnimationFrame(animate);
    } else {
      appendDebugLine(`[ANIM] Completing - no more segments`);
      state.animation.active = false;
      // Animation completed naturally - go to READY_TO_ANIMATE
      setMissionMode(MissionMode.READY_TO_ANIMATE, "animation complete");
      updateAnimationButtonStates();
      // Final stats update when animation completes
      updateStatsFromAnimation();
    }
  }

  state.animation.animationId = requestAnimationFrame(animate);

  // Transition to ANIMATING state
  setMissionMode(MissionMode.ANIMATING, "animation started");

  appendDebugLine(`Started animation for: ${droneIds.map((d) => `D${d}`).join(", ")}`);
}

function stopAnimation() {
  const wasAnimating = state.animation.active;

  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }

  state.animation.active = false;
  // DON'T clear drones - keep their current positions visible
  // state.animation.drones = {};  // Removed - drones should stay visible
  missionState.pauseContext = null;

  // Mark all animating drones as stopped (not animating)
  Object.values(state.animation.drones).forEach(droneState => {
    droneState.animating = false;
  });

  // If we were animating, transition back to READY_TO_ANIMATE
  if (wasAnimating && missionState.mode === MissionMode.ANIMATING) {
    setMissionMode(MissionMode.READY_TO_ANIMATE, "animation stopped");
  }

  updateAnimationButtonStates();
  drawEnvironment();
  appendDebugLine("Animation stopped. Drones remain at current positions.");
}

/**
 * Pause animation - save state for resuming
 */
function pauseAnimation() {
  const perms = getUiPermissions();
  if (!perms.canPause) {
    appendDebugLine("Cannot pause in current state");
    return;
  }

  // Save the current animation state for resume
  missionState.pauseContext = {
    droneStates: JSON.parse(JSON.stringify(state.animation.drones)),
    animatingDrones: Object.keys(state.animation.drones).filter(
      did => state.animation.drones[did].animating
    ),
  };

  // Stop the animation frame loop but keep drone positions
  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }
  state.animation.active = false;

  // Transition to PAUSED state
  setMissionMode(MissionMode.PAUSED_MID_ANIMATION, "animation paused");

  updateAnimationButtonStates();
  drawEnvironment();
  appendDebugLine("Animation paused. Click Animate to resume.");
}

/**
 * Resume animation from pause state
 * Uses full animation loop with target marking (same as startAnimation)
 */
function resumeAnimation() {
  if (!missionState.pauseContext) {
    appendDebugLine("No pause context to resume from");
    return;
  }

  // Restore animation state
  state.animation.drones = missionState.pauseContext.droneStates;
  state.animation.active = true;

  // Re-enable animating flag for drones that were animating
  missionState.pauseContext.animatingDrones.forEach(did => {
    if (state.animation.drones[did]) {
      state.animation.drones[did].animating = true;
    }
  });

  // Clear pause context
  missionState.pauseContext = null;

  // Restart animation loop with FULL target marking logic (same as startAnimation)
  const speedUnitsPerSec = 20;
  let lastTime = null;

  function animate(currentTime) {
    if (!state.animation.active) return;

    if (lastTime === null) lastTime = currentTime;

    const deltaTime = Math.min(currentTime - lastTime, 100);
    lastTime = currentTime;

    const dt = deltaTime / 1000;
    let anyAnimating = false;

    Object.entries(state.animation.drones).forEach(([did, droneState]) => {
      if (!droneState.animating) return;

      droneState.distanceTraveled += speedUnitsPerSec * dt;

      if (droneState.distanceTraveled >= droneState.totalDistance) {
        droneState.distanceTraveled = droneState.totalDistance;
        droneState.animating = false;
      } else {
        anyAnimating = true;
      }

      droneState.progress = (droneState.totalDistance > 0)
        ? (droneState.distanceTraveled / droneState.totalDistance)
        : 1;

      // TARGET MARKING: Check if drone has passed any target waypoints
      const routeInfo = state.routes[did];
      if (routeInfo && routeInfo.route && routeInfo.trajectory) {
        const trajectory = routeInfo.trajectory;
        const route = routeInfo.route;
        const cumulativeDistances = droneState.cumulativeDistances || [];
        const currentDistance = droneState.distanceTraveled;

        route.forEach((wp) => {
          if (!String(wp).startsWith("T")) return;
          if (state.visitedTargets.includes(wp)) return;

          // Find target position - check current env first, then all segments
          let target = state.env.targets?.find(t => t.id === wp);
          if (!target) {
            for (let segIdx = 0; segIdx < missionReplay.getSegmentCount(); segIdx++) {
              const seg = missionReplay.getSegment(segIdx);
              const segTarget = seg?.env?.targets?.find(t => t.id === wp);
              if (segTarget) {
                target = segTarget;
                break;
              }
            }
          }
          if (!target) return;

          // Find closest trajectory point to this target
          let minDist = Infinity;
          let closestIdx = -1;
          trajectory.forEach((pt, idx) => {
            const dist = Math.sqrt(Math.pow(pt[0] - target.x, 2) + Math.pow(pt[1] - target.y, 2));
            if (dist < minDist) {
              minDist = dist;
              closestIdx = idx;
            }
          });

          // If trajectory passes close enough and we've traveled past it, mark visited
          if (closestIdx >= 0 && minDist < 20.0 && cumulativeDistances[closestIdx] !== undefined) {
            const targetDistance = cumulativeDistances[closestIdx];
            if (currentDistance >= targetDistance) {
              state.visitedTargets.push(wp);
              appendDebugLine(`🎯 D${did} visited ${wp}`);
            }
          }
        });
      }
    });

    drawEnvironment(true);  // fromAnimationLoop = true
    updateStatsFromAnimation();

    if (anyAnimating && state.animation.active) {
      state.animation.animationId = requestAnimationFrame(animate);
    } else if (!state.animation.active) {
      return;  // Stopped externally
    } else {
      state.animation.active = false;
      setMissionMode(MissionMode.READY_TO_ANIMATE, "animation complete");
      updateAnimationButtonStates();
      updateStatsFromAnimation();
    }
  }

  state.animation.animationId = requestAnimationFrame(animate);

  // Transition to ANIMATING state
  setMissionMode(MissionMode.ANIMATING, "animation resumed");

  updateAnimationButtonStates();
  appendDebugLine("Animation resumed with target marking.");
}

/**
 * Cut/Checkpoint - freeze current position and truncate future
 */
function cutAtCheckpoint() {
  const perms = getUiPermissions();
  if (!perms.canCut) {
    appendDebugLine("Cannot cut in current state");
    return;
  }

  appendDebugLine(`cutAtCheckpoint called. animation.drones keys: ${Object.keys(state.animation.drones || {}).join(", ") || "NONE"}`);
  appendDebugLine(`animation.active: ${state.animation.active}`);

  // Use the existing freezeAtCheckpoint logic
  freezeAtCheckpoint();

  // Verify checkpoint was set up
  if (!state.checkpoint?.active) {
    appendDebugLine("⚠️ WARNING: freezeAtCheckpoint did not set checkpoint.active! Check drone data.");
  } else {
    appendDebugLine(`✅ Checkpoint set with ${Object.keys(state.checkpoint.segments || {}).length} drone segments`);
  }

  // Debug: show splitPoints for all drones at cut time
  appendDebugLine(
    `CUT DEBUG: checkpoint splitPoints = ` +
    Object.entries(state.checkpoint?.segments || {})
      .map(([did, s]) => `D${did}:${s?.splitPoint ? `[${s.splitPoint.map(v=>v.toFixed(1)).join(",")}]` : "null"}`)
      .join(" ")
  );

  // Note: pendingCutPositions is now populated inside freezeAtCheckpoint()
  // Log what was captured for debugging
  if (state.pendingCutPositions && Object.keys(state.pendingCutPositions).length > 0) {
    for (const [did, pos] of Object.entries(state.pendingCutPositions)) {
      appendDebugLine(`📍 Cut pos D${did} at [${pos[0].toFixed(2)}, ${pos[1].toFixed(2)}]`);
    }
    // Set legacy single cut position from first drone (backward compatibility)
    const firstDid = Object.keys(state.pendingCutPositions)[0];
    if (firstDid) {
      const p = state.pendingCutPositions[firstDid];
      state.pendingCutPosition = [p[0], p[1]];
    }
    appendDebugLine(`📍 Cut positions captured for drones: [${Object.keys(state.pendingCutPositions).join(", ")}]`);
  } else {
    appendDebugLine("⚠️ No cut positions captured - pendingCutPositions is empty");
  }

  // Transition to CHECKPOINT state - allow solving immediately
  // (Bug fix: don't set checkpointSource = "replay_cut" which blocks solving)
  missionState.checkpointSource = null;
  setMissionMode(MissionMode.CHECKPOINT, "cut at checkpoint");

  // Assign segment numbers to newly visited targets
  const newlyAssignedTargets = [];
  for (const targetId of state.visitedTargets) {
    // Only assign if not already assigned in a previous cut
    if (!state.previouslyAssignedTargets.includes(targetId)) {
      state.targetSegmentMap[targetId] = state.currentCutSegment;
      newlyAssignedTargets.push(targetId);
    }
  }

  // Log segment assignment
  if (newlyAssignedTargets.length > 0) {
    appendDebugLine(`🏷️ Segment ${state.currentCutSegment}: Assigned targets [${newlyAssignedTargets.join(", ")}]`);
  }

  // Update previously assigned targets and increment segment counter for next cut
  state.previouslyAssignedTargets = [...state.visitedTargets];
  state.currentCutSegment++;

  // Always redraw to show visited targets with green X and segment badges
  drawEnvironment();

  // Debug: Log visited targets
  appendDebugLine(`📍 Visited targets after cut: [${state.visitedTargets.join(", ")}]`);
  appendDebugLine(`Cut at checkpoint (Segment ${state.currentCutSegment - 1} complete). Edit environment and Run Planner to continue.`);
}

function updateAnimationButtonStates() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`anim-d${did}`);
    if (btn) {
      const droneState = state.animation.drones[String(did)];
      if (droneState && droneState.animating) {
        btn.classList.add("active");
      } else {
        btn.classList.remove("active");
      }
    }
  }

  const allBtn = $("anim-all");
  if (allBtn) {
    if (state.animation.active && Object.keys(state.animation.drones).length === 5) {
      allBtn.classList.add("active");
    } else {
      allBtn.classList.remove("active");
    }
  }
}

function attachAnimationControls() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`anim-d${did}`);
    if (btn) {
      btn.addEventListener("click", () => {
        startAnimation([String(did)]);
      });
    }
  }

  const allBtn = $("anim-all");
  if (allBtn) {
    allBtn.addEventListener("click", () => {
      // Start animation for all drones that have a valid trajectory
      const animDrones = Object.keys(state.routes || {})
        .map(String)
        .filter(did => Array.isArray(state.routes[did]?.trajectory) && state.routes[did].trajectory.length >= 2);

      appendDebugLine(`Anim All: animDrones=[${animDrones.join(",")}]`);

      if (animDrones.length > 0) {
        startAnimation(animDrones);
      } else {
        appendDebugLine("No routes to animate. Run planner first.");
      }
    });
  }

  const stopBtn = $("anim-stop");
  if (stopBtn) {
    stopBtn.addEventListener("click", stopAnimation);
  }
}
// ----------------------------------------------------
// Replan from checkpoint
// ----------------------------------------------------
async function replanFromCheckpoint() {
  if (!state.missionId) {
    appendDebugLine("No missionId. Run planner first.");
    return;
  }
  if (!state.checkpoint?.active) {
    appendDebugLine("No active checkpoint. Press C to freeze first.");
    return;
  }

  // Build env2: copy current env, then (for now) only change drone starts to splitPoint.
  // (We’ll exclude visited targets next; this gets the pipeline working end-to-end.)
  const env2 = JSON.parse(JSON.stringify(state.env));

  // Add synthetic start nodes per drone based on checkpoint splitPoint
  // and update drone configs to start from those nodes.
  env2.synthetic_starts = env2.synthetic_starts || {};
  const newDroneConfigs = JSON.parse(JSON.stringify(state.droneConfigs));

  Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
    if (!seg?.splitPoint) return;

    const nodeId = `D${did}_START`;
    env2.synthetic_starts[nodeId] = { x: seg.splitPoint[0], y: seg.splitPoint[1] };

    // Update drone config to start at synthetic node
    if (newDroneConfigs[did]) {
      newDroneConfigs[did].start_airport = nodeId; // naming consistent with existing schema
    }
  });

  appendDebugLine("Requesting replanned suffix from checkpoint...");

  // Call backend (new endpoint)
  const resp = await fetch("/api/mission/replan_checkpoint", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      mission_id: state.missionId,
      checkpoint_pct: state.checkpoint.pct,
      env2,
      drone_configs: newDroneConfigs,
    }),
  });

  if (!resp.ok) {
    const text = await resp.text();
    appendDebugLine("Replan failed. Check browser developer tools.");
    return;
  }

  const data = await resp.json();

  // Expected: data.routes like your normal solver response:
  // { "1": { trajectory: [[x,y],...], route:[...]} , ... }
  if (!data.routes) {
    appendDebugLine("Replan response invalid (missing routes).");
    return;
  }

  // Splice: finalTrajectory = prefix + newSuffixTrajectory (skipping duplicate split point)
  Object.entries(data.routes).forEach(([did, r]) => {
    const seg = state.checkpoint.segments[did];
    if (!seg?.prefix || !r?.trajectory || r.trajectory.length < 1) return;

    const prefix = seg.prefix;
    const newSuffix = r.trajectory;

    // Avoid duplicating the join point if the suffix starts at the split
    const joined =
      (prefix.length > 0 &&
       newSuffix.length > 0 &&
       prefix[prefix.length - 1][0] === newSuffix[0][0] &&
       prefix[prefix.length - 1][1] === newSuffix[0][1])
        ? prefix.concat(newSuffix.slice(1))
        : prefix.concat(newSuffix);

    // Install joined route back into UI state
    state.routes[did].trajectory = joined;

    // If you also display route labels, keep the new route (or splice labels later)
    if (r.route) state.routes[did].route = r.route;
  });

  // After replanning, clear checkpoint active (optional) and redraw
  state.checkpoint.active = false;

  drawEnvironment();
  appendDebugLine("Replan complete: suffix replaced and spliced.");
}

// ----------------------------------------------------
// Boot
// ----------------------------------------------------
window.addEventListener("load", () => {
  resizeCanvasToContainer();
  window.addEventListener("resize", resizeCanvasToContainer);

  attachCanvasEditing();
  attachConfigListeners();
  attachSequenceBar();
  attachIOHandlers();
  attachOptimizationHandlers();
  attachEditToggle();
  attachTabSwitching();
  attachTrajectoryControls();
  attachAnimationControls();

  const btnRun = $("btn-run-planner");
  if (btnRun) btnRun.addEventListener("click", runPlanner);

  const btnCancel = $("btn-cancel-solve");
  if (btnCancel) btnCancel.addEventListener("click", cancelSolve);

  // Add buttons (respect editMode)
  const btnAddA = $("btn-add-airport");
  const btnAddT = $("btn-add-target");
  const btnAddS = $("btn-add-sam");
  const btnDel  = $("btn-delete-selected");

  if (btnAddA) {
    btnAddA.addEventListener("click", () => {
      if (!state.editMode) return;
      setAddMode("airport");
    });
  }
  if (btnAddT) {
    btnAddT.addEventListener("click", () => {
      if (!state.editMode) return;
      setAddMode("target");
    });
  }
  if (btnAddS) {
    btnAddS.addEventListener("click", () => {
      if (!state.editMode) return;
      setAddMode("sam");
    });
  }
  if (btnDel) {
    btnDel.addEventListener("click", () => {
      if (!state.editMode) return;
      deleteSelected();
    });
  }

  // Status banner action buttons
  const btnAcceptEdits = $("btn-accept-edits");
  if (btnAcceptEdits) {
    btnAcceptEdits.addEventListener("click", acceptEdits);
  }

  const btnAcceptSolution = $("btn-accept-solution");
  if (btnAcceptSolution) {
    btnAcceptSolution.addEventListener("click", acceptSolution);
  }

  const btnDiscardDraft = $("btn-discard-draft");
  if (btnDiscardDraft) {
    btnDiscardDraft.addEventListener("click", discardDraftSolution);
  }

  // Mission Control Panel handlers
  attachMissionControlHandlers();

  // Agent Send button
  attachAgentHandlers();

  // Initialize status banner
  updateStatusBanner();
  updateButtonStates();

  initialLoadEnv();

  // Initialize Mission Control state
  updateMissionControlState();
});

// ----------------------------------------------------
// Mission Control Panel
// ----------------------------------------------------
function attachMissionControlHandlers() {
  const mcRunPlanner = $("mc-run-planner");
  const mcDiscard = $("mc-discard");
  const mcEdit = $("mc-edit");
  const mcDisplay = $("mc-display");
  const mcAnimate = $("mc-animate");
  const mcCut = $("mc-cut");
  const mcReset = $("mc-reset");

  // Set up segment switch callback to update UI when segment changes during animation
  missionReplay.onSegmentSwitch((fromIndex, toIndex, newSegment) => {
    updateMissionControlState();
  });

  // Run Planner / Accept toggle button
  if (mcRunPlanner) {
    mcRunPlanner.addEventListener("click", () => {
      try {
        const perms = getUiPermissions();
        if (perms.canAcceptSolution) {
          // In Accept mode - accept the solution
          acceptSolution();
        } else if (perms.canSolve) {
          // In Run Planner mode - run the planner
          runPlanner();
        } else if (missionState.mode === MissionMode.DRAFT_READY && missionState.draftSolution) {
          // Fallback: If we're in DRAFT_READY mode with a draft solution, accept anyway
          // This handles any edge case where perms isn't reflecting the correct state
          acceptSolution();
        } else {
        }
      } catch (err) {
        appendDebugLine(`Accept button error: ${err.message}`);
      }
    });
  } else {
  }

  // Display button - toggle all trajectories
  if (mcDisplay) {
    mcDisplay.addEventListener("click", () => {
      toggleAllTrajectories();
    });
  }

  // Edit button - toggles edit mode (Edit ↔ Done)
  if (mcEdit) {
    mcEdit.addEventListener("click", () => {
      const perms = getUiPermissions();
      if (perms.isEditing) {
        // Currently editing - accept edits and exit (Done)
        acceptEdits();
      } else if (perms.canEnterEdit) {
        // Enter edit mode
        enterEditMode();
      }
    });
  }

  // Discard button - works for both draft solution and edits
  if (mcDiscard) {
    mcDiscard.addEventListener("click", () => {
      const perms = getUiPermissions();
      if (perms.isEditing) {
        cancelEdits();
      } else if (perms.canDiscardDraft) {
        discardDraftSolution();
      }
    });
  }

  // Animate/Pause toggle button - starts, resumes, or pauses animation
  if (mcAnimate) {
    mcAnimate.addEventListener("click", () => {
      const perms = getUiPermissions();
      // If animating, this button acts as Pause
      if (perms.canPause) {
        pauseAnimation();
      } else if (perms.canResume) {
        resumeAnimation();
      } else if (perms.canAnimate) {
        // Start animation for all drones that have a valid trajectory
        const animDrones = Object.keys(state.routes || {})
          .map(String)
          .filter(did => Array.isArray(state.routes[did]?.trajectory) && state.routes[did].trajectory.length >= 2);

        appendDebugLine(`MC Animate: animDrones=[${animDrones.join(",")}] (routes=${Object.keys(state.routes||{}).join(",")})`);

        if (animDrones.length > 0) {
          startAnimation(animDrones);
        } else {
          appendDebugLine("No drones with valid routes to animate");
        }
      }
    });
  }

  // Cut/Checkpoint button
  if (mcCut) {
    mcCut.addEventListener("click", () => {
      cutAtCheckpoint();
    });
  }

  // Reset button
  if (mcReset) {
    mcReset.addEventListener("click", () => {
      resetMission();
    });
  }
}

/**
 * Update Mission Control panel button states and status based on current mode
 */
function updateMissionControlState() {
  const perms = getUiPermissions();
  const mode = missionState.mode;

  // Get all Mission Control buttons
  const mcRunPlanner = $("mc-run-planner");
  const mcDiscard = $("mc-discard");
  const mcEdit = $("mc-edit");
  const mcDisplay = $("mc-display");
  const mcAnimate = $("mc-animate");
  const mcCut = $("mc-cut");
  const mcReset = $("mc-reset");
  const mcStatusText = $("mc-status-text");
  const mcSegmentInfo = $("mc-segment-info");
  const mcPanel = $("mission-control");

  // Run Planner / Accept toggle button
  if (mcRunPlanner) {
    if (perms.canAcceptSolution) {
      // Show as Accept button
      mcRunPlanner.disabled = false;
      mcRunPlanner.textContent = "✓ Accept";
      mcRunPlanner.classList.remove("mc-btn-primary");
      mcRunPlanner.classList.add("mc-btn-success");
    } else if (perms.canSolve) {
      // Show as Run Planner button
      mcRunPlanner.disabled = false;
      mcRunPlanner.textContent = "Run Planner";
      mcRunPlanner.classList.remove("mc-btn-success");
      mcRunPlanner.classList.add("mc-btn-primary");
    } else if (mode === MissionMode.DRAFT_READY) {
      // Solution ready but solving - keep as Accept
      mcRunPlanner.disabled = false;
      mcRunPlanner.textContent = "✓ Accept";
      mcRunPlanner.classList.remove("mc-btn-primary");
      mcRunPlanner.classList.add("mc-btn-success");
    } else {
      // Disabled state (e.g., while solving)
      mcRunPlanner.disabled = true;
      mcRunPlanner.textContent = "Run Planner";
      mcRunPlanner.classList.remove("mc-btn-success");
      mcRunPlanner.classList.add("mc-btn-primary");
    }
  }

  // Display button - enabled when there are routes to display
  if (mcDisplay) {
    const hasRoutes = Object.keys(state.routes).length > 0;
    mcDisplay.disabled = !hasRoutes;
  }

  // Edit button - toggle between Edit and Done
  if (mcEdit) {
    mcEdit.disabled = !perms.canEnterEdit && !perms.isEditing;
    if (perms.isEditing) {
      mcEdit.classList.add("active");
      mcEdit.textContent = "Done";
    } else {
      mcEdit.classList.remove("active");
      mcEdit.textContent = "Edit";
    }
  }

  // Discard - enabled when can discard draft OR is editing
  if (mcDiscard) {
    mcDiscard.disabled = !perms.canDiscardDraft && !perms.isEditing;
  }

  // Animate/Pause toggle button
  if (mcAnimate) {
    if (perms.canPause) {
      // Currently animating - show Pause
      mcAnimate.disabled = false;
      mcAnimate.textContent = "⏸ Pause";
      mcAnimate.classList.remove("mc-btn-play");
      mcAnimate.classList.add("mc-btn-pause");
    } else if (perms.canResume) {
      // Currently paused - show Resume
      mcAnimate.disabled = false;
      mcAnimate.textContent = "▶ Resume";
      mcAnimate.classList.remove("mc-btn-pause");
      mcAnimate.classList.add("mc-btn-play");
    } else if (perms.canAnimate) {
      // Ready to animate - show Animate
      mcAnimate.disabled = false;
      mcAnimate.textContent = "▶ Animate";
      mcAnimate.classList.remove("mc-btn-pause");
      mcAnimate.classList.add("mc-btn-play");
    } else {
      // Not available
      mcAnimate.disabled = true;
      mcAnimate.textContent = "▶ Animate";
      mcAnimate.classList.remove("mc-btn-pause");
      mcAnimate.classList.add("mc-btn-play");
    }
  }

  if (mcCut) {
    mcCut.disabled = !perms.canCut;
  }

  if (mcReset) {
    mcReset.disabled = !perms.canReset;
  }

  // Update status text based on mode
  if (mcStatusText) {
    let statusText = "";
    switch (mode) {
      case MissionMode.IDLE:
        statusText = "Ready - Load environment or Run Planner";
        break;
      case MissionMode.EDITING_ENV:
        statusText = "Editing environment...";
        break;
      case MissionMode.DRAFT_READY:
        statusText = "Draft ready - Accept or Discard";
        break;
      case MissionMode.READY_TO_ANIMATE:
        statusText = "Solution accepted - Click Animate";
        break;
      case MissionMode.ANIMATING:
        statusText = "Animating... Cut to checkpoint";
        break;
      case MissionMode.PAUSED_MID_ANIMATION:
        statusText = "Paused - Resume or Stop";
        break;
      case MissionMode.CHECKPOINT:
        statusText = "Checkpoint - Edit & Re-plan";
        break;
      default:
        statusText = `Mode: ${mode}`;
    }
    mcStatusText.textContent = statusText;
  }

  // Update segment info
  if (mcSegmentInfo) {
    const segmentCount = missionReplay.getSegmentCount();
    const currentSegmentIdx = missionReplay.getCurrentSegmentIndex();
    if (segmentCount > 0) {
      mcSegmentInfo.textContent = `Segment ${currentSegmentIdx + 1}/${segmentCount}`;
    } else {
      mcSegmentInfo.textContent = "Segment 1";
    }
  }

  // Update panel state class for visual feedback
  if (mcPanel) {
    mcPanel.classList.remove("mc-state-animating", "mc-state-checkpoint", "mc-state-draft");
    if (mode === MissionMode.ANIMATING) {
      mcPanel.classList.add("mc-state-animating");
    } else if (mode === MissionMode.CHECKPOINT) {
      mcPanel.classList.add("mc-state-checkpoint");
    } else if (mode === MissionMode.DRAFT_READY) {
      mcPanel.classList.add("mc-state-draft");
    }
  }
}

// ----------------------------------------------------
// Agent Communication
// ----------------------------------------------------
function attachAgentHandlers() {
  const btnSend = $("btn-send-agent");
  const inputEl = $("agent-input");

  if (!btnSend || !inputEl) return;

  // Send on button click
  btnSend.addEventListener("click", () => sendAgentMessage());

  // Send on Enter (Shift+Enter for new line)
  inputEl.addEventListener("keydown", (e) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendAgentMessage();
    }
  });

  // Memory management handlers
  attachMemoryHandlers();

  // Trace tab handlers
  attachTraceHandlers();

  // Load memories on init
  loadAgentMemories();
}

// ----------------------------------------------------
// Agent Memory Management
// ----------------------------------------------------
function attachMemoryHandlers() {
  const btnToggle = $("btn-toggle-memory");
  const btnClear = $("btn-clear-memory");
  const btnAdd = $("btn-add-memory");
  const memoryInput = $("memory-input");

  if (btnToggle) {
    btnToggle.addEventListener("click", () => {
      const panel = $("memory-panel");
      if (panel) {
        const isHidden = panel.style.display === "none";
        panel.style.display = isHidden ? "block" : "none";
        btnToggle.textContent = isHidden ? "Hide" : "Show";
      }
    });
  }

  if (btnClear) {
    btnClear.addEventListener("click", async () => {
      if (!confirm("Clear all agent memories?")) return;
      try {
        const resp = await fetch("/api/agent/memory", { method: "DELETE" });
        const data = await resp.json();
        if (data.success) {
          loadAgentMemories();
          appendDebugLine("Cleared all agent memories");
        }
      } catch (e) {
      }
    });
  }

  if (btnAdd && memoryInput) {
    btnAdd.addEventListener("click", () => addAgentMemory());
    memoryInput.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        addAgentMemory();
      }
    });
  }
}

function attachTraceHandlers() {
  const btnClearTrace = $("btn-clear-trace");
  if (btnClearTrace) {
    btnClearTrace.addEventListener("click", () => {
      if (!confirm("Clear all trace logs?")) return;
      state.agentTraceRuns = [];
      renderAgentsTraceTab();
      appendDebugLine("Cleared agent trace logs");
    });
  }
}

async function loadAgentMemories() {
  try {
    const resp = await fetch("/api/agent/memory");
    const data = await resp.json();
    if (data.success) {
      renderMemoryList(data.memories);
      const countEl = $("memory-count");
      if (countEl) countEl.textContent = data.count;
    }
  } catch (e) {
  }
}

function renderMemoryList(memories) {
  const listEl = $("memory-list");
  if (!listEl) return;

  if (!memories || memories.length === 0) {
    listEl.innerHTML = '<div style="color: #6b7280; font-size: 0.7rem; text-align: center; padding: 0.5rem;">No memories yet. Add instructions the agent should remember.</div>';
    return;
  }

  listEl.innerHTML = memories.map(m => `
    <div class="memory-item" data-id="${m.id}">
      <span class="memory-category">${m.category}</span>
      <span class="memory-content">${escapeHtml(m.content)}</span>
      <button class="btn-delete-memory" onclick="deleteAgentMemory(${m.id})">×</button>
    </div>
  `).join("");
}

async function addAgentMemory() {
  const inputEl = $("memory-input");
  const categoryEl = $("memory-category");
  if (!inputEl || !categoryEl) return;

  const content = inputEl.value.trim();
  if (!content) return;

  const category = categoryEl.value;

  try {
    const resp = await fetch("/api/agent/memory", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ content, category })
    });
    const data = await resp.json();
    if (data.success) {
      inputEl.value = "";
      loadAgentMemories();
      appendDebugLine(`Added memory: [${category}] ${content}`);
    }
  } catch (e) {
  }
}

async function deleteAgentMemory(memoryId) {
  try {
    const resp = await fetch(`/api/agent/memory/${memoryId}`, { method: "DELETE" });
    const data = await resp.json();
    if (data.success) {
      loadAgentMemories();
      appendDebugLine(`Deleted memory #${memoryId}`);
    }
  } catch (e) {
  }
}

async function sendAgentMessage() {
  const inputEl = $("agent-input");
  const chatHistory = $("agent-chat-history");
  const btnSend = $("btn-send-agent");

  if (!inputEl || !chatHistory) return;

  const message = inputEl.value.trim();
  if (!message) return;

  // Hard guard: must have an environment loaded
  if (!state.env) {
    appendDebugLine("⚠️ No environment loaded. Import a JSON first.");
    return;
  }
  // Optional: require at least one target (recommended)
  if (!Array.isArray(state.env.targets) || state.env.targets.length === 0) {
    appendDebugLine("⚠️ Environment has no targets. Add/import targets before Agent Solve.");
    return;
  }

  // UI-permission guard: the Agent panel must follow the same mission state machine
  // as Mission Control to avoid stale env/solution mismatches.
  const perms = (typeof getUiPermissions === "function") ? getUiPermissions() : null;
  if (perms && perms.isAnimating) {
    appendDebugLine("⚠️ Agent Solve disabled during animation. Pause or stop the animation first.");
    updateAgentUiState(perms);
    return;
  }
  if (perms && perms.isEditing) {
    appendDebugLine("⚠️ Agent Solve disabled during Edit mode. Accept or cancel edits first.");
    updateAgentUiState(perms);
    return;
  }
  if (perms && !perms.canSolve) {
    appendDebugLine("⚠️ Agent Solve disabled in the current mission state (paused/replaying/checkpoint-locked or solution already accepted).");
    updateAgentUiState(perms);
    return;
  }

  // Create a Q&A block container
  const qaBlock = createQABlock(message);
  chatHistory.appendChild(qaBlock);
  inputEl.value = "";

  // Scroll to show the new block
  chatHistory.scrollTop = chatHistory.scrollHeight;

  // Disable button while processing
  if (btnSend) btnSend.disabled = true;

  try {
    // Debug: log what we're sending
    const targetCount = state.env?.targets?.length || 0;
    const targetIds = (state.env?.targets || []).map(t => t.id).join(", ");
    appendDebugLine(`🚀 Agent: ${targetCount} targets [${targetIds}]`);

    // Build existing solution context for agent memory
    // This allows follow-up requests like "move T5 to D1" to work
    const existingSolution = {};
    if (state.sequences && Object.keys(state.sequences).length > 0) {
      // Convert sequences like "A1,T1,T2,A1" to route arrays like ["A1","T1","T2","A1"]
      existingSolution.routes = {};
      existingSolution.allocation = {};
      for (const [did, seq] of Object.entries(state.sequences)) {
        if (seq) {
          const routeArr = seq.split(",").map(s => s.trim()).filter(s => s);
          existingSolution.routes[did] = routeArr;
          // Extract targets (non-airport waypoints) for allocation
          existingSolution.allocation[did] = routeArr.filter(wp => wp.startsWith("T"));
        }
      }
      appendDebugLine(`📦 Agent context: ${Object.keys(existingSolution.routes).length} drone routes`);
    }

    // Get LLM settings from localStorage
    const llmSettings = LLMSettings.getCurrentSettings();

    const response = await fetch("/api/agents/chat-v4", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        message,
        env: state.env,
        drone_configs: state.droneConfigs,
        mission_id: state.missionId || null,
        existing_solution: Object.keys(existingSolution).length > 0 ? existingSolution : null,
        llm_config: llmSettings,  // Include LLM provider configuration
      }),
    });

    const data = await response.json();

    // Store mission_id returned by backend, if any
    if (data.mission_id) {
      state.missionId = data.mission_id;
      appendDebugLine("Current mission_id: " + state.missionId);
    }

    // Store trace events for the Agents Trace tab
    if (data.trace_events && Array.isArray(data.trace_events)) {
      state.agentTraceRuns.push({
        ts: Date.now(),
        user_message: message,
        mission_id: data.mission_id || null,
        trace_events: data.trace_events,
        trace: data.trace || null,
      });
      renderAgentsTraceTab();
    }

    // Debug: log the response to see what we're getting
    appendDebugLine("Agent response keys: " + Object.keys(data).join(", "));
    if (data.routes) {
      appendDebugLine("Agent routes: " + JSON.stringify(data.routes));
    }

    // Update the Q&A block with the response
    const responseArea = qaBlock.querySelector(".qa-response");

    if (data.reply) {
      // 1) Escape model reply text
      let safeReply = escapeHtml(String(data.reply || ""));
      safeReply = safeReply.replace(/\n/g, "<br>");

      // 2) Append badges (controlled HTML)
      let badgesHtml = "";
      if (data.intent) {
        badgesHtml += ` <span class="agent-intent-badge">Intent: ${escapeHtml(String(data.intent))}</span>`;
      }

      if (Array.isArray(data.actions) && data.actions.length > 0) {
        const actionNames = data.actions.map(a => a?.type).filter(Boolean).join(", ");
        if (actionNames) badgesHtml += ` <span class="agent-actions-badge">Actions: ${escapeHtml(actionNames)}</span>`;
      }

      if (data.routes && Object.keys(data.routes).length > 0) {
        for (const [did, route] of Object.entries(data.routes)) {
          if (Array.isArray(route) && route.length > 0) {
            badgesHtml += ` <span class="agent-route-badge">D${escapeHtml(String(did))}: ${route.map(x => escapeHtml(String(x))).join(" → ")}</span>`;
          }
        }
      } else if (Array.isArray(data.route) && data.route.length > 0) {
        badgesHtml += ` <span class="agent-route-badge">Route: ${data.route.map(x => escapeHtml(String(x))).join(" → ")}</span>`;
      }

      responseArea.innerHTML = safeReply + (badgesHtml ? "<br><br>" + badgesHtml : "");
      responseArea.classList.remove("thinking");

      // Apply routes to UI
      if (data.routes && Object.keys(data.routes).length > 0) {
        applyAgentRoutes(data.routes, data.trajectories, data.points, data.fuel);
      } else if (Array.isArray(data.route) && data.route.length > 0) {
        applyAgentRoute(data.route, data.points, data.fuel);
      }
    } else {
      responseArea.innerHTML = "No response from agent.";
      responseArea.classList.add("error");
    }
  } catch (err) {
    const responseArea = qaBlock.querySelector(".qa-response");
    responseArea.innerHTML = `Error: ${escapeHtml(err.message)}`;
    responseArea.classList.add("error");
  } finally {
    if (btnSend) btnSend.disabled = false;
    chatHistory.scrollTop = chatHistory.scrollHeight;
  }
}

function createQABlock(question) {
  const timestamp = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  const block = document.createElement("div");
  block.className = "qa-block";
  block.innerHTML = `
    <div class="qa-question">
      <span class="qa-label">YOU [${timestamp}]</span>
      <span class="qa-text">${escapeHtml(question)}</span>
    </div>
    <div class="qa-response-wrapper">
      <div class="qa-response-label">AGENT RESPONSE</div>
      <div class="qa-response thinking">
        🤔 Thinking...
      </div>
    </div>
  `;
  return block;
}

function appendChatMessage(content, type) {
  // Legacy function - kept for system messages
  const chatHistory = $("agent-chat-history");
  if (!chatHistory) return null;

  const msgId = "msg-" + Date.now() + "-" + Math.random().toString(36).substring(2, 7);
  const msgDiv = document.createElement("div");
  msgDiv.id = msgId;
  msgDiv.className = "agent-message system-message";
  msgDiv.innerHTML = content;

  chatHistory.appendChild(msgDiv);
  chatHistory.scrollTop = chatHistory.scrollHeight;
  return msgId;
}

function removeChatMessage(msgId) {
  if (!msgId) return;
  const el = document.getElementById(msgId);
  if (el) el.remove();
}

function escapeHtml(text) {
  const div = document.createElement("div");
  div.textContent = text;
  return div.innerHTML;
}

/**
 * Render the Agents Trace tab with all captured trace runs.
 */
function renderAgentsTraceTab() {
  const container = $("agents-trace-content");
  if (!container) return;

  if (!state.agentTraceRuns || state.agentTraceRuns.length === 0) {
    container.innerHTML = '<div class="trace-empty">No agent runs yet. Send a message in the Agents tab.</div>';
    return;
  }

  let html = '';

  // Show runs in reverse order (newest first)
  for (let i = state.agentTraceRuns.length - 1; i >= 0; i--) {
    const run = state.agentTraceRuns[i];
    const time = new Date(run.ts).toLocaleTimeString();

    html += `<div class="trace-run">`;
    html += `<div class="trace-run-header">`;
    html += `<span class="trace-time">${escapeHtml(time)}</span>`;
    html += `<span class="trace-message">${escapeHtml(run.user_message.substring(0, 50))}${run.user_message.length > 50 ? '...' : ''}</span>`;
    if (run.mission_id) {
      html += `<span class="trace-mission-id">ID: ${escapeHtml(run.mission_id.substring(0, 8))}...</span>`;
    }
    html += `</div>`;

    html += `<div class="trace-events">`;
    for (const evt of run.trace_events) {
      const levelClass = evt.t === 'error' ? 'trace-error' : evt.t.includes('warn') ? 'trace-warn' : 'trace-info';
      html += `<div class="trace-event ${levelClass}">`;
      html += `<span class="trace-type">${escapeHtml(evt.t)}</span>`;
      html += `<span class="trace-msg">${escapeHtml(evt.msg)}</span>`;
      if (evt.data) {
        html += `<details class="trace-data-details"><summary>data</summary><pre class="trace-data">${escapeHtml(JSON.stringify(evt.data, null, 2))}</pre></details>`;
      }
      html += `</div>`;
    }
    html += `</div></div>`;
  }

  container.innerHTML = html;
}

function applyAgentRoutes(routes, trajectories, totalPoints, totalFuel) {
  // Apply multi-drone routes from the agent
  // routes is like {"1": ["A1", "T3", "A1"], "2": ["A2", "T5", "A2"], ...}
  // trajectories is like {"1": [[x1,y1], [x2,y2], ...], ...} - SAM-avoiding paths

  appendDebugLine(`🎯 applyAgentRoutes: ${Object.keys(routes || {}).length} routes`);

  if (!routes || Object.keys(routes).length === 0) {
    appendDebugLine("❌ applyAgentRoutes: No routes to apply");
    return;
  }

  // Build waypoint lookup for calculating distances and points
  const waypoints = {};
  const targetPriorities = {};

  if (state.env) {
    for (const a of (state.env.airports || [])) {
      const id = a.id || a.label || "A?";
      waypoints[id] = [a.x, a.y];
    }
    for (const t of (state.env.targets || [])) {
      const id = t.id || t.label || "T?";
      waypoints[id] = [t.x, t.y];
      targetPriorities[id] = t.priority || t.value || 5;
    }
  }

  // Build allocations for the allocation display
  const allocations = {};

  for (const [droneId, route] of Object.entries(routes)) {
    if (!route || route.length < 2) continue;

    const routeStr = route.join(",");

    // Store sequence
    state.sequences[droneId] = routeStr;

    // Extract targets for allocation display
    const routeTargets = route.filter(wp => String(wp).startsWith('T'));
    allocations[droneId] = routeTargets;

    // Calculate distance (sum of segment lengths)
    let distance = 0;
    for (let i = 0; i < route.length - 1; i++) {
      const from = waypoints[route[i]];
      const to = waypoints[route[i + 1]];
      if (from && to) {
        distance += _distance(from, to);
      }
    }

    // Calculate points (sum of target priorities)
    let points = 0;
    for (const tid of routeTargets) {
      points += targetPriorities[tid] || 0;
    }

    // Use server-provided SAM-avoiding trajectory if available, otherwise build locally
    let trajectory;
    if (trajectories && trajectories[droneId] && trajectories[droneId].length > 0) {
      trajectory = trajectories[droneId];
      appendDebugLine(`Agent route D${droneId}: using SAM-avoiding trajectory (${trajectory.length} points)`);
    } else {
      trajectory = buildTrajectoryFromRoute(route);
    }

    // Build route info for visualization
    state.routes[droneId] = {
      route: route,
      points: points,
      distance: distance,
      trajectory: trajectory,
    };

    // Ensure trajectory is visible for this drone
    state.trajectoryVisible[droneId] = true;

    appendDebugLine(`Agent route D${droneId}: ${routeStr} (${points} pts, ${distance.toFixed(1)} fuel)`);
  }

  // Store allocations
  state.allocations = allocations;

  // Update the sequence input to show the currently selected drone's sequence
  const curDrone = state.currentDroneForSeq || "1";
  const seqInput = $("sequence-input");
  if (seqInput && state.sequences[curDrone]) {
    seqInput.value = state.sequences[curDrone];
  }

  // Update allocation display in Env tab
  updateAllocationDisplay(allocations);

  // Update debug output
  appendDebugLine(`Agent applied ${Object.keys(routes).length} drone routes (${totalPoints || "?"} pts, ${totalFuel?.toFixed(1) || "?"} fuel)`);

  // Redraw and update stats
  drawEnvironment();
  updateStatsFromRoutes();
}

function applyAgentRoute(route, points, fuel) {
  // Apply the agent's route to drone D1 (legacy single-drone)
  // Route is like ["A1", "T3", "T7", "A1"]

  if (!route || route.length < 2) return;

  const routeStr = route.join(",");

  // Store as D1's sequence
  state.sequences["1"] = routeStr;

  // Update the sequence input if it exists
  const seqInput = $("sequence-input");
  if (seqInput) {
    seqInput.value = routeStr;
  }

  // Calculate points and distance if not provided
  let calculatedPoints = points || 0;
  let calculatedFuel = fuel || 0;

  if (!points || !fuel) {
    // Build waypoint lookup
    const waypoints = {};
    const targetPriorities = {};

    if (state.env) {
      for (const a of (state.env.airports || [])) {
        const id = a.id || a.label || "A?";
        waypoints[id] = [a.x, a.y];
      }
      for (const t of (state.env.targets || [])) {
        const id = t.id || t.label || "T?";
        waypoints[id] = [t.x, t.y];
        targetPriorities[id] = t.priority || t.value || 5;
      }
    }

    // Calculate distance
    if (!fuel) {
      calculatedFuel = 0;
      for (let i = 0; i < route.length - 1; i++) {
        const from = waypoints[route[i]];
        const to = waypoints[route[i + 1]];
        if (from && to) {
          calculatedFuel += _distance(from, to);
        }
      }
    }

    // Calculate points
    if (!points) {
      calculatedPoints = 0;
      const routeTargets = route.filter(wp => String(wp).startsWith('T'));
      for (const tid of routeTargets) {
        calculatedPoints += targetPriorities[tid] || 0;
      }
    }
  }

  // Build route info for visualization
  state.routes["1"] = {
    route: route,
    points: calculatedPoints,
    distance: calculatedFuel,
    trajectory: buildTrajectoryFromRoute(route),
  };
}

window.addEventListener("keydown", (e) => {
  if (String(e.key).toLowerCase() === "c") {
    // Skip if user is typing in an input field
    const activeTag = document.activeElement?.tagName?.toLowerCase();
    if (activeTag === "input" || activeTag === "textarea" || activeTag === "select") {
      return;
    }

    appendDebugLine("Keypress: C detected (attempting cut/checkpoint)");
    try {
      cutAtCheckpoint();
    } catch (err) {
      appendDebugLine("Cut failed (see console)");
    }
  }

  // P key for Pause
  if (String(e.key).toLowerCase() === "p") {
    // Skip if user is typing in an input field
    const activeTag = document.activeElement?.tagName?.toLowerCase();
    if (activeTag === "input" || activeTag === "textarea" || activeTag === "select") {
      return;
    }

    const perms = getUiPermissions();
    if (perms.canPause) {
      pauseAnimation();
    }
  }
});

function buildTrajectoryFromRoute(route) {
  // Build trajectory points from waypoint IDs
  if (!state.env || !route) return [];

  const trajectory = [];
  const waypoints = {};

  // Index airports
  for (const a of (state.env.airports || [])) {
    const id = a.id || a.label || "A?";
    waypoints[id] = [a.x, a.y];
  }

  // Index targets
  for (const t of (state.env.targets || [])) {
    const id = t.id || t.label || "T?";
    waypoints[id] = [t.x, t.y];
  }

  // Build trajectory
  for (const wp of route) {
    if (waypoints[wp]) {
      trajectory.push(waypoints[wp]);
    }
  }

  return trajectory;
}

// =============================================================================
// AI Settings Modal
// =============================================================================

const LLMSettings = {
  providers: [],
  currentProvider: null,
  currentModel: null,

  // LocalStorage keys
  STORAGE_KEYS: {
    PROVIDER: "isr_llm_provider",
    MODEL: "isr_llm_model",
    API_KEY_PREFIX: "isr_llm_key_",
  },

  // Initialize settings
  async init() {
    await this.loadProviders();
    this.setupEventListeners();
    this.loadSavedSettings();
  },

  // Fetch available providers from API
  async loadProviders() {
    try {
      const response = await fetch("/api/llm/providers");
      const data = await response.json();
      this.providers = data.providers || [];
      this.populateProviderDropdown();
      this.updateProviderStatus();
    } catch (error) {
      console.error("Failed to load LLM providers:", error);
      this.providers = [];
    }
  },

  // Populate provider dropdown
  populateProviderDropdown() {
    const select = document.getElementById("llm-provider");
    if (!select) return;

    select.innerHTML = this.providers
      .map(
        (p) =>
          `<option value="${p.id}">${p.name}${
            p.configured ? " (configured)" : ""
          }</option>`
      )
      .join("");

    // Trigger model update
    this.onProviderChange();
  },

  // Update model dropdown when provider changes
  onProviderChange() {
    const providerSelect = document.getElementById("llm-provider");
    const modelSelect = document.getElementById("llm-model");
    const apiKeyInput = document.getElementById("llm-api-key");

    if (!providerSelect || !modelSelect) return;

    const providerId = providerSelect.value;
    const provider = this.providers.find((p) => p.id === providerId);

    if (!provider) {
      modelSelect.innerHTML = '<option value="">Select a provider first</option>';
      return;
    }

    // Populate models
    modelSelect.innerHTML = provider.models
      .map((m) => `<option value="${m.id}" title="${m.description}">${m.name}</option>`)
      .join("");

    // Load saved API key for this provider
    const savedKey = this.getApiKey(providerId);
    if (apiKeyInput) {
      apiKeyInput.value = savedKey || "";
    }

    // Load saved model for this provider
    const savedModel = localStorage.getItem(
      this.STORAGE_KEYS.MODEL + "_" + providerId
    );
    if (savedModel && provider.models.some((m) => m.id === savedModel)) {
      modelSelect.value = savedModel;
    }

    this.clearConnectionStatus();
  },

  // Get API key for a provider from localStorage
  getApiKey(providerId) {
    return localStorage.getItem(this.STORAGE_KEYS.API_KEY_PREFIX + providerId) || "";
  },

  // Save API key for a provider to localStorage
  saveApiKey(providerId, apiKey) {
    if (apiKey) {
      localStorage.setItem(this.STORAGE_KEYS.API_KEY_PREFIX + providerId, apiKey);
    } else {
      localStorage.removeItem(this.STORAGE_KEYS.API_KEY_PREFIX + providerId);
    }
  },

  // Load saved settings
  loadSavedSettings() {
    const savedProvider = localStorage.getItem(this.STORAGE_KEYS.PROVIDER);
    const providerSelect = document.getElementById("llm-provider");

    if (savedProvider && providerSelect) {
      const providerExists = this.providers.some((p) => p.id === savedProvider);
      if (providerExists) {
        providerSelect.value = savedProvider;
        this.onProviderChange();
      }
    }
  },

  // Save current settings
  saveSettings() {
    const providerSelect = document.getElementById("llm-provider");
    const modelSelect = document.getElementById("llm-model");
    const apiKeyInput = document.getElementById("llm-api-key");

    if (!providerSelect || !modelSelect) return;

    const providerId = providerSelect.value;
    const modelId = modelSelect.value;
    const apiKey = apiKeyInput?.value?.trim() || "";

    // Save to localStorage
    localStorage.setItem(this.STORAGE_KEYS.PROVIDER, providerId);
    localStorage.setItem(this.STORAGE_KEYS.MODEL + "_" + providerId, modelId);
    this.saveApiKey(providerId, apiKey);

    this.currentProvider = providerId;
    this.currentModel = modelId;

    this.updateProviderStatus();
    this.closeModal();

    appendDebugLine(`AI Settings saved: ${providerId} / ${modelId}`);
  },

  // Test connection
  async testConnection() {
    const providerSelect = document.getElementById("llm-provider");
    const modelSelect = document.getElementById("llm-model");
    const apiKeyInput = document.getElementById("llm-api-key");
    const statusEl = document.getElementById("connection-status");

    if (!providerSelect || !modelSelect || !apiKeyInput || !statusEl) return;

    const providerId = providerSelect.value;
    const modelId = modelSelect.value;
    const apiKey = apiKeyInput.value.trim();

    if (!apiKey) {
      statusEl.textContent = "Please enter an API key";
      statusEl.className = "connection-status error";
      return;
    }

    statusEl.textContent = "Testing...";
    statusEl.className = "connection-status testing";

    try {
      const response = await fetch("/api/llm/test", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          provider: providerId,
          model: modelId,
          api_key: apiKey,
        }),
      });

      const data = await response.json();

      if (data.success) {
        statusEl.textContent = data.message;
        statusEl.className = "connection-status success";
      } else {
        statusEl.textContent = data.message;
        statusEl.className = "connection-status error";
      }
    } catch (error) {
      statusEl.textContent = "Connection failed: " + error.message;
      statusEl.className = "connection-status error";
    }
  },

  // Clear connection status
  clearConnectionStatus() {
    const statusEl = document.getElementById("connection-status");
    if (statusEl) {
      statusEl.textContent = "";
      statusEl.className = "connection-status";
    }
  },

  // Update provider status display
  updateProviderStatus() {
    const container = document.getElementById("provider-status");
    if (!container) return;

    const providerItems = this.providers
      .map((p) => {
        const hasKey = !!this.getApiKey(p.id);
        const statusClass = hasKey ? "configured" : "not-configured";
        const statusIcon = hasKey ? "&#10003; Key saved" : "&#x2717; No key";
        return `
          <div class="provider-item">
            <span class="provider-name">${p.name}</span>
            <span class="provider-status-icon ${statusClass}">${statusIcon}</span>
          </div>
        `;
      })
      .join("");

    container.innerHTML = `
      <h4>Provider Status</h4>
      <div class="provider-list">
        ${providerItems}
      </div>
    `;
  },

  // Open modal
  openModal() {
    const modal = document.getElementById("settings-modal");
    if (modal) {
      modal.style.display = "flex";
      this.updateProviderStatus();
    }
  },

  // Close modal
  closeModal() {
    const modal = document.getElementById("settings-modal");
    if (modal) {
      modal.style.display = "none";
    }
  },

  // Toggle API key visibility
  toggleKeyVisibility() {
    const input = document.getElementById("llm-api-key");
    const btn = document.getElementById("btn-toggle-key");
    if (!input || !btn) return;

    if (input.type === "password") {
      input.type = "text";
      btn.textContent = "Hide";
    } else {
      input.type = "password";
      btn.textContent = "Show";
    }
  },

  // Setup event listeners
  setupEventListeners() {
    // Settings button
    const btnSettings = document.getElementById("btn-settings");
    if (btnSettings) {
      btnSettings.addEventListener("click", () => this.openModal());
    }

    // Close button
    const btnClose = document.getElementById("btn-close-settings");
    if (btnClose) {
      btnClose.addEventListener("click", () => this.closeModal());
    }

    // Click outside to close
    const modal = document.getElementById("settings-modal");
    if (modal) {
      modal.addEventListener("click", (e) => {
        if (e.target === modal) this.closeModal();
      });
    }

    // Provider change
    const providerSelect = document.getElementById("llm-provider");
    if (providerSelect) {
      providerSelect.addEventListener("change", () => this.onProviderChange());
    }

    // Test connection
    const btnTest = document.getElementById("btn-test-connection");
    if (btnTest) {
      btnTest.addEventListener("click", () => this.testConnection());
    }

    // Save settings
    const btnSave = document.getElementById("btn-save-settings");
    if (btnSave) {
      btnSave.addEventListener("click", () => this.saveSettings());
    }

    // Toggle key visibility
    const btnToggle = document.getElementById("btn-toggle-key");
    if (btnToggle) {
      btnToggle.addEventListener("click", () => this.toggleKeyVisibility());
    }
  },

  // Get current LLM settings (for API calls)
  getCurrentSettings() {
    const provider = localStorage.getItem(this.STORAGE_KEYS.PROVIDER);
    if (!provider) return null;

    const model = localStorage.getItem(this.STORAGE_KEYS.MODEL + "_" + provider);
    const apiKey = this.getApiKey(provider);

    if (!apiKey) return null;

    return { provider, model, api_key: apiKey };
  },
};

// Initialize LLM Settings when DOM is ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", () => LLMSettings.init());
} else {
  LLMSettings.init();
}