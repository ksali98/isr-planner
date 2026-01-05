// isr.js  — single-file version with:
// - Edit mode ON/OFF
// - Canvas drawing + editing (airports/targets/SAMs)
// - Drone config + sequences
// - Stats
// - Import/Export
// - Planner integration

"use strict";

console.log("isr.js loaded", new Date().toISOString());

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

  // Drones lost at the current checkpoint (disabled after cut but before solve)
  // Array of drone IDs: ["1", "3"] - used when drawing lost drone markers
  pendingLostDrones: [],

  // Legacy single cut position (backward compatibility)
  pendingCutPosition: null,
};

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
// MissionReplay - Clean architecture for segment-based mission replay
// ----------------------------------------------------

/**
 * Segment - Immutable data structure for a mission segment
 * A segment represents one "solve" of the mission, including:
 * - The environment state at that point
 * - The solution (routes/trajectories)
 * - The checkpoint boundary (where this segment ends and next begins)
 */
class Segment {
  constructor({
    index,
    solution,
    env,
    cutDistance = null,      // Single distance value where this segment starts (fuel/distance at cut)
    prefixDistances = null,  // DEPRECATED: kept for backward compatibility during transition
    isCheckpointReplan = false,
    timestamp = new Date().toISOString(),
    cutPosition = null,      // [x, y] position where segment was cut (legacy, drone 1)
    cutPositions = null,     // { [droneId]: [x, y] } per-drone cut positions for marker display
    lostDrones = null,       // Array of drone IDs that were lost at this checkpoint
    visitedTargets = null    // Array of target IDs visited up to this segment
  }) {
    this.index = index;
    this.solution = Object.freeze(JSON.parse(JSON.stringify(solution)));
    this.env = Object.freeze(JSON.parse(JSON.stringify(env)));

    // New simple model: single cut distance for the segment
    // If cutDistance not provided, try to extract from prefixDistances (backward compat)
    if (cutDistance !== null) {
      this.cutDistance = cutDistance;
    } else if (prefixDistances && Object.keys(prefixDistances).length > 0) {
      // Use the first drone's prefix distance as the cut distance (they should all be the same)
      this.cutDistance = Object.values(prefixDistances)[0];
    } else {
      this.cutDistance = null;
    }

    this.isCheckpointReplan = isCheckpointReplan;
    this.timestamp = timestamp;
    this.cutPosition = cutPosition ? Object.freeze([...cutPosition]) : null;
    // Per-drone cut positions for drawing markers
    this.cutPositions = cutPositions ? Object.freeze(
      Object.fromEntries(
        Object.entries(cutPositions).map(([did, pos]) => [did, Object.freeze([...pos])])
      )
    ) : null;
    // Lost drones at this checkpoint (frozen array of drone ID strings)
    this.lostDrones = lostDrones ? Object.freeze([...lostDrones]) : null;
    // Visited targets up to this segment
    this.visitedTargets = visitedTargets ? Object.freeze([...visitedTargets]) : null;
    Object.freeze(this);
  }

  /**
   * Get the cut distance for this segment (where it starts)
   * Returns null for segment 0 (starts at 0)
   */
  getCutDistance() {
    return this.cutDistance;
  }

  /**
   * Check if this segment has a cut boundary (i.e., starts after distance 0)
   */
  hasCheckpointBoundary() {
    return this.cutDistance !== null && this.cutDistance > 0;
  }
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
class MissionReplay {
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
    const segment = new Segment({
      index: this._segments.length,
      ...segmentData
    });
    this._segments.push(segment);
    console.log(`[MissionReplay] Added segment ${segment.index}, isCheckpointReplan=${segment.isCheckpointReplan}`);
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
      console.log(`[MissionReplay] Set current segment to ${index}`);
    }
  }

  /**
   * Reset to beginning for replay
   */
  resetToStart() {
    this._currentSegmentIndex = 0;
    console.log(`[MissionReplay] Reset to start, ${this._segments.length} segments available`);
  }

  /**
   * Clear all solutions from all segments (for Reset in segmentInfo workflow)
   * Keeps env, cutDistance, cutPositions intact
   */
  clearAllSolutions() {
    for (const seg of this._segments) {
      seg.solution = { routes: {}, sequences: {} };
    }
    console.log(`[MissionReplay] Cleared solutions from ${this._segments.length} segments`);
  }

  /**
   * Clear all segments (full reset)
   */
  clear() {
    this._segments = [];
    this._currentSegmentIndex = 0;
    console.log(`[MissionReplay] Cleared all segments`);
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
      console.log(`[MissionReplay] Truncated to ${this._segments.length} segments`);
    }
  }

  /**
   * Replace a segment at a given index with new data
   * Creates a new frozen Segment with the provided data
   */
  replaceSegment(index, segmentData) {
    if (index < 0 || index >= this._segments.length) {
      console.warn(`[MissionReplay] Cannot replace segment at invalid index ${index}`);
      return false;
    }
    const newSegment = new Segment({
      index: index,
      ...segmentData
    });
    this._segments[index] = newSegment;
    console.log(`[MissionReplay] Replaced segment ${index}`);
    return true;
  }

  /**
   * Update the solution in the current segment (e.g., after optimization)
   */
  updateCurrentSegmentSolution(routes, sequences) {
    const segment = this.getCurrentSegment();
    if (!segment) {
      console.warn("[MissionReplay] No current segment to update");
      return false;
    }
    // Update the solution's routes and sequences
    if (segment.solution) {
      segment.solution.routes = JSON.parse(JSON.stringify(routes));
      if (sequences) {
        segment.solution.sequences = JSON.parse(JSON.stringify(sequences));
      }
      console.log(`[MissionReplay] Updated segment ${segment.index} solution with optimized routes`);
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
      console.log(`[MissionReplay] Segment switch: ${fromIndex} -> ${this._currentSegmentIndex} (distance ${missionDistance.toFixed(1)} >= cut ${cutDistance.toFixed(1)})`);

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
        envTargets: s.env.targets?.length ?? 0,
        routeCount: Object.keys(s.solution.routes || {}).length
      }))
    };
  }
}

// Global mission replay instance
const missionReplay = new MissionReplay();

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
/**
 * SegmentedImportManager - Wrapper around new SegmentedMissionManager
 * Provides backward-compatible API while using the clean implementation from segmented_mission.js
 */
class SegmentedImportManager {
  constructor() {
    // Use the clean implementation from segmented_mission.js
    this._manager = new window.SegmentedMissionManager();
  }

  // Legacy property access - proxy to manager
  get _currentSegmentIndex() {
    return this._manager.getCurrentSegmentIndex();
  }
  set _currentSegmentIndex(value) {
    this._manager.resetToSegment(value);
  }

  /** Clear all segmented import state */
  clear() {
    this._manager.clear();
  }

  /** Check if we're in segmented import mode */
  isActive() {
    return this._manager.isActive();
  }

  /** Get current segment index */
  getCurrentSegmentIndex() {
    return this._manager.getCurrentSegmentIndex();
  }

  /** Get all targets (for drawing) */
  getAllTargets() {
    return this._manager.getAllTargets();
  }

  // ========================================
  // GETTERS - Delegate to _manager
  // ========================================

  /** Get complete data for a segment */
  getSegmentData(segmentIndex) {
    return this._manager.getSegmentData(segmentIndex);
  }

  /** Get frozen targets for a segment (targets visited BEFORE this segment) */
  getFrozenTargetsForSegment(segmentIndex) {
    return this._manager.getFrozenTargets(segmentIndex);
  }

  /** Get frozen trajectories for a segment */
  getFrozenTrajectoriesForSegment(segmentIndex) {
    return this._manager.getFrozenTrajectories(segmentIndex);
  }

  /** Get active targets for a segment (targets to solve) */
  getActiveTargetsForSegment(segmentIndex) {
    return this._manager.getActiveTargets(segmentIndex);
  }

  /** Get drone_configs for a segment (with correct enabled states) */
  getDroneConfigsForSegment(segmentIndex) {
    return this._manager.getDroneConfigs(segmentIndex);
  }

  /** Get start positions for a segment (null for seg-0, cut positions for seg>0) */
  getStartPositionsForSegment(segmentIndex) {
    return this._manager.getStartPositions(segmentIndex);
  }

  /** Get SAMs for a segment */
  getSamsForSegment(segmentIndex) {
    const seg = this._manager.getSegmentData(segmentIndex);
    if (seg?.sams) {
      return JSON.parse(JSON.stringify(seg.sams));
    }
    const baseEnv = this._manager.getFullEnv();
    return baseEnv?.sams ? JSON.parse(JSON.stringify(baseEnv.sams)) : [];
  }

  /** Get base environment (for initial display) */
  getBaseEnv() {
    return this._manager.getFullEnv();
  }

  /** Get cut distance for segment */
  getCutDistanceForSegment(segmentIndex) {
    return this._manager.getCutDistance(segmentIndex);
  }

  /** Get lost drones at this segment's checkpoint */
  getLostDronesForSegment(segmentIndex) {
    return this._manager.getLostDrones(segmentIndex);
  }

  // Legacy getters for backward compatibility
  getVisitedTargetsForSegment(segmentIndex) {
    return this.getFrozenTargetsForSegment(segmentIndex);
  }

  getCutPositionForSegment(segmentIndex) {
    return this.getStartPositionsForSegment(segmentIndex);
  }

  getCumulativeLostDrones(segmentIndex) {
    const allLost = new Set();
    for (let i = 0; i <= segmentIndex; i++) {
      const lost = this._manager.getLostDrones(i);
      lost.forEach(d => allLost.add(d));
    }
    return Array.from(allLost);
  }

  /** Get total number of segments */
  getTotalSegments() {
    return this._manager.getTotalSegments();
  }

  /** Advance to next segment */
  advanceToNextSegment() {
    return this._manager.advanceToNextSegment();
  }

  /** Check if all segments are complete */
  isComplete() {
    const idx = this._manager.getCurrentSegmentIndex();
    const total = this._manager.getTotalSegments();
    return idx >= total - 1;
  }

  /** Get unfrozen targets for solver (active targets for current segment) */
  getUnfrozenTargets() {
    return this._manager.getActiveTargets(this._manager.getCurrentSegmentIndex());
  }

  /** Get targets to DISPLAY for current segment */
  getTargetsToDisplay() {
    // Delegate to manager's display env
    const env = this._manager.getEnvForDisplay();
    return env?.targets || [];
  }

  /** Get env for DISPLAY (only targets up to current segment) */
  getEnvForDisplay() {
    return this._manager.getEnvForDisplay();
  }

  /** Get base env with all targets (for drawing) */
  getFullEnv() {
    return this._manager.getFullEnv();
  }

  /** Get env for solver (filtered targets, synthetic starts for segment > 0) */
  getEnvForSolver() {
    // Use manager's solver env, passing state.droneConfigs as source of truth
    return this._manager.getEnvForSolver(state.droneConfigs);
  }

  /**
   * Load from segmentInfo format JSON
   * @param {Object} data - JSON with type:"segmented", targets, segmentInfo.segmentCuts
   */
  loadFromSegmentInfo(data) {
    this.clear();

    // Convert segmentInfo format to segments array format for the manager
    const allTargets = JSON.parse(JSON.stringify(data.targets || []));
    const baseDroneConfigs = JSON.parse(JSON.stringify(data.drone_configs || {}));
    const segmentCuts = data.segmentInfo?.segmentCuts || [];
    const numSegments = segmentCuts.length + 1;

    // Build segments array
    const segments = [];
    const cumulativeFrozen = [];

    for (let i = 0; i < numSegments; i++) {
      const cut = i > 0 ? segmentCuts[i - 1] : null;
      const frozenTargets = [...cumulativeFrozen];
      const frozenSet = new Set(frozenTargets);
      const activeTargetIds = allTargets.map(t => t.id).filter(id => !frozenSet.has(id));
      const activeTargets = allTargets.filter(t => activeTargetIds.includes(t.id));

      let startPositions = null;
      if (cut?.dronePositions) {
        startPositions = {};
        Object.entries(cut.dronePositions).forEach(([did, posData]) => {
          if (posData.position || (Array.isArray(posData) && posData.length === 2)) {
            startPositions[did] = posData.position || posData;
          }
        });
      }

      const firstDronePos = cut ? Object.values(cut.dronePositions || {})[0] : null;
      const cutDistance = firstDronePos?.distanceTraveled || firstDronePos?.totalDistance || 0;

      segments.push({
        frozenTargets,
        activeTargets: activeTargetIds,
        droneConfigs: JSON.parse(JSON.stringify(baseDroneConfigs)),
        startPositions,
        cutDistance,
        lostDrones: [],
        env: {
          targets: activeTargets,
          airports: data.airports || [],
          sams: data.sams || []
        }
      });

      if (cut?.visitedTargets) {
        cut.visitedTargets.forEach(t => {
          if (!cumulativeFrozen.includes(t)) cumulativeFrozen.push(t);
        });
      }
    }

    // Load via manager
    const convertedData = {
      schema: "isr_env_v2",
      segments,
      airports: data.airports || [],
      sams: data.sams || [],
      drone_configs: baseDroneConfigs
    };

    this._manager.loadFromSegmentsArray(convertedData);
    console.log(`[SegmentedImportManager] loadFromSegmentInfo: ${numSegments} segments, ${allTargets.length} targets`);
    return true;
  }

  /**
   * Load from segments array format - delegates to new manager
   */
  loadFromSegmentsArray(data) {
    this.clear();

    const result = this._manager.loadFromSegmentsArray(data);
    if (!result) return false;

    console.log(`[SegmentedImportManager] loadFromSegmentsArray: ${this._manager.getTotalSegments()} segments`);
    return true;
  }

  /** Debug info */
  getDebugInfo() {
    const segIdx = this._manager.getCurrentSegmentIndex();
    return {
      isActive: this._manager.isActive(),
      currentSegment: segIdx,
      totalSegments: this._manager.getTotalSegments(),
      allTargets: this._manager.getAllTargets().map(t => t.id),
      visitedNow: this._manager.getFrozenTargets(segIdx),
      unfrozen: this._manager.getActiveTargets(segIdx).map(t => t.id),
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
    console.log(`[mergeEnvForward] SKIPPING - segmentInfo workflow should not merge targets forward`);
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
    console.log(`[mergeEnvForward] Merged env into segmentedMission segments ${buildIdx + 1} to ${missionState.segmentedMission.segments.length - 1}`);
  }

  if (startSegmentIndex < missionReplay.getSegmentCount()) {
    console.log(`[mergeEnvForward] Merged env into missionReplay segments ${startSegmentIndex} to ${missionReplay.getSegmentCount() - 1}`);
  }
}

/**
 * Get UI permissions based on current mission mode
 * Returns which actions are allowed in the current state
 */
function getUiPermissions() {
  const mode = missionState.mode;
  const hasCommittedPlan = missionState.committedSegments.length > 0;
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
  console.log(`[MissionMode] ${oldMode} → ${newMode}${reason ? ` (${reason})` : ""}`);
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
  console.log(`[acceptSolution] ENTERED - mode=${missionState.mode}, draftSolution=${!!missionState.draftSolution}`);
  const perms = getUiPermissions();
  console.log(`[acceptSolution] perms.canAcceptSolution=${perms.canAcceptSolution}`);
  if (!perms.canAcceptSolution) {
    console.log(`[acceptSolution] BLOCKED - canAcceptSolution is false`);
    appendDebugLine("Cannot accept solution in current state");
    return;
  }

  if (!missionState.draftSolution) {
    console.log(`[acceptSolution] BLOCKED - no draftSolution`);
    appendDebugLine("No draft solution to accept");
    return;
  }

  // Check if we're using the new SegmentedImportManager
  const useNewManager = segmentedImport.isActive();
  console.log(`[acceptSolution] useNewManager=${useNewManager}, segmentedImport debug:`, segmentedImport.getDebugInfo());

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

  // Debug: show what's in state.env at start of accept
  appendDebugLine(`[ACCEPT START] state.env.targets=${(state.env.targets||[]).map(t=>t.id).join(",")}`);
  appendDebugLine(`[ACCEPT START] currentBuildSegmentIndex=${missionState.currentBuildSegmentIndex}`);

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

  // Check if we're in old segmentInfo workflow
  const isSegmentInfoWorkflow = (
    (state.importedSegmentCuts && state.importedSegmentCuts.length > 0) ||
    (existingSegment &&
      existingSegment.solution &&
      Object.keys(existingSegment.solution.routes || {}).length === 0 &&
      missionReplay.getSegmentCount() > 1)
  );

  // IMPORTANT: Use state.routes (which may have been optimized) instead of missionState.draftSolution.routes
  // The optimizers (Insert Missed, Trajectory Swap, Crossing Removal) update state.routes directly,
  // but NOT missionState.draftSolution. So we need to capture the CURRENT optimized routes.
  const solutionToStore = JSON.parse(JSON.stringify(missionState.draftSolution || {}));
  if (state.routes && Object.keys(state.routes).length > 0) {
    solutionToStore.routes = JSON.parse(JSON.stringify(state.routes));
    console.log(`[acceptSolution LEGACY] Using OPTIMIZED routes from state.routes`);
  }

  if (isSegmentInfoWorkflow) {
    // Old SegmentInfo workflow
    missionReplay.replaceSegment(currentIdx, {
      solution: solutionToStore,
      env: envSnapshot,
      cutDistance: existingSegment.cutDistance,
      cutPositions: existingSegment.cutPositions,
      isCheckpointReplan: existingSegment.isCheckpointReplan,
    });
    segment = missionReplay.getSegment(currentIdx);
    if (nextSegment) {
      missionReplay.setCurrentSegmentIndex(currentIdx + 1);
      missionState.currentBuildSegmentIndex = currentIdx + 1;
    }
  } else {
    // Normal workflow: ADD new segment
    segment = missionReplay.addSegment({
      solution: solutionToStore,
      env: envSnapshot,
      cutDistance: state.pendingCutDistance || null,
      isCheckpointReplan: solutionToStore.isCheckpointReplan || false,
      cutPositions: state.pendingCutPositions
        ? JSON.parse(JSON.stringify(state.pendingCutPositions))
        : null,
      lostDrones: state.pendingLostDrones.length > 0
        ? [...state.pendingLostDrones]
        : null,
    });
    missionReplay.setCurrentSegmentIndex(segment.index);
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
  state.pendingLostDrones = [];
  state.pendingCutDistance = null;

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

    // PRESERVE user's droneConfigs - UI checkboxes are the source of truth
    // state.droneConfigs already reflects user's choices; don't overwrite with env values
    state.env.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));
    missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));
    missionState.acceptedEnv.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));

    // Restore cut positions so the marker stays visible
    // The marker shows where the drone will start for the next segment
    if (savedCutPositions) {
      state.pendingCutPositions = savedCutPositions;
    }

    appendDebugLine(`[ACCEPT] Fresh segmentation: now ${state.env.targets?.length || 0} targets, ${state.visitedTargets.length} visited`);
    appendDebugLine(`[ACCEPT] pendingCutPositions: ${JSON.stringify(state.pendingCutPositions)}`);

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
  console.log(`[acceptSolutionWithManager] ENTRY. Mode: ${missionState.mode}`);
  const currentSegIdx = segmentedImport.getCurrentSegmentIndex();
  appendDebugLine(`[ACCEPT-NEW] Segment ${currentSegIdx}, manager: ${JSON.stringify(segmentedImport.getDebugInfo())}`);

  // IMPORTANT: Use state.routes (which may have been optimized) instead of missionState.draftSolution.routes
  // The optimizers (Insert Missed, Trajectory Swap, Crossing Removal) update state.routes directly,
  // but NOT missionState.draftSolution. So we need to capture the CURRENT optimized routes.
  const solutionToStore = JSON.parse(JSON.stringify(missionState.draftSolution || {}));

  // Override the routes with the current (potentially optimized) routes from state.routes
  if (state.routes && Object.keys(state.routes).length > 0) {
    solutionToStore.routes = JSON.parse(JSON.stringify(state.routes));
    console.log(`[acceptSolutionWithManager] Using OPTIMIZED routes from state.routes (not draftSolution)`);
  } else {
    console.log(`[acceptSolutionWithManager] Using draftSolution routes (no state.routes)`);
  }

  // 1. Update MissionReplay segment with the solution (using optimized routes)
  // IMPORTANT: Store the current segment's droneConfigs so export can use it
  const currentSeg = missionReplay.getSegment(currentSegIdx);
  const segmentEnv = segmentedImport.getFullEnv();
  // Use the CURRENT state.droneConfigs which reflects what was actually used for this solve
  segmentEnv.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));

  if (currentSeg) {
    missionReplay.replaceSegment(currentSegIdx, {
      solution: solutionToStore,
      env: segmentEnv,
      cutDistance: currentSeg.cutDistance,
      cutPositions: currentSeg.cutPositions,
      isCheckpointReplan: currentSeg.isCheckpointReplan,
    });
  }

  // 2. Add to committed segments for backward compatibility
  missionState.committedSegments.push({
    index: currentSegIdx,
    solution: JSON.parse(JSON.stringify(solutionToStore)),
    env: JSON.parse(JSON.stringify(segmentEnv)),  // Use segmentEnv with correct droneConfigs
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
    // The JSON has CUMULATIVE cutDistance (from mission start).
    // We need to use the FULL concatenated trajectory (all segments so far) to find the marker.
    const cutDistance = segmentedImport.getCutDistanceForSegment(newSegIdx);

    state.visitedTargets = [];
    state.pendingCutPositions = {};

    console.log(`[ACCEPT] Segment ${newSegIdx}: cutDistance=${cutDistance}`);

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

    // Calculate marker position for each drone
    // For newSegIdx > 1: use state.routes (actual displayed trajectory)
    // For newSegIdx === 1: use concatenatedRoutes (built from MissionReplay)
    const routesToUse = newSegIdx === 1 ? concatenatedRoutes : state.routes;
    Object.entries(routesToUse || {}).forEach(([did, routeData]) => {
      const traj = routeData.trajectory || [];

      if (traj.length < 2 || cutDistance <= 0) return;

      // Calculate marker position by splitting CONCATENATED trajectory at cumulative cutDistance
      const result = split_polyline_at_distance(traj, cutDistance);
      if (result.splitPoint && result.splitPoint.length === 2) {
        state.pendingCutPositions[did] = result.splitPoint;
        console.log(`[ACCEPT] Cut marker D${did}: concatenated traj=${traj.length}pts, dist=${cutDistance.toFixed(1)} -> [${result.splitPoint[0].toFixed(2)}, ${result.splitPoint[1].toFixed(2)}]`);
      }
    });

    // Calculate visited targets from the ACTUAL optimized trajectory, not the JSON
    // This is critical: after Swap Closer, targets may have been swapped between drones,
    // so the JSON's visitedTargets no longer reflects reality
    const jsonVisited = segmentedImport.getVisitedTargetsForSegment(newSegIdx);
    const actualVisited = calculateVisitedTargetsFromRoutes(routesToUse, cutDistance);

    // Use actual visited targets if we have them, fall back to JSON
    if (actualVisited.length > 0) {
      state.visitedTargets = actualVisited;
      console.log(`[ACCEPT] Visited targets from ACTUAL trajectory: ${actualVisited.join(", ")}`);
      if (JSON.stringify(actualVisited.sort()) !== JSON.stringify(jsonVisited.sort())) {
        console.log(`[ACCEPT] NOTE: Differs from JSON visited: ${jsonVisited.join(", ")}`);
      }
    } else {
      state.visitedTargets = jsonVisited;
      console.log(`[ACCEPT] Visited targets from JSON (no actual): ${jsonVisited.join(", ")}`);
    }

    if (Object.keys(state.pendingCutPositions).length === 0) {
      state.pendingCutPositions = null;
    }

    appendDebugLine(`[ACCEPT] Advanced to segment ${newSegIdx}, cutDist=${cutDistance.toFixed(1)}, visited=[${state.visitedTargets.join(",")}]`);

    // 6. Update MissionReplay index and save cut positions for the NEXT segment
    missionReplay.setCurrentSegmentIndex(newSegIdx);
    missionState.currentBuildSegmentIndex = newSegIdx;

    // Save the cut positions, visited targets, and lost drones into the next segment
    // This allows us to draw all previous markers and restore correct visited targets on Reset
    // Determine lost drones for this segment:
    // 1. For segmented imports: use lostDrones from SegmentedImportManager (derived from droneConfigs)
    // 2. For fresh segmentation: use state.pendingLostDrones (set during checkpoint replan)
    let lostDronesForSegment = null;
    if (segmentedImport.isActive()) {
      const importedLostDrones = segmentedImport.getLostDronesForSegment(newSegIdx);
      lostDronesForSegment = importedLostDrones.length > 0 ? [...importedLostDrones] : null;
      console.log(`[ACCEPT] Using lostDrones from segmentedImport: [${importedLostDrones.join(",")}]`);
    } else if (state.pendingLostDrones && state.pendingLostDrones.length > 0) {
      lostDronesForSegment = [...state.pendingLostDrones];
      console.log(`[ACCEPT] Using lostDrones from pendingLostDrones: [${lostDronesForSegment.join(",")}]`);
    }

    const nextSeg = missionReplay.getSegment(newSegIdx);
    if (nextSeg) {
      missionReplay.replaceSegment(newSegIdx, {
        ...nextSeg,
        cutPositions: state.pendingCutPositions ? JSON.parse(JSON.stringify(state.pendingCutPositions)) : nextSeg.cutPositions,
        visitedTargets: [...state.visitedTargets],  // Save calculated visited targets
        lostDrones: lostDronesForSegment,  // Drones lost at this checkpoint (from import or pending)
      });
      console.log(`[ACCEPT] Saved to segment ${newSegIdx}: cutPositions=`, state.pendingCutPositions, `visitedTargets=[${state.visitedTargets.join(",")}], lostDrones=`, lostDronesForSegment);
    }
    // More segments to solve
    // PRESERVE existing frozen trajectory and DON'T recalculate it
    // The frozen trajectory is IMMUTABLE once placed
    let frozenRoutes = {};

    // ========================================
    // ALWAYS calculate frozen trajectories from ACTUAL solved routes
    // Do NOT use stored frozen trajectories because:
    // 1. User may have re-solved with different allocations
    // 2. User may have optimized trajectories (Swap Closer, etc.)
    // 3. Stored trajectories are from the ORIGINAL export, not current session
    // ========================================
    {
      // V1 FORMAT (or fresh segmentation): Calculate frozen trajectories from actual routes
      console.log(`[ACCEPT] Calculating frozen trajectories (no stored data)`);

      // For each drone, keep the existing frozen trajectory (already in state.routes)
      // Do NOT rebuild from MissionReplay - that would use NEW solver trajectories
      const droneIds = newSegIdx === 1
        ? Object.keys(concatenatedRoutes || {})
        : Object.keys(state.routes || {});

      droneIds.forEach((droneId) => {
        if (newSegIdx > 1 && state.routes[droneId] && state.routes[droneId].trajectory) {
          // state.routes has the MERGED trajectory (frozen + new segment)
          // We need to truncate it at the current cutDistance to get the NEW frozen part
          const mergedTraj = state.routes[droneId].trajectory || [];

          if (mergedTraj.length > 0 && cutDistance > 0) {
            const result = split_polyline_at_distance(mergedTraj, cutDistance);
            frozenRoutes[droneId] = {
              ...state.routes[droneId],
              trajectory: result.prefixPoints,
            };
            console.log(`[ACCEPT] Frozen D${droneId}: ${mergedTraj.length}pts -> ${result.prefixPoints.length}pts (truncated at C${newSegIdx} marker from merged, dist=${cutDistance.toFixed(1)})`);
          } else if (mergedTraj.length > 0) {
            // Final segment - keep full
            frozenRoutes[droneId] = JSON.parse(JSON.stringify(state.routes[droneId]));
            console.log(`[ACCEPT] Frozen D${droneId}: ${mergedTraj.length}pts (final segment, no truncation)`);
          }
        } else if (newSegIdx === 1 && concatenatedRoutes && concatenatedRoutes[droneId]) {
          // First Accept - freeze the trajectory up to the marker
          const fullTraj = concatenatedRoutes[droneId].trajectory || [];

          // Get the full route data from the last segment for this drone
          let lastRouteData = null;
          for (let i = newSegIdx - 1; i >= 0; i--) {
            const seg = missionReplay.getSegment(i);
            if (seg?.solution?.routes?.[droneId]) {
              lastRouteData = seg.solution.routes[droneId];
              break;
            }
          }

          if (fullTraj.length > 0 && cutDistance > 0) {
            const result = split_polyline_at_distance(fullTraj, cutDistance);
            frozenRoutes[droneId] = {
              ...(lastRouteData || {}),
              trajectory: result.prefixPoints,
            };
            console.log(`[ACCEPT] Initial freeze D${droneId}: ${fullTraj.length}pts -> ${result.prefixPoints.length}pts (truncated at C${newSegIdx} marker, dist=${cutDistance.toFixed(1)})`);
          } else if (fullTraj.length > 0) {
            // No cutDistance - keep full trajectory
            frozenRoutes[droneId] = {
              ...(lastRouteData || {}),
              trajectory: fullTraj,
            };
            console.log(`[ACCEPT] Initial freeze D${droneId}: ${fullTraj.length}pts (final segment, no truncation)`);
          }
        }
      });
    }

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

        console.log(`[ACCEPT] Segment ${currentSegIdx} D${did}: cumulative=${cumulativeTraj.length}pts, prev=${prevTraj.length}pts, thisSegment=${segmentTraj.length}pts`);
      });

      missionReplay.replaceSegment(currentSegIdx, {
        ...currentSeg,
        solution: segmentSolution,
      });
    }

    state.routes = frozenRoutes;
    console.log(`[ACCEPT] Frozen routes from ${newSegIdx} segments:`, Object.keys(frozenRoutes));

    // Make frozen trajectories visible
    Object.keys(frozenRoutes).forEach(did => {
      state.trajectoryVisible[did] = true;
    });

    // Set up env for drawing - targets up to current segment (progressive reveal)
    const displayEnv = segmentedImport.getEnvForDisplay();
    console.log(`[ACCEPT] displayEnv.targets=`, displayEnv?.targets?.map(t => t.id));
    console.log(`[ACCEPT] state.visitedTargets=`, state.visitedTargets);

    state.env = displayEnv;

    // PRESERVE current state.droneConfigs - user's UI checkboxes are the source of truth
    // For fresh segmentation: user controls which drones are enabled via UI
    // For imported segments: UI was already synced to segment's droneConfigs on load
    // Either way, state.droneConfigs reflects what the user wants for the NEXT solve
    // Do NOT load from stored segment data (that would reset user's choices)

    // Log which drones are disabled going into new segment
    const disabledDrones = Object.entries(state.droneConfigs)
      .filter(([, cfg]) => cfg && !cfg.enabled)
      .map(([did]) => did);
    if (disabledDrones.length > 0) {
      console.log(`[ACCEPT] Preserving disabled drones for segment ${newSegIdx}: ${disabledDrones.join(", ")}`);
    }

    // Sync env with current droneConfigs (preserve user's choices)
    state.env.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));
    missionState.acceptedEnv = JSON.parse(JSON.stringify(displayEnv));
    missionState.acceptedEnv.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));

    console.log(`[ACCEPT] state.env.targets after assignment=`, state.env?.targets?.map(t => t.id));

    // Do NOT call initDroneConfigsFromEnv() - it would overwrite user's drone configs
    // The UI checkboxes should remain as the user set them
    updateSamWrappingClientSide();

    const unfrozen = segmentedImport.getUnfrozenTargets();
    appendDebugLine(`➡️ Segment ${newSegIdx}: ${segmentedImport.getAllTargets().length} total, ${unfrozen.length} to visit, frozen: ${state.visitedTargets.join(",") || "none"}`);
    console.log(`[ACCEPT-NEW] FINAL state.env.targets=`, state.env?.targets?.map(t => t.id));

    setMissionMode(MissionMode.IDLE, `segment ${newSegIdx} ready to solve`);
    console.log(`[acceptSolutionWithManager] EXIT (more segments). Mode: ${missionState.mode}`);
    drawEnvironment();
  } else {
    // All segments complete - ready to animate
    console.log("[ACCEPT] All segments complete - building final combined trajectory");

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

        console.log(`[ACCEPT] Final segment ${currentSegIdx} D${did}: full=${fullTraj.length}pts, prev=${prevTraj.length}pts, thisSegment=${segmentTraj.length}pts`);
      });

      missionReplay.replaceSegment(currentSegIdx, {
        ...finalSeg,
        solution: segmentSolution,
      });
    }

    const combinedRoutes = buildCombinedRoutesFromSegments();
    state.routes = combinedRoutes;
    Object.keys(combinedRoutes).forEach(did => {
      state.trajectoryVisible[did] = true;
    });

    // Mark ALL targets as visited (mission complete)
    state.visitedTargets = segmentedImport.getAllTargets().map(t => t.id);
    console.log(`[ACCEPT] All targets visited: ${state.visitedTargets.join(", ")}`);

    // Clear any pending cut markers
    state.pendingCutPositions = null;

    // Set env to full environment with all targets
    state.env = segmentedImport.getFullEnv();
    // PRESERVE user's droneConfigs - do NOT reset from env
    state.env.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));
    missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));
    missionState.acceptedEnv.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));

    setMissionMode(MissionMode.READY_TO_ANIMATE, "all segments accepted; ready to replay");
    appendDebugLine("✅ All segments accepted. Ready to Animate full mission.");
    console.log(`[acceptSolutionWithManager] EXIT (all segments complete). Mode: ${missionState.mode}`);
    drawEnvironment();
  }
}

/**
 * Discard draft solution - go back to previous state
 */
function discardDraftSolution() {
  // DEBUG: Log who called discard and from where
  console.log(`[discardDraftSolution] CALLED. Current mode: ${missionState.mode}`);
  console.trace("[discardDraftSolution] Stack trace:");

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
  if (missionReplay.getSegmentCount() === 0 && missionState.committedSegments.length === 0) {
    appendDebugLine("No committed segments to reset to");
    return;
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
  // NEW PATH: For SegmentedImportManager workflow
  // =====================================================
  if (segmentedImport.isActive()) {
    console.log("[resetMission] SegmentedImportManager workflow - resetting to segment 0 state");

    // Reset to segment 0 (just the first segment's truncated trajectory with C1 marker)
    const seg0 = missionReplay.getSegment(0);
    if (seg0 && seg0.solution && seg0.solution.routes) {
      state.routes = JSON.parse(JSON.stringify(seg0.solution.routes));
      Object.keys(state.routes).forEach(did => {
        state.trajectoryVisible[did] = true;
      });
      console.log("[resetMission] Loaded segment 0 routes");
    } else {
      state.routes = {};
      console.log("[resetMission] No segment 0 solution found");
    }

    // CRITICAL: Start animation with EMPTY visited targets
    // Targets get crossed during animation as drones actually pass them
    // The segment's visitedTargets are for the SOLVER (to exclude targets), not for animation state
    state.visitedTargets = [];
    console.log(`[resetMission] Starting animation with empty visitedTargets (targets crossed during animation)`);

    // Set pending cut positions for C1 marker
    const seg1 = missionReplay.getSegment(1);
    if (seg1 && seg1.cutPositions) {
      state.pendingCutPositions = JSON.parse(JSON.stringify(seg1.cutPositions));
      console.log("[resetMission] Set C1 marker positions:", state.pendingCutPositions);
    } else {
      state.pendingCutPositions = null;
    }

    // Reset segmented import manager to segment 0
    segmentedImport._currentSegmentIndex = 0;

    // Set env to display targets for segment 0
    state.env = segmentedImport.getEnvForDisplay();
    state.droneConfigs = JSON.parse(JSON.stringify(state.env.drone_configs || {}));
    state.env.drone_configs = state.droneConfigs;
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
    console.log(`[resetMission] segmentInfo workflow - cleared all solutions`);
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
  console.log(`[resetMission] Collected ${allTargetsFromSegments.length} targets from segments, existing snapshot has ${existingSnapshotTargetCount} targets`);

  // Only update initialEnvSnapshot if we collected MORE targets than it already has
  // (The original snapshot from first solve should have all targets)
  if (allTargetsFromSegments.length > existingSnapshotTargetCount && firstSegment && firstSegment.env) {
    const fullEnv = JSON.parse(JSON.stringify(firstSegment.env));
    fullEnv.targets = allTargetsFromSegments;
    state.initialEnvSnapshot = fullEnv;
    console.log(`[resetMission] initialEnvSnapshot UPDATED with ${allTargetsFromSegments.length} targets`);
  } else if (existingSnapshotTargetCount > 0) {
    console.log(`[resetMission] Keeping existing initialEnvSnapshot with ${existingSnapshotTargetCount} targets`);
  } else if (firstSegment && firstSegment.env) {
    // No existing snapshot and no better source - use segment targets
    const fullEnv = JSON.parse(JSON.stringify(firstSegment.env));
    fullEnv.targets = allTargetsFromSegments;
    state.initialEnvSnapshot = fullEnv;
    console.log(`[resetMission] initialEnvSnapshot created with ${allTargetsFromSegments.length} targets`);
  }

  // Set state.env from segment 0 - show only segment 0's targets after Reset
  // During animation, targets will appear as the drone enters each segment
  if (firstSegment && firstSegment.env) {
    state.env = JSON.parse(JSON.stringify(firstSegment.env));
    missionState.acceptedEnv = JSON.parse(JSON.stringify(firstSegment.env));

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
        console.log(`[resetMission] Fresh seg: filtered to ${state.env.targets.length} route targets: ${[...seg0RouteTargets].join(",")}`);
      } else {
        console.log(`[resetMission] Fresh seg: no route targets found, using all ${state.env.targets.length} targets`);
      }
    } else {
      const seg0Targets = (firstSegment.env.targets || []).map(t => t.id);
      console.log(`[resetMission] Using segment 0 env with ${seg0Targets.length} targets: ${seg0Targets.join(",")}`);
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
    console.log(`[resetMission] segmentInfo workflow - cleared state.routes`);
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
    console.log(`[invalidateMission] Preserving frozen trajectories (${missionState.committedSegments.length} committed segments)`);
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
          console.log("R key ignored - focus is on input element:", activeTag);
          return;
        }

        console.log("R key pressed, mode =", missionState.mode);
        appendDebugLine("R key pressed - attempting reset...");

        // Use state machine permissions
        const perms = getUiPermissions();
        if (perms.canReset) {
          console.log("Calling resetMission()");
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

    console.log(`[calcVisited] Cut position [${cutPos[0].toFixed(2)}, ${cutPos[1].toFixed(2)}] closest to traj point ${cutPointIdx} (dist=${minCutDist.toFixed(2)})`);

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
          console.log(`[calcVisited] ${wp} at trajIdx=${targetIdx} < cutIdx=${cutPointIdx} -> VISITED`);
        } else {
          console.log(`[calcVisited] ${wp} at trajIdx=${targetIdx} >= cutIdx=${cutPointIdx} -> NOT visited`);
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
function drawEnvironment() {
  const canvas = $("env-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  // Debug: Log visited targets every 30 frames (about once per second at 30fps)
  _drawEnvFrameCount++;
  if (_drawEnvFrameCount % 60 === 1 && state.visitedTargets.length > 0) {
    console.log(`[drawEnv] visitedTargets (${state.visitedTargets.length}):`, state.visitedTargets);
  }

  ctx.clearRect(0, 0, canvas.width, canvas.height);

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
    console.log(`[drawEnv] Drawing ${targets.length} targets: ${targets.map(t=>t.id).join(",")}`);
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
  console.log("drawEnvironment: wrappedPolygons count =", state.wrappedPolygons ? state.wrappedPolygons.length : 0);
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
    Object.entries(state.routes || {}).forEach(([did, info]) => {
      const trajLen = info.trajectory ? info.trajectory.length : 0;
      const hasFullTraj = info._fullTrajectory ? info._fullTrajectory.length : 0;
      if (trajLen > 0) {
        const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
        const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
        console.log(`[drawEnv] D${did}: trajectory=${trajLen}pts ${firstPt}→${lastPt}, _fullTrajectory=${hasFullTraj}pts`);
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
      // Draw if animating OR frozen at checkpoint OR drone has a valid position
      if (!droneState.animating && !state.checkpoint?.active && droneState.distanceTraveled === undefined) return;
      if (!state.trajectoryVisible[did]) return;

      const routeInfo = state.routes[did];
      if (!routeInfo) return;

      // Use trajectory if available (SAM-avoiding path), otherwise fall back to route waypoints
      const trajectory = routeInfo.trajectory || [];
      const route = routeInfo.route || [];
      const progress = droneState.progress;

      let droneX, droneY;

      // If frozen at checkpoint, draw drone at the splitPoint (end of prefix)
      if (state.checkpoint?.active && state.checkpoint.segments[did]?.splitPoint) {
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

  // Draw cut markers from missionReplay segments
  // Helper: check if position is at an airport (don't draw cut markers at airports)
  const isAtAirport = (x, y) => {
    const airports = state.env?.airports || [];
    const threshold = 5.0; // Distance threshold to consider "at" airport
    return airports.some(a => {
      const dx = a.x - x;
      const dy = a.y - y;
      return Math.sqrt(dx * dx + dy * dy) < threshold;
    });
  };

  // Draw cut markers
  // For segmented import: use state.pendingCutPositions ONLY (set by Accept)
  // For fresh segmentation: use missionReplay cutPositions + pendingCutPositions
  console.log(`[drawEnv] VERSION: 2024-12-30-targetfix-v3`);
  const currentSegIdx = missionReplay.getCurrentSegmentIndex();

  if (segmentedImport.isActive()) {
    // SEGMENTED IMPORT: Draw ALL markers from MissionReplay segments + current pending marker
    const currentSegIdx = segmentedImport.getCurrentSegmentIndex();

    // Draw markers from all segments that have cutPositions (segments 1+)
    console.log(`[drawEnv-IMPORT] currentSegIdx=${currentSegIdx}, checking segments 1 to ${currentSegIdx + 1}`);
    for (let segIdx = 1; segIdx <= currentSegIdx + 1; segIdx++) {
      const seg = missionReplay.getSegment(segIdx);
      console.log(`[drawEnv-IMPORT] segIdx=${segIdx}, seg.cutPositions=`, seg?.cutPositions);
      if (seg && seg.cutPositions && Object.keys(seg.cutPositions).length > 0) {
        console.log(`[drawEnv-IMPORT] Drawing marker C${segIdx} at:`, seg.cutPositions);
        const lostDrones = seg.lostDrones || [];
        Object.entries(seg.cutPositions).forEach(([did, pos]) => {
          if (!pos || pos.length !== 2) return;
          if (isAtAirport(pos[0], pos[1])) return;
          const [mx, my] = w2c(pos[0], pos[1]);
          const isLost = lostDrones.includes(did);
          drawCutMarker(ctx, mx, my, `C${segIdx}`, isLost);
        });
      }
    }

    // Also draw the current pending marker (if we just pressed Accept but haven't solved yet)
    if (state.pendingCutPositions && Object.keys(state.pendingCutPositions).length > 0) {
      const nextSegIdx = currentSegIdx;
      Object.entries(state.pendingCutPositions).forEach(([did, pos]) => {
        if (!pos || pos.length !== 2) return;
        if (isAtAirport(pos[0], pos[1])) return;
        const [mx, my] = w2c(pos[0], pos[1]);
        // Check drone's CURRENT enabled state for pending markers
        const cfg = state.droneConfigs[did];
        const isLost = cfg && cfg.enabled === false;
        drawCutMarker(ctx, mx, my, `C${nextSegIdx}`, isLost);
      });
    }
  } else {
    // FRESH SEGMENTATION: Use missionReplay cutPositions
    const maxSegToShow = currentSegIdx + 2;
    console.log(`[drawEnv] currentSegIdx=${currentSegIdx}, checking cut markers for segments 1 to ${maxSegToShow - 1}`);

    for (let segIdx = 1; segIdx < maxSegToShow; segIdx++) {
      const seg = missionReplay.getSegment(segIdx);
      const prevSeg = missionReplay.getSegment(segIdx - 1);
      const prevHasSolution = prevSeg && Object.keys(prevSeg.solution?.routes || {}).length > 0;

      console.log(`[drawEnv] segIdx=${segIdx}, prevHasSolution=${prevHasSolution}`);

      if (seg && seg.cutPositions && prevHasSolution) {
        const lostDrones = seg.lostDrones || [];
        Object.entries(seg.cutPositions).forEach(([did, pos]) => {
          if (!pos || pos.length !== 2) return;
          if (isAtAirport(pos[0], pos[1])) return;
          const [mx, my] = w2c(pos[0], pos[1]);
          const isLost = lostDrones.includes(did);
          drawCutMarker(ctx, mx, my, `C${segIdx}`, isLost);
        });
      }
    }

    // Draw pending cut marker (fresh segmentation - after pressing C but before Accept)
    if (state.pendingCutPositions && Object.keys(state.pendingCutPositions).length > 0) {
      const nextSegIdx = missionReplay.getSegmentCount();
      Object.entries(state.pendingCutPositions).forEach(([did, pos]) => {
        if (!pos || pos.length !== 2) return;
        if (isAtAirport(pos[0], pos[1])) return;
        const [mx, my] = w2c(pos[0], pos[1]);
        // Check drone's CURRENT enabled state for pending markers
        const cfg = state.droneConfigs[did];
        const isLost = cfg && cfg.enabled === false;
        drawCutMarker(ctx, mx, my, `C${nextSegIdx}`, isLost);
      });
    }
  }

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

    // Always capture current drone configs into env snapshot
    const envSnapshot = JSON.parse(JSON.stringify(state.env));
    envSnapshot.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs || envSnapshot.drone_configs || {}));

    // Debug: Log current drone configs being exported
    console.log(`[EXPORT] state.droneConfigs:`, state.droneConfigs);
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
      // MISSION export - each segment stores ALL 5 items:
      // 1. frozenTargets - targets visited before this segment
      // 2. frozenTrajectories - locked trajectory portions from previous segments
      // 3. activeTargets - targets to solve in this segment
      // 4. droneConfigs - which drones enabled, start/end airports, fuel
      // 5. startPositions/endAirports - where drones start (airport or cut) and end (airport)
      segmentCount = segments.length;

      // Get base drone configs from segment 0 - use actual values, don't force all enabled
      const seg0Env = segments[0]?.env || {};
      const baseDroneConfigs = JSON.parse(JSON.stringify(seg0Env.drone_configs || state.droneConfigs || {}));

      // Track cumulative state as we iterate through segments
      const cumulativeLost = new Set();
      const cumulativeFrozenTargets = [];
      const cumulativeFrozenTrajectories = {};  // { droneId: [[x,y], ...] }

      exportObj = {
        schema: "isr_env_v2",  // New schema version with complete segment data
        is_segmented: true,
        segment_count: segmentCount,
        drone_configs: baseDroneConfigs,  // Top-level: ORIGINAL configs (all drones enabled)
        airports: JSON.parse(JSON.stringify(seg0Env.airports || [])),
        sams: JSON.parse(JSON.stringify(seg0Env.sams || [])),
        segments: segments.map((s, idx) => {
          // 1. Frozen targets - targets visited BEFORE this segment
          const frozenTargets = [...cumulativeFrozenTargets];

          // 2. Frozen trajectories - trajectory portions from segments 0 to idx-1
          const frozenTrajectories = JSON.parse(JSON.stringify(cumulativeFrozenTrajectories));

          // 3. Active targets - targets in this segment's env (to be solved)
          const activeTargets = (s.env?.targets || []).map(t => t.id);

          // 4. Drone configs for THIS segment
          // Use the segment's actual drone_configs if available, otherwise derive from cumulative lost
          let segmentDroneConfigs;
          if (s.env?.drone_configs) {
            // Use segment's own drone_configs (source of truth)
            segmentDroneConfigs = JSON.parse(JSON.stringify(s.env.drone_configs));
          } else {
            // Fallback: derive from baseDroneConfigs + cumulativeLost
            segmentDroneConfigs = JSON.parse(JSON.stringify(baseDroneConfigs));
            cumulativeLost.forEach(did => {
              if (segmentDroneConfigs[did]) {
                segmentDroneConfigs[did].enabled = false;
              }
            });
          }

          // 5. Start positions (cutPositions for seg>0, airports for seg 0) and end airports
          const startPositions = idx === 0 ? null : (s.cutPositions || null);

          // Build the segment export object
          const segmentExport = {
            index: s.index,
            // Complete segment state - no derivation needed on import
            frozenTargets: frozenTargets,
            frozenTrajectories: frozenTrajectories,
            activeTargets: activeTargets,
            droneConfigs: segmentDroneConfigs,
            startPositions: startPositions,  // null for seg-0, cutPositions for seg>0
            // Also keep original env for compatibility
            env: s.env,
            cutDistance: s.cutDistance || null,
            cutPositions: s.cutPositions || null,
            lostDrones: (s.lostDrones && s.lostDrones.length > 0) ? [...s.lostDrones] : null,
            visitedTargets: s.visitedTargets ? [...s.visitedTargets] : null,
          };

          // Update cumulative state for NEXT segment
          // Add this segment's visited targets to frozen targets for next segment
          if (s.visitedTargets) {
            s.visitedTargets.forEach(t => {
              if (!cumulativeFrozenTargets.includes(t)) {
                cumulativeFrozenTargets.push(t);
              }
            });
          }
          // Add this segment's trajectories to frozen trajectories for next segment
          if (s.solution?.routes) {
            Object.entries(s.solution.routes).forEach(([did, routeData]) => {
              const traj = routeData.trajectory || [];
              if (traj.length > 0) {
                if (!cumulativeFrozenTrajectories[did]) {
                  cumulativeFrozenTrajectories[did] = [];
                }
                // Append this segment's trajectory
                cumulativeFrozenTrajectories[did] = [
                  ...cumulativeFrozenTrajectories[did],
                  ...traj
                ];
              }
            });
          }
          // Add lost drones to cumulative set (affects NEXT segment)
          (s.lostDrones || []).forEach(did => cumulativeLost.add(did));

          return segmentExport;
        }),
      };
      appendDebugLine(`   ${segmentCount} segments with complete state (v2 schema)`);
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

function initDroneConfigsFromEnv() {
  const airports = (state.env && state.env.airports) || [];
  const airportIds = airports.map((a) => String(a.id));
  const defaultAirport = airportIds[0] || "A1";

  // Check if drone_configs are saved in the environment
  const savedConfigs = (state.env && state.env.drone_configs) || {};

  // DEBUG: Log what we're reading
  const enabledStates = Object.entries(savedConfigs).map(([did, cfg]) => `D${did}:${cfg?.enabled}`).join(", ");
  console.log(`[initDroneConfigsFromEnv] savedConfigs enabled states: ${enabledStates || "(empty)"}`);

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
  console.log("📊 updateStatsFromRoutes called");
  console.log("📊 state.routes:", state.routes);

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
    console.log(`📊 Drone ${did}: points=${points}, distance=${distance}, route=`, route);

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
 * Check if JSON data is a segmented mission file (vs plain environment)
 * Detection: data.segments is array and data.segments[0].env exists
 */
function isSegmentedMissionJson(data, filename = "") {
  const hasSegmentsArray = data && Array.isArray(data.segments) && data.segments.length > 0 && data.segments[0].env;
  return hasSegmentsArray && isSegmentedByFilename(filename);
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

  console.log(`[isSegmentInfoJson] hasSegmentCuts=${hasSegmentCuts}, hasFilenamePattern=${hasFilenamePattern}, filename=${filename}`);
  return hasSegmentCuts || hasFilenamePattern;
}

/**
 * Load a segmented mission from segmentInfo format (replay format)
 * Uses SegmentedImportManager as single source of truth
 */
function loadSegmentInfoFromJson(data, filename = "") {
  console.log(`[loadSegmentInfoFromJson] CALLED - using SegmentedImportManager`);
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
  missionState.committedSegments = [];
  missionState.draftSolution = null;
  state.visitedTargets = [];  // Segment 0 starts with no visited targets
  state.importedSegmentCuts = null;  // We use segmentedImport now, not this
  state.wrappedPolygons = [];  // Clear ghost SAM boundaries from previous imports

  // 4. Create MissionReplay segments (placeholder solutions to be filled on Solve)
  const totalSegments = segmentedImport.getTotalSegments();
  appendDebugLine(`📊 Creating ${totalSegments} segments`);

  for (let i = 0; i < totalSegments; i++) {
    const cutDistance = segmentedImport.getCutDistanceForSegment(i);
    const lostDrones = segmentedImport.getLostDronesForSegment(i);
    // Don't load cutPositions on import - they're calculated fresh when you Accept
    // const cutPositions = segmentedImport.getCutPositionForSegment(i);

    // Log segment info
    const visited = segmentedImport.getVisitedTargetsForSegment(i);
    appendDebugLine(`   Segment ${i}: cutDist=${cutDistance.toFixed(1)}, visited=[${visited.join(",")}], lostDrones=[${lostDrones.join(",")}]`);

    missionReplay.addSegment({
      solution: { routes: {}, sequences: {} },
      env: fullEnv,  // All segments see all targets; filtering happens in solver
      cutDistance: i === 0 ? null : cutDistance,
      cutPositions: null,  // Don't copy cutPositions on import - they're set when you Accept
      isCheckpointReplan: i > 0,
      lostDrones: lostDrones.length > 0 ? lostDrones : null,  // Drones killed at this checkpoint
    });
  }

  // 5. Reset to start
  missionReplay.resetToStart();
  missionState.currentBuildSegmentIndex = 0;

  // 6. NO cut marker on import - markers appear after you Accept solutions
  state.pendingCutPositions = null;
  state.pendingLostDrones = [];

  appendDebugLine(`✅ Loaded ${segmentedImport.getTotalSegments()} segments. Solve segment 0 first.`);
  console.log(`[loadSegmentInfoFromJson] segmentedImport debug:`, segmentedImport.getDebugInfo());

  // Update UI checkboxes to reflect segment 0's droneConfigs
  initDroneConfigsFromEnv();
  updateAirportDropdowns();

  setMissionMode(MissionMode.IDLE, "segmentInfo loaded - solve segment 0");
}

/**
 * Load a segmented mission from JSON - strips solutions, keeps only env + cut distances
 * User must solve each segment sequentially before animating
 * NOW USES SegmentedImportManager for clean state management
 */
function loadSegmentedMissionFromJson(data, filename = "") {
  console.log(`[loadSegmentedMissionFromJson] CALLED - using SegmentedImportManager`);
  appendDebugLine(`📥 Loading segmented mission from ${filename}`);

  // 1. Load into the segmented import manager (single source of truth)
  segmentedImport.loadFromSegmentsArray(data);

  // 2. Set up global state - show only segment 0's targets on import
  const displayEnv = segmentedImport.getEnvForDisplay();  // Only segment 0's targets
  const fullEnv = segmentedImport.getFullEnv();           // All targets for snapshot
  state.env = displayEnv;
  state.droneConfigs = JSON.parse(JSON.stringify(displayEnv.drone_configs || {}));
  state.env.drone_configs = state.droneConfigs;

  // Store initial snapshot (ALL targets - never overwrite this)
  state.initialEnvSnapshot = JSON.parse(JSON.stringify(fullEnv));
  console.log(`[loadSegmentedMissionFromJson] Displaying ${displayEnv.targets.length} targets: ${displayEnv.targets.map(t=>t.id).join(",")}`);
  console.log(`[loadSegmentedMissionFromJson] initialEnvSnapshot has ALL ${fullEnv.targets.length} targets: ${fullEnv.targets.map(t=>t.id).join(",")}`);

  // 3. Reset mission state
  missionReplay.clear();
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

  for (let i = 0; i < totalSegments; i++) {
    const cutDistance = segmentedImport.getCutDistanceForSegment(i);
    const lostDrones = segmentedImport.getLostDronesForSegment(i);
    // Don't load cutPositions on import - they're calculated fresh when you Accept
    // const cutPositions = segmentedImport.getCutPositionForSegment(i);

    // Log segment info
    const visited = segmentedImport.getVisitedTargetsForSegment(i);
    appendDebugLine(`   Segment ${i}: cutDist=${cutDistance.toFixed(1)}, visited=[${visited.join(",")}], lostDrones=[${lostDrones.join(",")}]`);

    missionReplay.addSegment({
      solution: { routes: {}, sequences: {} },
      env: fullEnv,  // All segments see all targets; filtering happens in solver
      cutDistance: i === 0 ? null : cutDistance,
      cutPositions: null,  // Don't copy cutPositions on import - they're set when you Accept
      isCheckpointReplan: i > 0,
      lostDrones: lostDrones.length > 0 ? lostDrones : null,  // Drones killed at this checkpoint
    });
  }

  // 5. Reset to start
  missionReplay.resetToStart();
  missionState.currentBuildSegmentIndex = 0;

  // 6. NO cut marker on import - it appears after Solve
  state.pendingCutPositions = null;
  state.pendingLostDrones = [];

  appendDebugLine(`✅ Loaded ${segmentedImport.getTotalSegments()} segments. Solve segment 0 first.`);
  console.log(`[loadSegmentedMissionFromJson] segmentedImport debug:`, segmentedImport.getDebugInfo());

  // Update UI checkboxes to reflect segment 0's droneConfigs
  initDroneConfigsFromEnv();
  updateAirportDropdowns();

  // Set mode to IDLE - user must solve segment 0 first
  setMissionMode(MissionMode.IDLE, `segmented mission loaded - solve segment 0`);
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
  state.wrappedPolygons = [];  // Clear ghost SAM boundaries from previous imports
  missionState.draftSolution = null;
  missionState.checkpointSource = null;
  missionState.segmentedMission = null;
  missionState.currentBuildSegmentIndex = 0;

  // --- Case 1: SegmentInfo format (type: "segmented" with segmentInfo.segmentCuts) ---
  // Check this FIRST because some files may have both segments array AND segmentInfo
  // IMPORTANT: Now requires filename pattern _Nx_ where x > 1 to be treated as segmented
  const segCountFromFilename = getSegmentCountFromFilename(filename);
  console.log(`[loadFromJson] Checking formats: filename=${filename}, segCountFromFilename=${segCountFromFilename}`);
  console.log(`[loadFromJson] isSegmentInfoJson=${isSegmentInfoJson(data, filename)}, isSegmentedMissionJson=${isSegmentedMissionJson(data, filename)}`);

  if (isSegmentInfoJson(data, filename)) {
    console.log(`[loadFromJson] MATCHED: SegmentInfo format (filename has _N${segCountFromFilename}_)`);
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
  console.log(`[IMPORT] state.env.drone_configs before init:`, state.env?.drone_configs);
  initDroneConfigsFromEnv();
  console.log(`[IMPORT] state.droneConfigs after init:`, state.droneConfigs);
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
        console.error("Error parsing JSON:", err);
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
    console.log(`[getFrozenTrajectoryCounts] Got cutDistance from draftSolution: ${cutDistance?.toFixed(1) || "null"}`);
  } else if (state.pendingCutDistance) {
    // Fallback to pending cut distance
    cutDistance = state.pendingCutDistance;
  }

  if (!cutDistance || cutDistance <= 0) {
    return counts;
  }

  console.log(`[getFrozenTrajectoryCounts] Using cutDistance=${cutDistance.toFixed(1)}`);


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
    console.log(`[getFrozenTrajectoryCounts] D${did}: cutDist=${cutDistance.toFixed(1)}, traj=${traj.length}pts, frozen=${frozenCount}pts`);
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
  if (!cutDistance || cutDistance <= 0) return [];

  const visitedTargets = new Set();

  // Get all target positions
  const allTargets = segmentedImport.getAllTargets();
  const targetPositions = {};
  allTargets.forEach(t => {
    targetPositions[t.id] = [t.x, t.y];
  });

  // For each drone, find which targets are visited within the cut distance
  Object.entries(routes || {}).forEach(([droneId, routeData]) => {
    const traj = routeData.trajectory || [];
    const sequence = routeData.route || routeData.sequence || [];

    if (traj.length < 2) return;

    // Calculate cumulative distance at each trajectory point
    const distances = [0];
    for (let i = 1; i < traj.length; i++) {
      const dx = traj[i][0] - traj[i-1][0];
      const dy = traj[i][1] - traj[i-1][1];
      distances.push(distances[i-1] + Math.sqrt(dx * dx + dy * dy));
    }

    // For each target in the sequence, check if it's visited within the cut distance
    sequence.forEach(nodeId => {
      // Skip airports and synthetic starts
      if (nodeId.startsWith('A') || nodeId.includes('_START')) return;

      const targetPos = targetPositions[nodeId];
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
            visitedTargets.add(nodeId);
          }
          break;
        }
      }
    });
  });

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

  // For each drone, find which targets are visited within the frozen trajectory portion
  Object.entries(state.routes || {}).forEach(([droneId, routeData]) => {
    const traj = routeData.trajectory || [];
    const sequence = routeData.route || routeData.sequence || [];

    if (traj.length < 2) return;

    // Calculate cumulative distance at each trajectory point
    const distances = [0];
    for (let i = 1; i < traj.length; i++) {
      const dx = traj[i][0] - traj[i-1][0];
      const dy = traj[i][1] - traj[i-1][1];
      distances.push(distances[i-1] + Math.sqrt(dx * dx + dy * dy));
    }

    // Get all target positions
    const allTargets = segmentedImport.getAllTargets();
    const targetPositions = {};
    allTargets.forEach(t => {
      targetPositions[t.id] = [t.x, t.y];
    });

    // For each target in the sequence, check if it's visited within the frozen distance
    sequence.forEach(nodeId => {
      // Skip airports and synthetic starts
      if (nodeId.startsWith('A') || nodeId.includes('_START')) return;

      const targetPos = targetPositions[nodeId];
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
            frozenTargets.add(nodeId);
            console.log(`[getActualFrozenTargets] D${droneId} Target ${nodeId} at traj[${i}] (dist=${distances[i].toFixed(1)}) is FROZEN (cutDist=${cutDistance.toFixed(1)})`);
          } else {
            console.log(`[getActualFrozenTargets] D${droneId} Target ${nodeId} at traj[${i}] (dist=${distances[i].toFixed(1)}) is UNFROZEN (cutDist=${cutDistance.toFixed(1)})`);
          }
          break;
        }
      }
    });
  });

  console.log(`[getActualFrozenTargets] Final frozen targets: ${Array.from(frozenTargets).join(", ")}`);
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

  console.log(`[getUnfrozenEnv] state.visitedTargets=${(state.visitedTargets || []).join(",")}`);
  console.log(`[getUnfrozenEnv] frozenTargets=${frozenTargets.join(",")}`);
  console.log(`[getUnfrozenEnv] Filtered ${frozenSet.size} frozen targets, ${env.targets.length} remain: ${env.targets.map(t => t.id).join(",")}`);
  console.log(`[getUnfrozenEnv] drone_configs:`, Object.entries(droneConfigs).filter(([,c]) => c.enabled).map(([d,c]) => `D${d}:${c.start_airport}→${c.end_airport}`).join(", "));
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
    console.log(`[mergeWithFrozenRoutes] Preserved existing route for D${did} with ${routeData.trajectory?.length || 0}pts`);
  });

  // Then process routes from the optimizer, merging with frozen portions
  Object.entries(optimizedRoutes || {}).forEach(([did, optimizedData]) => {
    const frozenCount = frozenCounts[did] || 0;
    const originalRoute = state.routes[did];

    if (frozenCount === 0 || !originalRoute) {
      // No frozen portion - use optimized route directly
      mergedRoutes[did] = JSON.parse(JSON.stringify(optimizedData));
      console.log(`[mergeWithFrozenRoutes] D${did}: no frozen portion, using optimized (${optimizedData.trajectory?.length || 0}pts)`);
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

      console.log(`[mergeWithFrozenRoutes] D${did}: frozen=${frozenCount}pts + optimized=${optimizedTraj.length}pts = merged=${mergedTraj.length}pts`);
    }
  });

  console.log(`[mergeWithFrozenRoutes] Final routes: ${Object.keys(mergedRoutes).map(d => `D${d}:${mergedRoutes[d].trajectory?.length || 0}pts`).join(", ")}`);
  return mergedRoutes;
}

// ----------------------------------------------------
// Optimization buttons
// ----------------------------------------------------
function attachOptimizationHandlers() {
  const btnInsert = $("btn-optimize-insert");
  const btnSwap = $("btn-optimize-swap");
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
        console.log(`[Insert Missed] segmentedFrozen=${segmentedFrozen}, checkpointFrozen=${checkpointFrozen}, hasFrozen=${hasFrozen}`);
        console.log(`[Insert Missed] actualFrozen=${actualFrozenTargets.join(",")}, visitedTargets=${(state.visitedTargets || []).join(",")}`);

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
              console.log(`[Insert Missed] Merged frozen + optimized routes`);
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
        btnInsert.textContent = "Insert Missed";
      }
    });
  }

  if (btnSwap) {
    console.log("Swap button handler attached");
    btnSwap.addEventListener("click", async () => {
      console.log("Swap button clicked!", state.routes);
      if (!state.routes || Object.keys(state.routes).length === 0) {
        appendDebugLine("No routes to optimize. Run planner first.");
        return;
      }
      appendDebugLine("Running Trajectory Swap optimization (auto-iterate mode)...");
      btnSwap.disabled = true;
      btnSwap.textContent = "Optimizing...";

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
        console.log(`[Trajectory Swap] segmentedFrozen=${segmentedFrozen}, checkpointFrozen=${checkpointFrozen}, hasFrozen=${hasFrozen}`);
        console.log(`[Trajectory Swap] actualFrozen=${actualFrozenTargets.join(",")}, visitedTargets=${(state.visitedTargets || []).join(",")}`);

        // Debug: Log what we're sending to the backend
        console.log("=== SWAP CLOSER REQUEST ===");
        console.log("Current routes:", state.routes);
        console.log("Distance matrix available:", !!state.distanceMatrix);
        if (state.distanceMatrix) {
          console.log("Distance matrix labels:", state.distanceMatrix.labels);
        }

        const resp = await fetch("/api/trajectory_swap_optimize", {
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

        // Debug: Log the complete response
        console.log("=== SWAP CLOSER RESPONSE ===");
        console.log("Success:", data.success);
        console.log("Swaps made:", data.swaps_made);
        console.log("Number of swaps:", (data.swaps_made || []).length);
        if (data.swaps_made && data.swaps_made.length > 0) {
          console.log("Swap details:");
          data.swaps_made.forEach(swap => {
            console.log(`  ${swap.target}: D${swap.from_drone} -> D${swap.to_drone}`);
          });
        }
        console.log("Routes in response:", Object.keys(data.routes || {}));
        console.log("Full response data:", data);

        if (data.success) {
          const swaps = data.swaps_made || [];
          const iterations = data.iterations || 1;
          const bestIteration = data.best_iteration || iterations;
          const bestDistance = data.best_distance || 0;
          const cycleDetected = data.cycle_detected || false;
          const converged = data.converged || false;

          if (swaps.length > 0) {
            // FROZEN TRAJECTORY RESPECT: Save frozen trajectories BEFORE updating state.routes
            // The optimizer returns routes without trajectories, so we need to preserve
            // the frozen portions from the current state.routes and let regenerateTrajectories
            // handle the merge later
            let savedFrozenTrajectories = null;
            if (hasFrozen) {
              const frozenCounts = getFrozenTrajectoryCounts();
              savedFrozenTrajectories = {};
              Object.entries(state.routes || {}).forEach(([did, routeData]) => {
                const frozenCount = frozenCounts[did] || 0;
                if (frozenCount > 0 && routeData.trajectory && routeData.trajectory.length >= frozenCount) {
                  savedFrozenTrajectories[did] = routeData.trajectory.slice(0, frozenCount);
                  console.log(`[Trajectory Swap] Saved ${frozenCount}pts frozen trajectory for D${did}`);
                }
              });
            }

            // Deep clone optimizer routes (has route sequences but no/empty trajectories)
            state.routes = JSON.parse(JSON.stringify(data.routes));

            // Restore frozen trajectories so regenerateTrajectories can use them
            if (savedFrozenTrajectories) {
              Object.entries(savedFrozenTrajectories).forEach(([did, frozenTraj]) => {
                if (state.routes[did]) {
                  state.routes[did].trajectory = frozenTraj;
                  console.log(`[Trajectory Swap] Restored frozen trajectory for D${did}: ${frozenTraj.length}pts`);
                }
              });
            }

            // Deep clone sequences to avoid read-only property errors
            state.sequences = JSON.parse(JSON.stringify(data.sequences || {}));

            // Update sequence display
            for (const [did, seq] of Object.entries(state.sequences)) {
              state.sequences[did] = seq;
            }

            // Show summary of auto-iteration
            if (cycleDetected) {
              appendDebugLine(`Swap optimization: Cycle detected after ${iterations} iterations.`);
              appendDebugLine(`  Using best solution from iteration ${bestIteration} (distance: ${bestDistance.toFixed(1)})`);
            } else if (converged) {
              appendDebugLine(`Swap optimization: Converged after ${iterations} iterations.`);
            } else {
              appendDebugLine(`Swap optimization: Completed ${iterations} iterations.`);
            }

            appendDebugLine(`  Total swaps: ${swaps.length}, Best distance: ${bestDistance.toFixed(1)}`);

            // Show individual swaps (limit to first 10 if many)
            const swapsToShow = swaps.slice(0, 10);
            swapsToShow.forEach(swap => {
              const iter = swap.iteration ? ` (iter ${swap.iteration})` : '';
              appendDebugLine(`  ${swap.target}: D${swap.from_drone} -> D${swap.to_drone}${iter}`);
            });
            if (swaps.length > 10) {
              appendDebugLine(`  ... and ${swaps.length - 10} more swaps`);
            }

            // Recalculate trajectories for modified routes
            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();

            // Update the committed segment so Reset preserves optimizer changes
            missionReplay.updateCurrentSegmentSolution(state.routes, state.sequences);
          } else {
            appendDebugLine(`Swap optimization: No beneficial swaps found (${iterations} iteration${iterations > 1 ? 's' : ''}).`);
          }
        } else {
          appendDebugLine("Swap optimization error: " + (data.error || "Unknown error"));
        }
      } catch (err) {
        appendDebugLine("Swap optimization error: " + err.message);
      } finally {
        btnSwap.disabled = false;
        btnSwap.textContent = "Swap Closer";
      }
    });
  }

  if (btnCrossRemove) {
    console.log("Cross Remove button handler attached");
    btnCrossRemove.addEventListener("click", async () => {
      console.log("Cross Remove button clicked!", state.routes);
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
        console.log(`[Crossing Removal] segmentedFrozen=${segmentedFrozen}, checkpointFrozen=${checkpointFrozen}, hasFrozen=${hasFrozen}`);
        console.log(`[Crossing Removal] actualFrozen=${actualFrozenTargets.join(",")}, visitedTargets=${(state.visitedTargets || []).join(",")}`);

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
              console.log(`[Crossing Removal] Merged frozen + optimized routes`);
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
        btnCrossRemove.textContent = "Cross Remove";
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
          console.log(`[regenerateTrajectories] Added synthetic start ${nodeId} from draftSolution: (${seg.splitPoint[0].toFixed(1)}, ${seg.splitPoint[1].toFixed(1)})`);
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
        console.log(`[regenerateTrajectories] Added synthetic start ${nodeId} from frozen trajectory: (${lastFrozenPt[0].toFixed(1)}, ${lastFrozenPt[1].toFixed(1)})`);
      }
    });
  }

  for (const [did, routeData] of Object.entries(state.routes)) {
    if (!routeData.route || routeData.route.length < 2) {
      console.log(`[regenerateTrajectories] D${did}: skipping (route length=${routeData.route?.length || 0})`);
      continue;
    }

    try {
      // If we have frozen trajectory, we need to preserve it
      const frozenCount = frozenCounts[did] || 0;
      const frozenTraj = hasFrozen && frozenCount > 0
        ? routeData.trajectory?.slice(0, frozenCount)
        : null;

      console.log(`[regenerateTrajectories] D${did}: route=${JSON.stringify(routeData.route)}, frozenCount=${frozenCount}, frozenTraj=${frozenTraj?.length || 0}pts, hasFrozen=${hasFrozen}`);

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
      console.log(`[regenerateTrajectories] D${did}: API response success=${data.success}, trajectory.length=${data.trajectory?.length || 0}`);

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

          console.log(`[regenerateTrajectories] D${did}: preserved ${frozenTraj.length}pt frozen, appended ${isDuplicate ? newTraj.length - 1 : newTraj.length}pt new (isDuplicate=${isDuplicate})`);
        } else {
          // No frozen trajectory - use regenerated directly
          state.routes[did].trajectory = data.trajectory;
        }
        state.routes[did].distance = data.distance;
        state.routes[did].points = data.points;
      }
    } catch (err) {
      console.error(`Failed to regenerate trajectory for D${did}:`, err);
    }
  }
}

// ----------------------------------------------------
// Planner integration
// ----------------------------------------------------
async function fetchWrappedPolygons(recalculate = false) {
  console.log("fetchWrappedPolygons called, recalculate =", recalculate, "sams =", state.env?.sams?.length || 0);

  // Abort any pending wrapping request
  if (state.wrappingAbortController) {
    console.log("fetchWrappedPolygons: aborting previous request");
    state.wrappingAbortController.abort();
  }

  // Create new abort controller for this request
  state.wrappingAbortController = new AbortController();
  const signal = state.wrappingAbortController.signal;

  try {
    let resp, data;

    if (recalculate && state.env && state.env.sams && state.env.sams.length > 0) {
      // Recalculate wrapping with current SAM positions
      console.log("fetchWrappedPolygons: calling POST /api/calculate_wrapping with", state.env.sams.length, "SAMs");
      resp = await fetch("/api/calculate_wrapping", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ sams: state.env.sams }),
        signal: signal
      });
      data = await resp.json();
      console.log("fetchWrappedPolygons: response received", data);
    } else {
      // Get cached wrapping from last solve
      console.log("fetchWrappedPolygons: calling GET /api/wrapped_polygons (cached)");
      resp = await fetch("/api/wrapped_polygons", { signal: signal });
      data = await resp.json();
    }

    if (data && data.success && data.polygons) {
      console.log("fetchWrappedPolygons: received", data.polygons.length, "polygons");
      state.wrappedPolygons = data.polygons;
    } else {
      console.log("fetchWrappedPolygons: no polygons in response");
      state.wrappedPolygons = [];
    }
  } catch (err) {
    if (err.name === 'AbortError') {
      console.log("fetchWrappedPolygons: request was aborted (this is ok)");
      return; // Don't clear polygons on abort
    }
    console.error("fetchWrappedPolygons error:", err);
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
    console.log(`[saveInitialEnvSnapshot] Saving with ${targetIds.length} targets: ${targetIds.join(",")}`);
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
 * Build env2 for replanning after checkpoint freeze
 * - Creates synthetic start nodes at frozen positions
 * - Marks visited targets
 * - Returns modified env and droneConfigs
 */
function buildCheckpointEnv() {
  appendDebugLine(`buildCheckpointEnv: checkpoint.active=${state.checkpoint?.active}, segments=${Object.keys(state.checkpoint?.segments || {}).join(",") || "NONE"}`);
  appendDebugLine(`buildCheckpointEnv: visitedTargets=[${state.visitedTargets.join(", ")}]`);

  // ========================================
  // NEW CLEAN PATH: Use SegmentedImportManager with stored segment data
  // Each segment stores all 5 items:
  //   1. frozenTargets - targets visited before this segment
  //   2. frozenTrajectories - locked trajectory portions from previous segments
  //   3. activeTargets - targets to solve in this segment
  //   4. droneConfigs - which drones enabled, start/end airports, fuel
  //   5. startPositions - where drones start (airport or cut marker)
  // ========================================
  if (segmentedImport.isActive()) {
    const segIdx = segmentedImport.getCurrentSegmentIndex();
    appendDebugLine(`buildCheckpointEnv: NEW MANAGER, segment ${segIdx}`);

    // Check if we have a checkpoint cut during animation (state.checkpoint.active)
    // In this case, we need to use the checkpoint split positions, not the original segment cut positions
    const hasCheckpointCut = state.checkpoint?.active && Object.keys(state.checkpoint.segments || {}).length > 0;
    appendDebugLine(`   hasCheckpointCut=${hasCheckpointCut}`);

    // ========================================
    // Get stored segment data - all 5 items
    // ========================================
    const segmentData = segmentedImport.getSegmentData(segIdx);
    if (segmentData) {
      appendDebugLine(`   Segment data loaded: frozenTargets=${segmentData.frozenTargets?.length || 0}, activeTargets=${segmentData.activeTargets?.length || 0}`);
    }

    // 1. Frozen targets (targets visited before this segment)
    const frozenTargetIds = segmentData?.frozenTargets || [];

    // 2. Frozen trajectories (locked trajectory portions from previous segments)
    // Note: These are handled by the accept flow, not buildCheckpointEnv

    // 3. Active targets (targets to solve in this segment)
    const activeTargets = segmentData?.activeTargets || [];

    // 4. Drone configs - USE UI state.droneConfigs as source of truth
    // The user controls which drones are enabled via UI checkboxes
    // This applies to both fresh segmentation AND imported segments being re-solved
    const droneConfigs = JSON.parse(JSON.stringify(state.droneConfigs || {}));

    // 5. Start positions (where drones start - airport or cut marker)
    const startPositions = segmentData?.startPositions || null;

    // Log disabled drones only
    const disabledDrones = Object.keys(droneConfigs).filter(did => !droneConfigs[did]?.enabled);
    if (disabledDrones.length > 0) {
      appendDebugLine(`   Disabled drones: ${disabledDrones.join(", ")}`);
    }

    // Build solver environment with active targets only (no frozen targets)
    const baseEnv = segmentedImport.getBaseEnv();
    const solverEnv = {
      airports: JSON.parse(JSON.stringify(baseEnv?.airports || state.env?.airports || [])),
      sams: JSON.parse(JSON.stringify(state.env?.sams || [])),  // Use current SAMs (user may have edited)
      targets: JSON.parse(JSON.stringify(activeTargets)),
      drone_configs: JSON.parse(JSON.stringify(droneConfigs)),
      synthetic_starts: {},
    };

    // IMPORTANT: Also include any targets added during edit mode that aren't in the stored data
    const activeTargetIds = new Set(activeTargets.map(t => t.id));
    const stateTargets = state.env?.targets || [];
    const newlyAddedTargets = stateTargets.filter(t =>
      !activeTargetIds.has(t.id) && !frozenTargetIds.includes(t.id)
    );

    if (newlyAddedTargets.length > 0) {
      appendDebugLine(`   Newly added targets from edits: ${newlyAddedTargets.map(t => t.id).join(",")}`);
      solverEnv.targets = [...solverEnv.targets, ...newlyAddedTargets];
    }

    appendDebugLine(`   Targets for solver: ${solverEnv.targets.map(t => t.id).join(",")}`);
    appendDebugLine(`   Frozen (excluded): ${frozenTargetIds.join(",")}`);

    // If there's a checkpoint cut during animation, use those positions instead of stored segment positions
    if (hasCheckpointCut) {
      appendDebugLine(`   Using CHECKPOINT cut positions (mid-animation cut)`);

      Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
        if (!seg?.splitPoint) return;
        const nodeId = `D${did}_START`;
        solverEnv.synthetic_starts[nodeId] = { id: nodeId, x: seg.splitPoint[0], y: seg.splitPoint[1] };
        // Update start_airport to use synthetic start
        if (droneConfigs[did]) {
          droneConfigs[did].start_airport = nodeId;
        }
        // Also update in solverEnv.drone_configs
        if (solverEnv.drone_configs && solverEnv.drone_configs[did]) {
          solverEnv.drone_configs[did].start_airport = nodeId;
        }
        appendDebugLine(`   Drone ${did}: start from checkpoint [${seg.splitPoint[0].toFixed(2)}, ${seg.splitPoint[1].toFixed(2)}]`);
      });
    } else if (segIdx > 0 && startPositions) {
      // Use stored start positions for segment > 0
      Object.entries(startPositions).forEach(([did, pos]) => {
        if (!pos || pos.length !== 2) return;
        const nodeId = `D${did}_START`;
        solverEnv.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };
        // Update droneConfigs to use synthetic start
        if (droneConfigs[did]) {
          droneConfigs[did].start_airport = nodeId;
        }
        if (solverEnv.drone_configs && solverEnv.drone_configs[did]) {
          solverEnv.drone_configs[did].start_airport = nodeId;
        }
        appendDebugLine(`   Drone ${did}: start from stored position [${pos[0].toFixed(2)}, ${pos[1].toFixed(2)}] → start_airport=${nodeId}`);
      });
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
    // Use state.droneConfigs as base to preserve UI-configured values (end_airport, fuel_budget)
    const newDroneConfigs = JSON.parse(JSON.stringify(state.droneConfigs));

    Object.entries(currentSeg.cutPositions).forEach(([did, pos]) => {
      if (!pos || pos.length !== 2) return;
      const nodeId = `D${did}_START`;
      env2.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };
      if (newDroneConfigs[did]) {
        newDroneConfigs[did].start_airport = nodeId;
      }
    });

    return { env: env2, droneConfigs: newDroneConfigs, isCheckpointReplan: true };
  }

  if (!state.checkpoint?.active || !state.checkpoint.segments) {
    appendDebugLine("buildCheckpointEnv: NOT a checkpoint replan (checkpoint.active is false or no segments)");
    return { env: state.env, droneConfigs: state.droneConfigs, isCheckpointReplan: false };
  }

  // Deep copy current env
  const env2 = JSON.parse(JSON.stringify(state.env));

  // Add synthetic start nodes for each frozen drone
  env2.synthetic_starts = env2.synthetic_starts || {};

  // Deep copy drone configs
  const newDroneConfigs = JSON.parse(JSON.stringify(state.droneConfigs));

  // Calculate remaining fuel for each drone
  Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
    if (!seg?.splitPoint) return;

    const nodeId = `D${did}_START`;
    env2.synthetic_starts[nodeId] = {
      id: nodeId,
      x: seg.splitPoint[0],
      y: seg.splitPoint[1],
    };

    // Update drone config to start from synthetic node
    if (newDroneConfigs[did]) {
      newDroneConfigs[did].start_airport = nodeId;

      // Calculate remaining fuel (original budget minus distance traveled)
      const originalBudget = newDroneConfigs[did].fuel_budget || 150;
      const distanceTraveled = seg.checkpointDist || 0;
      newDroneConfigs[did].fuel_budget = Math.max(0, originalBudget - distanceTraveled);
    }
  });

  // Mark visited targets (exclude them from planning or mark as completed)
  env2.visited_targets = [...state.visitedTargets];

  // Optionally filter out visited targets from the targets array
  // (depending on how backend handles this)
  // env2.targets = env2.targets.filter(t => !state.visitedTargets.includes(t.id));

  appendDebugLine(`buildCheckpointEnv: IS checkpoint replan. synthetic_starts=${Object.keys(env2.synthetic_starts).join(",")}, visited=${env2.visited_targets.length}`);

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
 * Build combined routes from all MissionReplay segments
 * For multi-segment missions, concatenates trajectories from frozen solutions
 * Returns combined routes object ready for display/animation
 */
function buildCombinedRoutesFromSegments() {
  const segmentCount = missionReplay.getSegmentCount();
  if (segmentCount <= 1) {
    // Single segment or no segments - return state.routes as-is
    return state.routes;
  }

  console.log(`[buildCombinedRoutesFromSegments] Combining ${segmentCount} segments`);
  appendDebugLine(`Building combined routes from ${segmentCount} segments...`);

  const combinedRoutes = {};

  for (let i = 0; i < segmentCount; i++) {
    const seg = missionReplay.getSegment(i);
    if (!seg || !seg.solution || !seg.solution.routes) {
      console.log(`[buildCombinedRoutesFromSegments] Segment ${i}: no solution/routes`);
      continue;
    }

    Object.entries(seg.solution.routes).forEach(([droneId, routeData]) => {
      const trajectory = routeData.trajectory || [];
      if (trajectory.length === 0) return;

      if (!combinedRoutes[droneId]) {
        // First segment for this drone - copy route data
        combinedRoutes[droneId] = {
          ...routeData,
          trajectory: [...trajectory],
          distance: routeData.distance || 0,
        };
        console.log(`[buildCombinedRoutesFromSegments] Drone ${droneId}: segment ${i} init with ${trajectory.length} pts`);
      } else {
        // Subsequent segment - concatenate trajectory
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
        console.log(`[buildCombinedRoutesFromSegments] Drone ${droneId}: segment ${i} added, total pts=${combinedRoutes[droneId].trajectory.length}`);
      }
    });
  }

  const droneCount = Object.keys(combinedRoutes).length;
  appendDebugLine(`Combined routes: ${droneCount} drones from ${segmentCount} segments`);
  return combinedRoutes;
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
  // Show start/end airports for each drone (for debugging end_airport issue)
  const airportInfo = Object.entries(configsToSolve).map(([did, cfg]) =>
    `D${did}:${cfg.start_airport || '?'}→${cfg.end_airport || '?'}`
  ).join(", ");
  appendDebugLine(`   Start→End airports: ${airportInfo}`);
  console.log(`[SOLVE] Drone configs sent to solver:`, configsToSolve);


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
      appendDebugLine("❌ Planner reported an error.");
      return;
    }

    // Log raw solver response for debugging trajectory issues
    if (isCheckpointReplan) {
      console.log("🔍 [Solver Raw Response] routes:");
      Object.entries(data.routes || {}).forEach(([did, r]) => {
        console.log(`🔍 D${did}: route=${JSON.stringify(r.route)}, trajectory.length=${r.trajectory?.length || 0}`);
      });
    }

    // If this was a checkpoint replan, splice new trajectories with frozen prefixes
    if (isCheckpointReplan && state.checkpoint?.segments) {
      const newRoutes = data.routes || {};

      Object.entries(newRoutes).forEach(([did, routeData]) => {
        const seg = state.checkpoint.segments[did];
        if (!seg?.prefix) return;

        // If solver didn't return a trajectory, we'll just use the prefix for now
        // and regenerate later. Log this as a warning.
        if (!routeData?.trajectory || routeData.trajectory.length === 0) {
          console.warn(`[Splice] D${did}: solver returned no trajectory, using prefix only (will need regeneration)`);
          routeData.trajectory = seg.prefix;
          return;
        }

        const prefix = seg.prefix;
        const newSuffix = routeData.trajectory;

        // Splice: prefix + newSuffix (avoiding duplicate join point)
        let joined;
        if (prefix.length > 0 && newSuffix.length > 0) {
          const lastPrefixPt = prefix[prefix.length - 1];
          const firstSuffixPt = newSuffix[0];
          if (Math.abs(lastPrefixPt[0] - firstSuffixPt[0]) < 0.001 &&
              Math.abs(lastPrefixPt[1] - firstSuffixPt[1]) < 0.001) {
            joined = prefix.concat(newSuffix.slice(1));
          } else {
            joined = prefix.concat(newSuffix);
          }
        } else {
          joined = prefix.concat(newSuffix);
        }

        routeData.trajectory = joined;
        const firstPt = joined.length > 0 ? `(${joined[0][0].toFixed(1)},${joined[0][1].toFixed(1)})` : "N/A";
        const lastPt = joined.length > 0 ? `(${joined[joined.length-1][0].toFixed(1)},${joined[joined.length-1][1].toFixed(1)})` : "N/A";
        const prefixFirst = prefix.length > 0 ? `(${prefix[0][0].toFixed(1)},${prefix[0][1].toFixed(1)})` : "N/A";
        const prefixLast = prefix.length > 0 ? `(${prefix[prefix.length-1][0].toFixed(1)},${prefix[prefix.length-1][1].toFixed(1)})` : "N/A";
        appendDebugLine(`📎 D${did}: prefix=${prefix.length}pts [${prefixFirst}→${prefixLast}] + suffix=${newSuffix.length}pts = joined=${joined.length}pts [${firstPt}→${lastPt}]`);
      });

      data.routes = newRoutes;
      appendDebugLine("📎 Spliced new trajectories with frozen prefixes");
      // NOTE: state.pendingCutDistance was already set in freezeAtCheckpoint()
      appendDebugLine(`📏 Cut distance for this segment: ${state.pendingCutDistance?.toFixed(1) || "null"}`);
    }

    // Store as draft solution (two-phase commit)
    // IMPORTANT: Deep copy routes to prevent modifications to state.routes from affecting the draft
    // Also save checkpoint state so it can be restored if the draft is discarded
    missionState.draftSolution = {
      sequences: JSON.parse(JSON.stringify(data.sequences || {})),
      routes: JSON.parse(JSON.stringify(data.routes || {})),
      wrappedPolygons: JSON.parse(JSON.stringify(data.wrapped_polygons || [])),
      allocations: JSON.parse(JSON.stringify(data.allocations || {})),
      allocationStrategy: data.allocation_strategy || null,
      distanceMatrix: data.distance_matrix ? JSON.parse(JSON.stringify(data.distance_matrix)) : null,
      isCheckpointReplan: isCheckpointReplan,
      checkpointSegments: isCheckpointReplan ? JSON.parse(JSON.stringify(state.checkpoint.segments)) : null,
      cutDistance: isCheckpointReplan ? state.pendingCutDistance : null,
      // Save cut positions so they can be restored on discard
      pendingCutPositions: state.pendingCutPositions ? JSON.parse(JSON.stringify(state.pendingCutPositions)) : null,
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
            console.warn(`[Solve] D${did}: trajectory seems incomplete (${trajLen}pts <= frozen ${frozenCount}pts), will regenerate`);
            routesNeedingRegeneration.push(did);
          }
        }
      });

      if (routesNeedingRegeneration.length > 0) {
        console.log(`[Solve] Regenerating trajectories for drones: ${routesNeedingRegeneration.join(", ")}`);
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
  console.log("📥 applyDraftSolutionToUI called, isCheckpointReplan:", draft.isCheckpointReplan);
  Object.entries(draft.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
      console.log(`📥 draft.routes[${did}]: ${trajLen}pts ${firstPt}→${lastPt}`);
    }
  });

  // Deep copy to allow UI modifications without affecting the stored draft
  state.sequences = JSON.parse(JSON.stringify(draft.sequences || {}));

  // For segmented import: MERGE draft routes with frozen routes (don't replace!)
  if (segmentedImport.isActive() && draft.isCheckpointReplan) {
    // Keep existing frozen routes and add/update with draft routes
    const mergedRoutes = { ...state.routes };
    Object.entries(draft.routes || {}).forEach(([did, routeData]) => {
      // For checkpoint replans, the draft contains ONLY the new segment's trajectory
      // We need to prepend the frozen trajectory for this drone
      if (mergedRoutes[did] && mergedRoutes[did].trajectory) {
        // Already have frozen trajectory - concatenate new segment
        const frozenTraj = mergedRoutes[did].trajectory;
        const newTraj = routeData.trajectory || [];

        // Check for duplicate at junction
        const lastPoint = frozenTraj[frozenTraj.length - 1];
        const firstPoint = newTraj[0];
        const isDuplicate = lastPoint && firstPoint &&
          Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
          Math.abs(lastPoint[1] - firstPoint[1]) < 0.001;

        mergedRoutes[did] = {
          ...routeData,
          trajectory: isDuplicate ? frozenTraj.concat(newTraj.slice(1)) : frozenTraj.concat(newTraj),
          distance: (mergedRoutes[did].distance || 0) + (routeData.distance || 0),
        };
      } else {
        // No frozen trajectory yet, just use the draft
        mergedRoutes[did] = JSON.parse(JSON.stringify(routeData));
      }
    });
    state.routes = mergedRoutes;
    console.log(`📥 Merged frozen + draft routes for drones: ${Object.keys(mergedRoutes).join(", ")}`);
  } else {
    // Not a segmented import checkpoint - just replace as before
    state.routes = JSON.parse(JSON.stringify(draft.routes || {}));
  }

  state.wrappedPolygons = JSON.parse(JSON.stringify(draft.wrappedPolygons || []));
  state.allocations = JSON.parse(JSON.stringify(draft.allocations || {}));
  state.allocationStrategy = draft.allocationStrategy || null;
  state.distanceMatrix = draft.distanceMatrix ? JSON.parse(JSON.stringify(draft.distanceMatrix)) : null;

  // Debug: Log what state.routes looks like after copy
  console.log("📥 state.routes after copy:");
  Object.entries(state.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
      console.log(`📥 state.routes[${did}]: ${trajLen}pts ${firstPt}→${lastPt}`);
    }
  });

  // After checkpoint replan, update animation drones to show at their new starting positions
  if (draft.isCheckpointReplan && state.animation.drones && state.pendingCutDistance !== null) {
    const cutDist = state.pendingCutDistance;
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
  console.log("🎯 draft.allocations:", draft.allocations);
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

  // Initialize per-drone cut positions map
  state.pendingCutPositions = {};

  // NEW: Capture single cut distance (same for all drones since they fly at same speed)
  // Use missionDistance from animation state, or calculate from first animating drone
  const firstDroneState = Object.values(state.animation.drones)[0];
  state.pendingCutDistance = firstDroneState?.distanceTraveled || 0;
  appendDebugLine(`✂️ Cut at distance: ${state.pendingCutDistance.toFixed(1)}`);

  // Track which targets are visited in the current (frozen) routes
  const targetsVisitedThisSegment = [];

  Object.entries(state.animation.drones).forEach(([did, droneState]) => {
    const traj = state.routes[did]?.trajectory || [];
    if (traj.length < 2 || droneState.totalDistance <= 0) return;

    // Use the drone's CURRENT distanceTraveled (actual position during animation)
    const currentDist = droneState.distanceTraveled || 0;

    const { prefixPoints, suffixPoints, splitPoint } =
      split_polyline_at_distance(traj, currentDist);

    // Store the current position as splitPoint for drawing
    state.checkpoint.segments[did] = {
      prefix: prefixPoints,
      suffix: suffixPoints,
      splitPoint,
      checkpointDist: currentDist,
    };

    // Capture per-drone cut position for later "Accept"
    if (splitPoint && splitPoint.length === 2) {
      state.pendingCutPositions[did] = [splitPoint[0], splitPoint[1]];
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
        console.log(`✓ D${did}: Marked ${wp} as visited (dist to traj: ${minDist.toFixed(1)}, targetDist: ${targetDist.toFixed(1)}, currentDist: ${currentDist.toFixed(1)})`);
      } else {
        console.log(`○ D${did}: ${wp} NOT visited (minDist=${minDist.toFixed(1)}, targetDist=${targetDist.toFixed(1)}, currentDist=${currentDist.toFixed(1)})`);
      }
    });

    // Stop animating but keep current position
    droneState.animating = false;

    // Store full trajectory and replace with prefix (traveled portion only)
    state.routes[did]._fullTrajectory = state.routes[did]._fullTrajectory || traj;
    state.routes[did].trajectory = prefixPoints;
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

  // Debug: log captured cut positions
  appendDebugLine(`✂️ pendingCutPositions=${JSON.stringify(state.pendingCutPositions || {})}`);

  updateAnimationButtonStates();
  drawEnvironment();
  appendDebugLine(`Frozen at ${avgPct.toFixed(0)}% progress`);
}
console.log("freezeAtCheckpoint defined?", typeof freezeAtCheckpoint, "line marker A");

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
      console.log(`⚠️ startAnimation D${did}: _fullTrajectory exists (${state.routes[did]._fullTrajectory.length} pts), restoring...`);
      state.routes[did].trajectory = state.routes[did]._fullTrajectory;
      delete state.routes[did]._fullTrajectory;
    }
  });

  // Debug: Log trajectories at animation start
  console.log("🎬 startAnimation: trajectories before animation:");
  Object.entries(state.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
      console.log(`🎬 state.routes[${did}]: ${trajLen}pts ${firstPt}→${lastPt}`);
    }
  });

  // Initialize animation state for selected drones
  state.animation.active = true;
  state.animation.drones = {};

  // NEW: Single mission distance tracker (same for all drones since they fly at same speed)
  // Get starting distance from MissionReplay (0 for segment 0, cutDistance for checkpoint replans)
  state.animation.missionDistance = missionReplay.getStartingDistance();
  appendDebugLine(`🎬 Mission starting at distance: ${state.animation.missionDistance.toFixed(1)}`);

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
      console.log("SPLIT TEST", did, {
        total,
        checkpointDist,
        splitPoint: res.splitPoint,
        prefixLen: res.prefixPoints.length,
        suffixLen: res.suffixPoints.length,
        splitIndex: res.splitIndex,
        t: res.t,
      });
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

    // NEW: Advance single mission distance (same for all drones)
    state.animation.missionDistance += speedUnitsPerSec * dt;

    Object.entries(state.animation.drones).forEach(([did, droneState]) => {
      if (!droneState.animating) return;

      // 1) Sync drone's distanceTraveled with mission distance
      droneState.distanceTraveled = state.animation.missionDistance;

      // DEBUG: Log distance periodically (every 50 units)
      const lastDistLog = droneState._lastDistLog || 0;
      if (droneState.distanceTraveled - lastDistLog > 50) {
        droneState._lastDistLog = droneState.distanceTraveled;
        console.log(`📍 D${did}: distanceTraveled=${droneState.distanceTraveled.toFixed(1)}, totalDistance=${droneState.totalDistance.toFixed(1)}`);
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

      // 4) Check if drone has passed any target waypoints and mark them as visited
      const routeInfo = state.routes[did];
      if (routeInfo && routeInfo.route && routeInfo.trajectory) {
        const trajectory = routeInfo.trajectory;
        const route = routeInfo.route;
        const cumulativeDistances = droneState.cumulativeDistances || [];
        const currentDistance = droneState.distanceTraveled;

        // Debug: Log once per drone at start of animation
        if (droneState.distanceTraveled < 1 && !droneState._debugLogged) {
          droneState._debugLogged = true;
          const targets = route.filter(wp => String(wp).startsWith("T"));
          const envTargetIds = (state.env.targets || []).map(t => t.id);
          console.log(`🔍 D${did} route targets: ${targets.join(", ")}, trajectory points: ${trajectory.length}`);
          console.log(`🔍 D${did} env targets: [${envTargetIds.join(", ")}]`);
          appendDebugLine(`🔍 D${did} route targets: [${targets.join(", ")}]`);

          // Check which route targets are NOT in env (with detailed type info)
          const missingInEnv = targets.filter(t => !envTargetIds.includes(t));
          if (missingInEnv.length > 0) {
            console.log(`⚠️ D${did} MISSING in env: [${missingInEnv.join(", ")}]`);
            appendDebugLine(`⚠️ D${did} MISSING in env: [${missingInEnv.join(", ")}]`);
            // Log type info for debugging
            console.log(`⚠️ D${did} route target types: ${targets.map(t => typeof t).join(", ")}`);
            console.log(`⚠️ D${did} env target ID types: ${envTargetIds.slice(0,5).map(t => typeof t).join(", ")}`);
          }

          // Also log if any targets are already visited
          const alreadyVisited = targets.filter(t => state.visitedTargets.includes(t));
          if (alreadyVisited.length > 0) {
            console.log(`✓ D${did} already visited: [${alreadyVisited.join(", ")}]`);
          }
        }

        // For each target in the route, check if we've passed its position
        route.forEach((wp, wpIdx) => {
          if (!String(wp).startsWith("T")) return;  // Only targets
          if (state.visitedTargets.includes(wp)) return;  // Already visited

          // CRITICAL FIX: Only mark targets in SEQUENTIAL ORDER
          // Check if all previous targets in the route have been visited
          // This prevents future targets from being marked prematurely even if geometrically close
          let canMarkThisTarget = true;
          for (let i = 0; i < wpIdx; i++) {
            const prevWp = route[i];
            if (String(prevWp).startsWith("T") && !state.visitedTargets.includes(prevWp)) {
              // Previous target not yet visited, can't mark this one
              canMarkThisTarget = false;
              break;
            }
          }
          if (!canMarkThisTarget) return;

          // Find the target's position in the environment
          const target = state.env.targets?.find(t => t.id === wp);
          if (!target) {
            // Debug: target not found in env - this target is in a future segment
            if (!droneState[`_debugNoTarget_${wp}`]) {
              droneState[`_debugNoTarget_${wp}`] = true;
              console.log(`⚠️ D${did} target ${wp} not in state.env.targets - skipping (future segment)`);
            }
            return;
          }

          // Check if we've passed close to this target's position
          // Find the trajectory point closest to the target
          let minDist = Infinity;
          let closestIdx = -1;
          trajectory.forEach((pt, idx) => {
            const dist = Math.sqrt(Math.pow(pt[0] - target.x, 2) + Math.pow(pt[1] - target.y, 2));
            if (dist < minDist) {
              minDist = dist;
              closestIdx = idx;
            }
          });

          // Debug: Log closest distance once per target
          if (!droneState[`_debugDist_${wp}`]) {
            droneState[`_debugDist_${wp}`] = true;
            console.log(`📏 D${did} target ${wp} at (${target.x.toFixed(1)}, ${target.y.toFixed(1)}), closest traj point idx=${closestIdx}, minDist=${minDist.toFixed(2)}`);
          }

          // If we found a close point and we've traveled past it, mark target as visited
          // Increased threshold from 1.0 to 20.0 to account for trajectory smoothing
          if (closestIdx >= 0 && minDist < 20.0 && cumulativeDistances[closestIdx] !== undefined) {
            const targetDistance = cumulativeDistances[closestIdx];
            if (currentDistance >= targetDistance) {
              state.visitedTargets.push(wp);
              console.log(`🎯 D${did} passed target ${wp} at distance ${targetDistance.toFixed(1)} (minDist=${minDist.toFixed(2)})`);
              appendDebugLine(`🎯 D${did} visited ${wp}`);
            } else {
              // Debug: Log why target not yet passed (periodically)
              const logKey = `_debugNotYet_${wp}`;
              const lastLog = droneState[logKey] || 0;
              if (currentDistance - lastLog > 20) {  // Log every 20 units of travel
                droneState[logKey] = currentDistance;
                console.log(`⏳ D${did} target ${wp}: currentDist=${currentDistance.toFixed(1)} < targetDist=${targetDistance.toFixed(1)} (need ${(targetDistance - currentDistance).toFixed(1)} more)`);
              }
            }
          } else if (!droneState[`_debugSkip_${wp}`] && closestIdx >= 0) {
            // Debug: Log why target was skipped (only once)
            droneState[`_debugSkip_${wp}`] = true;
            if (minDist >= 20.0) {
              console.log(`⚠️ D${did} target ${wp} skipped: minDist=${minDist.toFixed(2)} >= 20.0`);
            } else if (cumulativeDistances[closestIdx] === undefined) {
              console.log(`⚠️ D${did} target ${wp} skipped: cumulativeDistances[${closestIdx}] undefined (len=${cumulativeDistances.length})`);
            }
          }
        });
      }
    });

    // MISSION REPLAY: Check segment boundary using single mission distance
    // This is simple: when missionDistance >= nextSegment.cutDistance, switch segments
    const { switched, newSegment } = missionReplay.checkSegmentBoundary(state.animation.missionDistance);

    if (switched && newSegment) {
      appendDebugLine(`🔄 === SEGMENT SWITCH to ${newSegment.index} (via MissionReplay) ===`);

      // Also sync missionState for backward compatibility
      missionState.currentSegmentIndex = newSegment.index;

      // SEGMENTED IMPORT: Update environment and targets using SegmentedImportManager
      if (segmentedImport.isActive()) {
        // Sync SegmentedImportManager's current segment index
        segmentedImport._currentSegmentIndex = newSegment.index;

        // Get environment for display (cumulative targets from segments 0 to current)
        state.env = segmentedImport.getEnvForDisplay();
        state.droneConfigs = JSON.parse(JSON.stringify(state.env.drone_configs || {}));
        state.env.drone_configs = state.droneConfigs;

        appendDebugLine(`🔄 SegmentedImport: Updated to segment ${newSegment.index}, now showing ${state.env.targets?.length || 0} targets`);
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
            appendDebugLine(`🔄 Added ${targetsToAdd.length} targets from segment ${newSegment.index}: ${targetsToAdd.map(t=>t.id).join(",")}`);
          }
          appendDebugLine(`🔄 Now displaying ${state.env.targets?.length || 0} targets total`);
        } else if (!state.initialEnvSnapshot) {
          // Fallback: use segment env if no snapshot available
          state.env = JSON.parse(JSON.stringify(newSegment.env));
          appendDebugLine(`🔄 Environment updated with ${state.env.targets?.length || 0} targets (no snapshot available)`);
        }
      }

      // SEGMENTED IMPORT: Build CUMULATIVE trajectory from all segments 0 to newSegment.index
      // Each segment in MissionReplay stores only its PORTION (e.g., C1->C2)
      // We need to concatenate them to get the full trajectory (A1->C1->C2)
      if (segmentedImport.isActive()) {
        console.log(`🔄 SEGMENT SWITCH (SegmentedImport): Building cumulative trajectory for segments 0 to ${newSegment.index}`);

        // Build cumulative routes from all segments
        const cumulativeRoutes = {};
        for (let segIdx = 0; segIdx <= newSegment.index; segIdx++) {
          const seg = missionReplay.getSegment(segIdx);
          if (!seg || !seg.solution || !seg.solution.routes) continue;

          Object.entries(seg.solution.routes).forEach(([did, routeData]) => {
            const traj = routeData.trajectory || [];
            const route = routeData.route || [];
            if (traj.length === 0) return;

            if (!cumulativeRoutes[did]) {
              // First segment for this drone
              cumulativeRoutes[did] = {
                trajectory: [...traj],
                route: [...route],
              };
              console.log(`🔄 D${did} seg${segIdx}: initial ${traj.length} pts`);
            } else {
              // Concatenate with previous segments
              const existingTraj = cumulativeRoutes[did].trajectory;
              const existingRoute = cumulativeRoutes[did].route;
              const lastPoint = existingTraj[existingTraj.length - 1];
              const firstPoint = traj[0];

              // Check for duplicate at junction
              const isDuplicate = lastPoint && firstPoint &&
                Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
                Math.abs(lastPoint[1] - firstPoint[1]) < 0.001;

              cumulativeRoutes[did].trajectory = isDuplicate
                ? existingTraj.concat(traj.slice(1))
                : existingTraj.concat(traj);
              cumulativeRoutes[did].route = existingRoute.concat(route);

              console.log(`🔄 D${did} seg${segIdx}: added ${traj.length} pts, cumulative now ${cumulativeRoutes[did].trajectory.length} pts`);
            }
          });
        }

        // Update state.routes with cumulative trajectories
        Object.entries(cumulativeRoutes).forEach(([did, routeData]) => {
          if (state.routes[did]) {
            const oldLen = state.routes[did].trajectory?.length || 0;
            state.routes[did].trajectory = routeData.trajectory;
            state.routes[did].route = routeData.route;
            const newLen = state.routes[did].trajectory.length;
            appendDebugLine(`🔄 D${did}: cumulative trajectory ${oldLen}→${newLen} pts (segments 0-${newSegment.index})`);

            // Update drone's cumulative distances and total distance for the new trajectory
            const droneState = state.animation.drones[did];
            if (droneState) {
              const trajectory = state.routes[did].trajectory;
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
              // Keep distanceTraveled as-is (we're continuing from C point)

              // Re-enable animation if there's more to fly
              if (droneState.distanceTraveled < totalDistance) {
                if (!droneState.animating) {
                  appendDebugLine(`🔄 D${did}: Re-enabling animation (was finished, now has more trajectory)`);
                }
                droneState.animating = true;
              }

              appendDebugLine(`🔄 D${did}: totalDistance now ${totalDistance.toFixed(1)}, distanceTraveled=${droneState.distanceTraveled.toFixed(1)}, animating=${droneState.animating}`);
            }
          }
        });
      } else {
        // LEGACY: For non-segmented imports, use newSegment's routes directly
        console.log(`🔄 SEGMENT SWITCH (Legacy): newSegment.solution.routes keys:`, Object.keys(newSegment.solution.routes || {}));
        Object.entries(newSegment.solution.routes || {}).forEach(([did, routeData]) => {
          const traj = routeData.trajectory || [];
          console.log(`🔄 D${did} from segment: trajectory=${traj.length} pts, route=[${(routeData.route || []).join(",")}]`);
          // IMPORTANT: Skip empty trajectories - this preserves the frozen trajectory
          // for drones that were disabled/lost at a previous checkpoint
          if (traj.length === 0) {
            console.log(`🔄 D${did}: skipping empty trajectory (drone may be lost/disabled)`);
            return;
          }
          if (state.routes[did]) {
            const oldLen = state.routes[did].trajectory?.length || 0;
            state.routes[did].trajectory = JSON.parse(JSON.stringify(traj));
            state.routes[did].route = JSON.parse(JSON.stringify(routeData.route || []));
            const newLen = state.routes[did].trajectory.length;
            appendDebugLine(`🔄 D${did}: trajectory ${oldLen}→${newLen} pts`);

            // Update drone's cumulative distances and total distance for the new trajectory
            const droneState = state.animation.drones[did];
            if (droneState) {
              const trajectory = state.routes[did].trajectory;
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
              // Keep distanceTraveled as-is (we're continuing from C point)

              // IMPORTANT: Re-enable animation for drones that have more to fly
              // This handles drones that completed their segment 1 flight but have new targets in segment 2
              if (droneState.distanceTraveled < totalDistance) {
                if (!droneState.animating) {
                  appendDebugLine(`🔄 D${did}: Re-enabling animation (was finished, now has more trajectory)`);
                }
                droneState.animating = true;
              }

              appendDebugLine(`🔄 D${did}: totalDistance now ${totalDistance.toFixed(1)}, distanceTraveled=${droneState.distanceTraveled.toFixed(1)}, animating=${droneState.animating}`);
            }
          }
        });
      }

      appendDebugLine(`🔄 === SEGMENT SWITCH COMPLETE ===`);

      // Clear debug flags so targets get re-evaluated with new trajectory
      Object.entries(state.animation.drones).forEach(([did, ds]) => {
        // Clear _debugDist_ flags so we log distances for targets in new segment
        Object.keys(ds).filter(k => k.startsWith('_debug')).forEach(k => delete ds[k]);
        console.log(`🔄 Cleared debug flags for D${did}`);
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

        route.forEach((wp, wpIdx) => {
          if (!String(wp).startsWith("T")) return;
          if (state.visitedTargets.includes(wp)) return;

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

          const target = state.env.targets?.find(t => t.id === wp);
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
          if (closestIdx >= 0 && minDist < 20.0 && cumulativeDistances[closestIdx] !== undefined) {
            const targetDistance = cumulativeDistances[closestIdx];
            if (currentDistance >= targetDistance) {
              state.visitedTargets.push(wp);
              console.log(`🎯 SEGMENT-SWITCH: D${did} passed target ${wp} at distance ${targetDistance.toFixed(1)} (current=${currentDistance.toFixed(1)})`);
              appendDebugLine(`🎯 D${did} visited ${wp} (at segment switch)`);
            }
          }
        });
      });

      // Pause for 1000ms at the C point to make the transition visible
      drawEnvironment();
      updateStatsFromAnimation();  // Update stats at segment switch
      setTimeout(() => {
        if (state.animation.active) {
          console.log(`🔄 Resuming animation after segment switch pause`);
          state.animation.animationId = requestAnimationFrame(animate);
        }
      }, 1000);
      return;  // Exit this frame, resume after timeout
    }

    drawEnvironment();

    // Update stats with real-time distance traveled
    updateStatsFromAnimation();

    // Check if there are more segments to switch to (pending checkpoints)
    const hasMoreSegments = missionReplay.getNextSegment() !== null;

    if (anyAnimating && state.animation.active) {
      state.animation.animationId = requestAnimationFrame(animate);
    } else if (!state.animation.active) {
      // Stopped externally (pause/cut) - don't change mode
      return;
    } else if (hasMoreSegments) {
      // No drones animating, but there are more segments pending
      // This can happen if a drone finishes early but other drones need segment switches
      // Keep the animation loop running to process segment switches
      console.log(`[animate] No drones animating but ${missionReplay.getSegmentCount() - missionReplay.getCurrentSegmentIndex() - 1} segments remaining`);
      state.animation.animationId = requestAnimationFrame(animate);
    } else {
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

          const target = state.env.targets?.find(t => t.id === wp);
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
              console.log(`🎯 D${did} visited ${wp} at distance ${currentDistance.toFixed(1)}`);
              appendDebugLine(`🎯 D${did} visited ${wp}`);
            }
          }
        });
      }
    });

    drawEnvironment();
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

  // Transition to CHECKPOINT state (from replay/cut - solve disabled)
  missionState.checkpointSource = "replay_cut";
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
    console.error("Replan failed:", text);
    appendDebugLine("Replan failed. See console.");
    return;
  }

  const data = await resp.json();

  // Expected: data.routes like your normal solver response:
  // { "1": { trajectory: [[x,y],...], route:[...]} , ... }
  if (!data.routes) {
    console.error("Replan response missing routes:", data);
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
    console.log(`[UI] Segment switch: ${fromIndex} -> ${toIndex}`);
    updateMissionControlState();
  });

  // Run Planner / Accept toggle button
  if (mcRunPlanner) {
    mcRunPlanner.addEventListener("click", () => {
      try {
        console.log(`[Accept Button] Click detected!`);
        console.log(`[Accept Button] missionState.mode=${missionState.mode}`);
        console.log(`[Accept Button] missionState.draftSolution exists=${!!missionState.draftSolution}`);
        const perms = getUiPermissions();
        console.log(`[Accept Button] perms.canAcceptSolution=${perms.canAcceptSolution}, perms.canSolve=${perms.canSolve}`);

        if (perms.canAcceptSolution) {
          // In Accept mode - accept the solution
          console.log(`[Accept Button] Calling acceptSolution()...`);
          acceptSolution();
        } else if (perms.canSolve) {
          // In Run Planner mode - run the planner
          console.log(`[Accept Button] Calling runPlanner()...`);
          runPlanner();
        } else if (missionState.mode === MissionMode.DRAFT_READY && missionState.draftSolution) {
          // Fallback: If we're in DRAFT_READY mode with a draft solution, accept anyway
          // This handles any edge case where perms isn't reflecting the correct state
          console.warn(`[Accept Button] FALLBACK: mode=${missionState.mode}, forcing acceptSolution()`);
          acceptSolution();
        } else {
          console.log(`[Accept Button] Neither canAcceptSolution nor canSolve is true - button click ignored`);
          console.log(`[Accept Button] Full perms:`, perms);
        }
      } catch (err) {
        console.error(`[Accept Button] ERROR:`, err);
        appendDebugLine(`Accept button error: ${err.message}`);
      }
    });
  } else {
    console.error(`[Accept Button] mc-run-planner element not found!`);
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
        console.error("Error clearing memories:", e);
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
    console.error("Error loading memories:", e);
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
    console.error("Error adding memory:", e);
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
    console.error("Error deleting memory:", e);
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

  // UI-permission guard: allow agent chat in most modes except animation
  // Agent can answer questions, re-optimize, or explain even after a solution exists
  const perms = (typeof getUiPermissions === "function") ? getUiPermissions() : null;
  if (perms && perms.isAnimating) {
    appendDebugLine("⚠️ Agent disabled during animation. Pause or stop animation first.");
    return;
  }
  // Block during editing mode to avoid conflicts
  if (perms && perms.isEditing) {
    appendDebugLine("⚠️ Agent disabled during Edit mode. Accept or cancel edits first.");
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
    console.log(`🚀 Agent request: ${targetCount} targets [${targetIds}], mission_id=${state.missionId || 'null'}`);
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
      console.log("📦 Sending existing solution to agent:", existingSolution);
      appendDebugLine(`📦 Agent context: ${Object.keys(existingSolution.routes).length} drone routes`);
    }

    const response = await fetch("/api/agents/chat-v4", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        message,
        env: state.env,
        drone_configs: state.droneConfigs,
        mission_id: state.missionId || null,
        existing_solution: Object.keys(existingSolution).length > 0 ? existingSolution : null,
      }),
    });

    const data = await response.json();

    // Store mission_id returned by backend, if any
    if (data.mission_id) {
      state.missionId = data.mission_id;
      appendDebugLine("Current mission_id: " + state.missionId);
    }

    // Debug: log the response to see what we're getting
    console.log("Agent response:", data);
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
    console.error("Agent error:", err);
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

function applyAgentRoutes(routes, trajectories, totalPoints, totalFuel) {
  // Apply multi-drone routes from the agent
  // routes is like {"1": ["A1", "T3", "A1"], "2": ["A2", "T5", "A2"], ...}
  // trajectories is like {"1": [[x1,y1], [x2,y2], ...], ...} - SAM-avoiding paths

  console.log("🎯 applyAgentRoutes called with:", { routes, trajectories, totalPoints, totalFuel });
  appendDebugLine(`🎯 applyAgentRoutes: ${Object.keys(routes || {}).length} routes`);

  if (!routes || Object.keys(routes).length === 0) {
    console.log("❌ No routes to apply");
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
      console.error("cutAtCheckpoint failed:", err);
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
