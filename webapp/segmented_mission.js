/**
 * SEGMENTED MISSION MANAGEMENT - CLEAN REWRITE
 *
 * DESIGN PRINCIPLES:
 * 1. state.droneConfigs is ALWAYS the source of truth for UI
 * 2. Solve READS from state.droneConfigs, never modifies it
 * 3. Accept SAVES state.droneConfigs to segment, never modifies state.droneConfigs
 * 4. Export READS each segment's saved droneConfigs
 * 5. Import LOADS segment 0's droneConfigs into state.droneConfigs once
 *
 * DATA MODEL:
 * Each segment stores 5 items:
 *   1. frozenTargets - target IDs visited BEFORE this segment
 *   2. frozenTrajectories - trajectory portions from previous segments
 *   3. activeTargets - targets to solve in this segment
 *   4. droneConfigs - which drones enabled, their configs at solve time
 *   5. startPositions - where drones start (null for seg-0, cut positions for seg>0)
 */

// =============================================================================
// SEGMENT CLASS - Immutable data structure for one solved segment
// =============================================================================
class Segment {
  constructor(data) {
    this.index = data.index ?? 0;
    this.solution = data.solution ?? { routes: {}, sequences: {} };
    this.env = data.env ?? {};
    this.cutDistance = data.cutDistance ?? null;
    this.cutPositions = data.cutPositions ?? null;
    this.lostDrones = data.lostDrones ?? [];
    this.visitedTargets = data.visitedTargets ?? [];
    this.droneConfigs = data.droneConfigs ?? {};
    this.timestamp = data.timestamp ?? Date.now();

    // Freeze to prevent accidental mutation
    Object.freeze(this);
  }
}

// =============================================================================
// MISSION REPLAY - Manages multi-segment mission playback
// =============================================================================
class MissionReplay {
  constructor() {
    this._segments = [];
    this._currentSegmentIndex = 0;
  }

  clear() {
    this._segments = [];
    this._currentSegmentIndex = 0;
  }

  addSegment(data) {
    const segment = new Segment({
      ...data,
      index: this._segments.length
    });
    this._segments.push(segment);
    return segment;
  }

  replaceSegment(index, data) {
    if (index < 0 || index >= this._segments.length) return null;
    const segment = new Segment({ ...data, index });
    this._segments[index] = segment;
    return segment;
  }

  getSegment(index) {
    return this._segments[index] ?? null;
  }

  getCurrentSegment() {
    return this.getSegment(this._currentSegmentIndex);
  }

  getSegmentCount() {
    return this._segments.length;
  }

  getCurrentSegmentIndex() {
    return this._currentSegmentIndex;
  }

  setCurrentSegmentIndex(index) {
    this._currentSegmentIndex = Math.max(0, Math.min(index, this._segments.length - 1));
  }

  resetToStart() {
    this._currentSegmentIndex = 0;
  }
}

// =============================================================================
// SEGMENTED MISSION MANAGER - Single source of truth for segmented workflows
// =============================================================================
class SegmentedMissionManager {
  constructor() {
    this.clear();
  }

  clear() {
    this._segments = [];           // Array of segment data
    this._allTargets = [];         // ALL targets (never modified after load)
    this._baseEnv = null;          // Base environment (airports, sams)
    this._currentSegmentIndex = 0;
    this._isActive = false;
  }

  isActive() {
    return this._isActive;
  }

  getCurrentSegmentIndex() {
    return this._currentSegmentIndex;
  }

  getTotalSegments() {
    return this._segments.length;
  }

  getAllTargets() {
    return JSON.parse(JSON.stringify(this._allTargets));
  }

  // =========================================================================
  // SEGMENT DATA ACCESSORS
  // =========================================================================

  getSegmentData(index) {
    return this._segments[index] ?? null;
  }

  getFrozenTargets(segmentIndex) {
    const seg = this._segments[segmentIndex];
    return seg?.frozenTargets ? [...seg.frozenTargets] : [];
  }

  getFrozenTrajectories(segmentIndex) {
    const seg = this._segments[segmentIndex];
    return seg?.frozenTrajectories ? JSON.parse(JSON.stringify(seg.frozenTrajectories)) : {};
  }

  getActiveTargets(segmentIndex) {
    const seg = this._segments[segmentIndex];
    return seg?.activeTargets ? JSON.parse(JSON.stringify(seg.activeTargets)) : [];
  }

  getDroneConfigs(segmentIndex) {
    const seg = this._segments[segmentIndex];
    return seg?.droneConfigs ? JSON.parse(JSON.stringify(seg.droneConfigs)) : {};
  }

  getStartPositions(segmentIndex) {
    const seg = this._segments[segmentIndex];
    return seg?.startPositions ? JSON.parse(JSON.stringify(seg.startPositions)) : null;
  }

  getCutDistance(segmentIndex) {
    const seg = this._segments[segmentIndex];
    return seg?.cutDistance ?? 0;
  }

  getLostDrones(segmentIndex) {
    const seg = this._segments[segmentIndex];
    return seg?.lostDrones ? [...seg.lostDrones] : [];
  }

  // =========================================================================
  // ENVIRONMENT BUILDERS
  // =========================================================================

  /**
   * Get environment for DISPLAY (targets up to current segment)
   * Uses current segment's droneConfigs
   */
  getEnvForDisplay() {
    if (!this._baseEnv) return null;

    const env = JSON.parse(JSON.stringify(this._baseEnv));

    // Get targets: frozen + active for current segment
    const frozenIds = new Set(this.getFrozenTargets(this._currentSegmentIndex));
    const activeTargets = this.getActiveTargets(this._currentSegmentIndex);

    // Combine frozen targets (from allTargets) + active targets
    const frozenTargetObjects = this._allTargets.filter(t => frozenIds.has(t.id));
    env.targets = [...frozenTargetObjects, ...activeTargets];

    // Use current segment's droneConfigs
    env.drone_configs = this.getDroneConfigs(this._currentSegmentIndex);

    return env;
  }

  /**
   * Get environment for SOLVER
   * ONLY active targets, synthetic starts for segment > 0
   * @param {Object} uiDroneConfigs - state.droneConfigs from UI (source of truth)
   */
  getEnvForSolver(uiDroneConfigs) {
    if (!this._baseEnv) return null;

    const env = JSON.parse(JSON.stringify(this._baseEnv));

    // Only active (unfrozen) targets
    env.targets = this.getActiveTargets(this._currentSegmentIndex);

    // Use UI droneConfigs - this is the source of truth
    env.drone_configs = JSON.parse(JSON.stringify(uiDroneConfigs));

    // Add synthetic starts for segment > 0
    env.synthetic_starts = {};
    if (this._currentSegmentIndex > 0) {
      const startPositions = this.getStartPositions(this._currentSegmentIndex);
      if (startPositions) {
        Object.entries(startPositions).forEach(([droneId, pos]) => {
          if (pos && Array.isArray(pos) && pos.length === 2) {
            const nodeId = `D${droneId}_START`;
            env.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };

            // Update drone's start_airport to synthetic start
            if (env.drone_configs[droneId]) {
              env.drone_configs[droneId].start_airport = nodeId;
            }
          }
        });
      }
    }

    return env;
  }

  /**
   * Get full environment (all targets) for export/snapshot
   */
  getFullEnv() {
    if (!this._baseEnv) return null;

    const env = JSON.parse(JSON.stringify(this._baseEnv));
    env.targets = JSON.parse(JSON.stringify(this._allTargets));
    return env;
  }

  // =========================================================================
  // SEGMENT NAVIGATION
  // =========================================================================

  advanceToNextSegment() {
    if (this._currentSegmentIndex < this._segments.length - 1) {
      this._currentSegmentIndex++;
      return true;
    }
    return false;
  }

  resetToSegment(index) {
    this._currentSegmentIndex = Math.max(0, Math.min(index, this._segments.length - 1));
  }

  // =========================================================================
  // IMPORT FROM JSON
  // =========================================================================

  /**
   * Load from v2 segments array format
   * Returns the droneConfigs for segment 0 (caller should set state.droneConfigs)
   */
  loadFromSegmentsArray(data) {
    this.clear();

    const segments = data.segments || [];
    if (segments.length === 0) return null;

    // Extract base environment
    const seg0Env = segments[0]?.env || {};
    this._baseEnv = {
      airports: JSON.parse(JSON.stringify(data.airports || seg0Env.airports || [])),
      sams: JSON.parse(JSON.stringify(data.sams || seg0Env.sams || [])),
      drone_configs: JSON.parse(JSON.stringify(data.drone_configs || seg0Env.drone_configs || {}))
    };

    // Collect all unique targets
    const targetMap = new Map();
    segments.forEach(seg => {
      (seg.env?.targets || []).forEach(t => {
        if (!targetMap.has(t.id)) {
          targetMap.set(t.id, JSON.parse(JSON.stringify(t)));
        }
      });
    });
    this._allTargets = Array.from(targetMap.values());

    // Build segment data
    segments.forEach((seg, idx) => {
      this._segments.push({
        index: idx,
        frozenTargets: seg.frozenTargets ? [...seg.frozenTargets] : [],
        frozenTrajectories: seg.frozenTrajectories ? JSON.parse(JSON.stringify(seg.frozenTrajectories)) : {},
        activeTargets: seg.activeTargets
          ? (seg.env?.targets || []).filter(t => seg.activeTargets.includes(t.id))
          : (seg.env?.targets || []),
        droneConfigs: seg.droneConfigs
          ? JSON.parse(JSON.stringify(seg.droneConfigs))
          : JSON.parse(JSON.stringify(this._baseEnv.drone_configs)),
        startPositions: seg.startPositions ? JSON.parse(JSON.stringify(seg.startPositions)) : null,
        cutDistance: seg.cutDistance ?? 0,
        lostDrones: seg.lostDrones ? [...seg.lostDrones] : [],
        sams: seg.env?.sams ? JSON.parse(JSON.stringify(seg.env.sams)) : []
      });
    });

    this._currentSegmentIndex = 0;
    this._isActive = true;

    console.log(`[SegmentedMissionManager] Loaded ${this._segments.length} segments, ${this._allTargets.length} targets`);

    // Return segment 0's droneConfigs for caller to set state.droneConfigs
    return this.getDroneConfigs(0);
  }

  // =========================================================================
  // FRESH SEGMENTATION (from non-segmented JSON)
  // =========================================================================

  /**
   * Initialize for fresh segmentation workflow
   * @param {Object} env - The loaded environment
   * @param {Object} droneConfigs - Initial drone configs
   */
  initFreshSegmentation(env, droneConfigs) {
    this.clear();

    this._baseEnv = {
      airports: JSON.parse(JSON.stringify(env.airports || [])),
      sams: JSON.parse(JSON.stringify(env.sams || [])),
      drone_configs: JSON.parse(JSON.stringify(droneConfigs))
    };

    this._allTargets = JSON.parse(JSON.stringify(env.targets || []));

    // Create segment 0 with all targets active
    this._segments.push({
      index: 0,
      frozenTargets: [],
      frozenTrajectories: {},
      activeTargets: JSON.parse(JSON.stringify(this._allTargets)),
      droneConfigs: JSON.parse(JSON.stringify(droneConfigs)),
      startPositions: null,
      cutDistance: 0,
      lostDrones: [],
      sams: JSON.parse(JSON.stringify(env.sams || []))
    });

    this._currentSegmentIndex = 0;
    this._isActive = true;

    console.log(`[SegmentedMissionManager] Fresh segmentation initialized: ${this._allTargets.length} targets`);
  }

  /**
   * Create next segment after a cut
   * @param {Object} params - Cut parameters
   * @param {Array} params.visitedTargets - Targets visited so far
   * @param {Object} params.cutPositions - Per-drone cut positions
   * @param {number} params.cutDistance - Distance at cut point
   * @param {Object} params.frozenTrajectories - Trajectory portions to freeze
   * @param {Object} params.droneConfigs - Current drone configs at cut time
   * @param {Array} params.lostDrones - Drones disabled at this cut
   */
  createNextSegment(params) {
    const {
      visitedTargets,
      cutPositions,
      cutDistance,
      frozenTrajectories,
      droneConfigs,
      lostDrones
    } = params;

    const frozenSet = new Set(visitedTargets);
    const activeTargets = this._allTargets.filter(t => !frozenSet.has(t.id));

    // Merge frozen trajectories from previous segments
    const prevFrozen = this._currentSegmentIndex > 0
      ? this.getFrozenTrajectories(this._currentSegmentIndex)
      : {};

    const mergedFrozen = { ...prevFrozen };
    Object.entries(frozenTrajectories || {}).forEach(([droneId, traj]) => {
      if (!mergedFrozen[droneId]) {
        mergedFrozen[droneId] = [];
      }
      mergedFrozen[droneId] = [...mergedFrozen[droneId], ...traj];
    });

    const newSegment = {
      index: this._segments.length,
      frozenTargets: [...visitedTargets],
      frozenTrajectories: mergedFrozen,
      activeTargets: JSON.parse(JSON.stringify(activeTargets)),
      droneConfigs: JSON.parse(JSON.stringify(droneConfigs)),
      startPositions: cutPositions ? JSON.parse(JSON.stringify(cutPositions)) : null,
      cutDistance: cutDistance ?? 0,
      lostDrones: lostDrones ? [...lostDrones] : [],
      sams: this._baseEnv?.sams ? JSON.parse(JSON.stringify(this._baseEnv.sams)) : []
    };

    this._segments.push(newSegment);
    this._currentSegmentIndex = newSegment.index;

    console.log(`[SegmentedMissionManager] Created segment ${newSegment.index}: ${activeTargets.length} active targets, ${visitedTargets.length} frozen`);

    return newSegment;
  }

  /**
   * Update current segment's droneConfigs (after solve/accept)
   */
  updateSegmentDroneConfigs(droneConfigs) {
    const seg = this._segments[this._currentSegmentIndex];
    if (seg) {
      seg.droneConfigs = JSON.parse(JSON.stringify(droneConfigs));
    }
  }
}

// =============================================================================
// EXPORT FUNCTIONS
// =============================================================================

/**
 * Export segmented mission to JSON
 * @param {MissionReplay} missionReplay - The mission replay instance
 * @param {SegmentedMissionManager} segmentManager - The segment manager
 * @param {Object} baseEnv - Base environment (airports, sams)
 * @param {Object} baseDroneConfigs - Original drone configs (all enabled)
 */
function exportSegmentedMission(missionReplay, segmentManager, baseEnv, baseDroneConfigs) {
  const segmentCount = missionReplay.getSegmentCount();

  if (segmentCount === 0) {
    console.warn('[Export] No segments to export');
    return null;
  }

  // Build cumulative state as we iterate through segments
  const cumulativeFrozenTargets = [];
  const cumulativeFrozenTrajectories = {};

  const segments = [];

  for (let i = 0; i < segmentCount; i++) {
    const seg = missionReplay.getSegment(i);
    if (!seg) continue;

    // 1. Frozen targets for THIS segment = cumulative from previous
    const frozenTargets = [...cumulativeFrozenTargets];

    // 2. Frozen trajectories for THIS segment = cumulative from previous
    const frozenTrajectories = JSON.parse(JSON.stringify(cumulativeFrozenTrajectories));

    // 3. Active targets = targets in this segment's solution
    const activeTargets = (seg.env?.targets || []).map(t => t.id);

    // 4. Drone configs - use segment's stored configs
    const segmentDroneConfigs = seg.droneConfigs
      ? JSON.parse(JSON.stringify(seg.droneConfigs))
      : JSON.parse(JSON.stringify(baseDroneConfigs));

    // 5. Start positions
    const startPositions = i === 0 ? null : (seg.cutPositions || null);

    // Build segment export object
    segments.push({
      index: i,
      frozenTargets,
      frozenTrajectories,
      activeTargets,
      droneConfigs: segmentDroneConfigs,
      startPositions,
      env: seg.env,
      cutDistance: seg.cutDistance || null,
      cutPositions: seg.cutPositions || null,
      lostDrones: seg.lostDrones || null,
      visitedTargets: seg.visitedTargets || null
    });

    // Update cumulative state for NEXT segment
    if (seg.visitedTargets) {
      seg.visitedTargets.forEach(t => {
        if (!cumulativeFrozenTargets.includes(t)) {
          cumulativeFrozenTargets.push(t);
        }
      });
    }

    if (seg.solution?.routes) {
      Object.entries(seg.solution.routes).forEach(([droneId, routeData]) => {
        const traj = routeData.trajectory || [];
        if (traj.length > 0) {
          if (!cumulativeFrozenTrajectories[droneId]) {
            cumulativeFrozenTrajectories[droneId] = [];
          }
          cumulativeFrozenTrajectories[droneId] = [
            ...cumulativeFrozenTrajectories[droneId],
            ...traj
          ];
        }
      });
    }
  }

  // Build final export object
  const exportData = {
    schema: "isr_env_v2",
    is_segmented: true,
    segment_count: segmentCount,
    drone_configs: JSON.parse(JSON.stringify(baseDroneConfigs)),
    airports: JSON.parse(JSON.stringify(baseEnv?.airports || [])),
    sams: JSON.parse(JSON.stringify(baseEnv?.sams || [])),
    segments
  };

  return exportData;
}

/**
 * Export single environment (non-segmented) to JSON
 */
function exportSingleEnvironment(env) {
  return {
    schema: "isr_env_v1",
    is_segmented: false,
    segment_count: 1,
    env: JSON.parse(JSON.stringify(env))
  };
}

// =============================================================================
// IMPORT FUNCTIONS
// =============================================================================

/**
 * Detect if JSON is segmented mission format
 */
function isSegmentedJson(data) {
  return (
    (data.schema === "isr_env_v2" && data.is_segmented === true) ||
    (data.segments && Array.isArray(data.segments) && data.segments.length > 0)
  );
}

/**
 * Import segmented mission from JSON
 * Returns { env, droneConfigs, segmentCount }
 */
function importSegmentedMission(data, segmentManager) {
  const initialDroneConfigs = segmentManager.loadFromSegmentsArray(data);

  if (!initialDroneConfigs) {
    return null;
  }

  const displayEnv = segmentManager.getEnvForDisplay();

  return {
    env: displayEnv,
    droneConfigs: initialDroneConfigs,
    segmentCount: segmentManager.getTotalSegments()
  };
}

/**
 * Import plain environment from JSON
 */
function importPlainEnvironment(data) {
  let env;

  if (data.schema === "isr_env_v1" && data.env) {
    env = data.env;
  } else if (data.environment) {
    env = data.environment;
  } else {
    env = data;
  }

  // Extract drone configs or create defaults
  const droneConfigs = {};
  const savedConfigs = env.drone_configs || {};

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const saved = savedConfigs[idStr] || {};

    droneConfigs[idStr] = {
      enabled: saved.enabled !== undefined ? saved.enabled : true,
      fuel_budget: saved.fuel_budget ?? 300,
      start_airport: saved.start_airport || `A${did}`,
      end_airport: saved.end_airport || `A${did}`,
      target_access: saved.target_access || { a: true, b: true, c: true, d: true, e: true }
    };
  }

  return {
    env: JSON.parse(JSON.stringify(env)),
    droneConfigs
  };
}

// =============================================================================
// SOLVER INTEGRATION
// =============================================================================

/**
 * Build environment for solver
 * @param {Object} state - Global state object
 * @param {SegmentedMissionManager} segmentManager - Segment manager
 * @param {boolean} isCheckpoint - Is this a checkpoint replan?
 */
function buildSolverEnv(state, segmentManager, isCheckpoint) {
  // ALWAYS use state.droneConfigs - it's the source of truth
  const droneConfigs = JSON.parse(JSON.stringify(state.droneConfigs));

  let env;
  let isCheckpointReplan = false;

  if (segmentManager.isActive()) {
    // Segmented workflow - use manager's filtered env
    env = segmentManager.getEnvForSolver(droneConfigs);
    isCheckpointReplan = segmentManager.getCurrentSegmentIndex() > 0;
  } else if (isCheckpoint && state.pendingCutPositions) {
    // Fresh segmentation checkpoint
    env = JSON.parse(JSON.stringify(state.env));
    env.drone_configs = droneConfigs;
    env.synthetic_starts = {};

    // Filter out visited targets
    const visitedSet = new Set(state.visitedTargets || []);
    env.targets = (env.targets || []).filter(t => !visitedSet.has(t.id));

    // Add synthetic starts from cut positions
    Object.entries(state.pendingCutPositions).forEach(([droneId, pos]) => {
      if (pos && Array.isArray(pos) && pos.length === 2) {
        const nodeId = `D${droneId}_START`;
        env.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };

        if (droneConfigs[droneId]) {
          droneConfigs[droneId].start_airport = nodeId;
        }
      }
    });

    isCheckpointReplan = true;
  } else {
    // Fresh solve
    env = JSON.parse(JSON.stringify(state.env));
    env.drone_configs = droneConfigs;
  }

  // Log disabled drones
  const disabledDrones = Object.entries(droneConfigs)
    .filter(([, cfg]) => !cfg.enabled)
    .map(([did]) => `D${did}`);

  if (disabledDrones.length > 0) {
    console.log(`[buildSolverEnv] Disabled drones: ${disabledDrones.join(", ")}`);
  }

  return {
    env,
    droneConfigs,
    isCheckpointReplan
  };
}

// =============================================================================
// ACCEPT SOLUTION
// =============================================================================

/**
 * Accept current solution and save to segment
 * NEVER modifies state.droneConfigs - only reads it
 */
function acceptSolution(state, missionState, missionReplay, segmentManager) {
  const solution = missionState.draftSolution;
  if (!solution) {
    console.error('[acceptSolution] No draft solution to accept');
    return null;
  }

  // Use optimized routes from state.routes if available
  const solutionToStore = JSON.parse(JSON.stringify(solution));
  if (state.routes && Object.keys(state.routes).length > 0) {
    solutionToStore.routes = JSON.parse(JSON.stringify(state.routes));
  }

  // Create segment with CURRENT state.droneConfigs
  const segmentData = {
    solution: solutionToStore,
    env: JSON.parse(JSON.stringify(state.env)),
    droneConfigs: JSON.parse(JSON.stringify(state.droneConfigs)), // Save current UI state
    cutDistance: state.pendingCutDistance || null,
    cutPositions: state.pendingCutPositions ? JSON.parse(JSON.stringify(state.pendingCutPositions)) : null,
    lostDrones: state.pendingLostDrones ? [...state.pendingLostDrones] : [],
    visitedTargets: state.visitedTargets ? [...state.visitedTargets] : []
  };

  // Store in segment.env.drone_configs too for export compatibility
  segmentData.env.drone_configs = JSON.parse(JSON.stringify(state.droneConfigs));

  // Add or replace segment
  let segment;
  const currentIdx = missionReplay.getCurrentSegmentIndex();
  const existingSegment = missionReplay.getSegment(currentIdx);

  if (existingSegment && Object.keys(existingSegment.solution?.routes || {}).length === 0) {
    // Replace placeholder segment
    segment = missionReplay.replaceSegment(currentIdx, segmentData);
  } else {
    // Add new segment
    segment = missionReplay.addSegment(segmentData);
  }

  // Update segment manager if active
  if (segmentManager.isActive()) {
    segmentManager.updateSegmentDroneConfigs(state.droneConfigs);
  }

  // Clear draft
  missionState.draftSolution = null;

  console.log(`[acceptSolution] Saved segment ${segment.index} with droneConfigs:`,
    Object.entries(state.droneConfigs).map(([d, c]) => `D${d}:${c.enabled}`).join(", "));

  return segment;
}

// =============================================================================
// UI SYNC
// =============================================================================

/**
 * Sync UI checkboxes with state.droneConfigs
 * Call this ONCE after import, not during workflow
 */
function syncUiWithDroneConfigs(droneConfigs) {
  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const cfg = droneConfigs[idStr];
    if (!cfg) continue;

    const cbEnabled = document.getElementById(`cfg-d${did}-enabled`);
    const fuelInput = document.getElementById(`cfg-d${did}-fuel`);
    const startSel = document.getElementById(`cfg-d${did}-start`);
    const endSel = document.getElementById(`cfg-d${did}-end`);

    if (cbEnabled) cbEnabled.checked = cfg.enabled;
    if (fuelInput) fuelInput.value = cfg.fuel_budget;
    if (startSel) startSel.value = cfg.start_airport;
    if (endSel) endSel.value = cfg.end_airport;

    // Target access checkboxes
    ["a", "b", "c", "d", "e"].forEach(t => {
      const cb = document.getElementById(`cfg-d${did}-type-${t}`);
      if (cb && cfg.target_access?.[t] !== undefined) {
        cb.checked = cfg.target_access[t];
      }
    });
  }
}

// =============================================================================
// EXPORTS
// =============================================================================

// Export for use in isr.js
if (typeof window !== 'undefined') {
  window.SegmentedMissionManager = SegmentedMissionManager;
  window.MissionReplayClean = MissionReplay;
  window.SegmentClean = Segment;
  window.exportSegmentedMission = exportSegmentedMission;
  window.exportSingleEnvironment = exportSingleEnvironment;
  window.isSegmentedJson = isSegmentedJson;
  window.importSegmentedMission = importSegmentedMission;
  window.importPlainEnvironment = importPlainEnvironment;
  window.buildSolverEnv = buildSolverEnv;
  window.acceptSolutionClean = acceptSolution;
  window.syncUiWithDroneConfigs = syncUiWithDroneConfigs;
}
