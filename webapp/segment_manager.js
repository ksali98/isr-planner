/**
 * SegmentManager - Clean implementation for segmented mission workflows
 *
 * Single source of truth for segment data. Replaces:
 * - SegmentedMissionManager (from segmented_mission.js)
 * - SegmentedImportManager (from isr.js)
 *
 * Design Principles:
 * 1. state.droneConfigs is the UI source of truth - this class never modifies it directly
 * 2. Each segment stores an immutable snapshot of droneConfigs at accept time
 * 3. Import returns initial droneConfigs for caller to set
 * 4. Accept returns next segment's droneConfigs for caller to set
 */

class SegmentManager {
  constructor() {
    this.clear();
  }

  clear() {
    this._segments = [];        // Array of immutable segment objects
    this._baseEnv = null;       // { airports, sams } - never changes after load
    this._allTargets = [];      // All targets across all segments
    this._currentIndex = 0;
    this._isActive = false;
  }

  // ===========================================================================
  // STATE ACCESSORS
  // ===========================================================================

  isActive() {
    return this._isActive;
  }

  getCurrentIndex() {
    return this._currentIndex;
  }

  getTotalSegments() {
    return this._segments.length;
  }

  getCurrentSegment() {
    return this._segments[this._currentIndex] || null;
  }

  getSegment(index) {
    return this._segments[index] || null;
  }

  getAllTargets() {
    return JSON.parse(JSON.stringify(this._allTargets));
  }

  // ===========================================================================
  // IMPORT FROM JSON
  // ===========================================================================

  /**
   * Load segmented mission from JSON
   * @param {Object} data - The parsed JSON data
   * @returns {Object|null} segment 0's droneConfigs for UI initialization, or null on failure
   */
  loadFromJson(data) {
    this.clear();

    const segments = data.segments || [];
    if (segments.length === 0) {
      console.warn('[SegmentManager] No segments in JSON');
      return null;
    }

    console.log(`[SegmentManager] Loading ${segments.length} segments...`);

    // Extract base environment (airports, sams - constant across all segments)
    const seg0Env = segments[0]?.env || {};
    this._baseEnv = {
      airports: JSON.parse(JSON.stringify(data.airports || seg0Env.airports || [])),
      sams: JSON.parse(JSON.stringify(data.sams || seg0Env.sams || []))
    };

    // Collect all unique targets from all segments
    const targetMap = new Map();
    segments.forEach(seg => {
      (seg.env?.targets || []).forEach(t => {
        if (!targetMap.has(t.id)) {
          targetMap.set(t.id, JSON.parse(JSON.stringify(t)));
        }
      });
    });
    this._allTargets = Array.from(targetMap.values());

    // Parse each segment - first pass: read droneConfigs from JSON
    const segmentConfigs = segments.map((seg, idx) =>
      this._getDroneConfigsFromJson(seg, idx)
    );

    // Parse each segment - second pass: create segment objects with lostDrones
    segments.forEach((seg, idx) => {
      const droneConfigs = segmentConfigs[idx];

      // Derive lostDrones by comparing with previous segment's enabled drones
      let lostDrones = seg.lostDrones ? [...seg.lostDrones] : [];
      if (idx > 0 && lostDrones.length === 0) {
        // If no explicit lostDrones, derive from comparing droneConfigs
        const prevConfigs = segmentConfigs[idx - 1];
        lostDrones = [];
        Object.entries(prevConfigs).forEach(([did, cfg]) => {
          if (cfg.enabled && !droneConfigs[did]?.enabled) {
            lostDrones.push(did);
          }
        });
        if (lostDrones.length > 0) {
          console.log(`[SegmentManager] Seg ${idx}: derived lostDrones=[${lostDrones.join(',')}]`);
        }
      }

      this._segments.push({
        index: idx,
        solution: seg.solution ? JSON.parse(JSON.stringify(seg.solution)) : { routes: {}, sequences: {} },
        droneConfigs: droneConfigs,
        frozenTargetIds: seg.frozenTargets ? [...seg.frozenTargets] : [],
        activeTargetIds: seg.activeTargets ? [...seg.activeTargets] : [],
        startPositions: seg.startPositions ? JSON.parse(JSON.stringify(seg.startPositions)) : null,
        cutDistance: seg.cutDistance ?? 0,
        cutPositions: seg.cutPositions ? JSON.parse(JSON.stringify(seg.cutPositions)) : null,
        lostDrones: lostDrones,
        frozenTrajectories: seg.frozenTrajectories ? JSON.parse(JSON.stringify(seg.frozenTrajectories)) : {}
      });

      const enabled = Object.entries(droneConfigs).filter(([,c]) => c.enabled).map(([d]) => `D${d}`).join(',');
      console.log(`[SegmentManager] Segment ${idx}: ${enabled || 'none'} enabled, lostDrones=[${lostDrones.join(',')}]`);
    });

    this._currentIndex = 0;
    this._isActive = true;

    console.log(`[SegmentManager] Loaded ${this._segments.length} segments, ${this._allTargets.length} targets`);

    // Return segment 0's droneConfigs for caller to set in UI
    return this.getDroneConfigsForSegment(0);
  }

  /**
   * Get droneConfigs for a segment directly from JSON
   * No derivation - trust what's stored in the file
   */
  _getDroneConfigsFromJson(seg, idx) {
    // Read stored configs from JSON (check both possible keys)
    const storedConfigs = seg.drone_configs || seg.droneConfigs || {};

    // Build full config object for all 5 drones
    const configs = {};
    for (let d = 1; d <= 5; d++) {
      const did = String(d);
      const stored = storedConfigs[did] || {};
      configs[did] = {
        enabled: stored.enabled ?? false,
        fuel_budget: stored.fuel_budget ?? 150,
        start_airport: stored.start_airport || `A${d}`,
        end_airport: stored.end_airport || `A${d}`,
        target_access: stored.target_access || { a: true, b: true, c: true, d: true, e: true }
      };
    }

    const enabled = Object.entries(configs).filter(([,c]) => c.enabled).map(([d]) => `D${d}`).join(',');
    console.log(`[SegmentManager] Seg ${idx}: loaded from JSON: [${enabled || 'none'}] enabled`);

    return configs;
  }

  // ===========================================================================
  // DATA ACCESSORS
  // ===========================================================================

  getSegment(index) {
    return this._segments[index] || null;
  }

  getDroneConfigsForSegment(index) {
    const seg = this._segments[index];
    if (!seg?.droneConfigs) return null;
    return JSON.parse(JSON.stringify(seg.droneConfigs));
  }

  getStartPositions(index) {
    const seg = this._segments[index];
    return seg?.startPositions ? JSON.parse(JSON.stringify(seg.startPositions)) : null;
  }

  getCutDistance(index) {
    const seg = this._segments[index];
    return seg?.cutDistance ?? 0;
  }

  getCutPositions(index) {
    const seg = this._segments[index];
    return seg?.cutPositions ? JSON.parse(JSON.stringify(seg.cutPositions)) : null;
  }

  getLostDrones(index) {
    const seg = this._segments[index];
    return seg?.lostDrones ? [...seg.lostDrones] : [];
  }

  getFrozenTargetIds(index) {
    const seg = this._segments[index];
    return seg?.frozenTargetIds ? [...seg.frozenTargetIds] : [];
  }

  getActiveTargetIds(index) {
    const seg = this._segments[index];
    return seg?.activeTargetIds ? [...seg.activeTargetIds] : [];
  }

  getFrozenTrajectories(index) {
    const seg = this._segments[index];
    return seg?.frozenTrajectories ? JSON.parse(JSON.stringify(seg.frozenTrajectories)) : {};
  }

  getVisitedTargets(index) {
    // Visited targets = frozen targets from current segment (targets visited BEFORE this segment)
    return this.getFrozenTargetIds(index);
  }

  // ===========================================================================
  // ENVIRONMENT BUILDERS
  // ===========================================================================

  /**
   * Get environment for DISPLAY (drawing the canvas)
   * Includes frozen targets + active targets for current segment
   */
  getDisplayEnv() {
    if (!this._baseEnv) return null;

    const seg = this.getCurrentSegment();
    if (!seg) return null;

    // Get frozen target objects
    const frozenIds = new Set(seg.frozenTargetIds);
    const frozenTargets = this._allTargets.filter(t => frozenIds.has(t.id));

    // Get active target objects
    const activeIds = new Set(seg.activeTargetIds);
    const activeTargets = this._allTargets.filter(t => activeIds.has(t.id));

    return {
      airports: JSON.parse(JSON.stringify(this._baseEnv.airports)),
      sams: JSON.parse(JSON.stringify(this._baseEnv.sams)),
      targets: [...frozenTargets, ...activeTargets],
      drone_configs: JSON.parse(JSON.stringify(seg.droneConfigs))
    };
  }

  /**
   * Get environment for SOLVER
   * Only active targets, with synthetic start positions for segment > 0
   * @param {Object} uiDroneConfigs - Current state.droneConfigs from UI (source of truth)
   */
  getSolverEnv(uiDroneConfigs) {
    if (!this._baseEnv) return null;

    const seg = this.getCurrentSegment();
    if (!seg) return null;

    const env = {
      airports: JSON.parse(JSON.stringify(this._baseEnv.airports)),
      sams: JSON.parse(JSON.stringify(this._baseEnv.sams)),
      targets: this._allTargets.filter(t => seg.activeTargetIds.includes(t.id)),
      drone_configs: JSON.parse(JSON.stringify(uiDroneConfigs))
    };

    // For segment > 0, add synthetic start positions
    if (this._currentIndex > 0 && seg.startPositions) {
      env.synthetic_starts = {};
      Object.entries(seg.startPositions).forEach(([did, pos]) => {
        if (uiDroneConfigs[did]?.enabled && Array.isArray(pos) && pos.length === 2) {
          env.synthetic_starts[did] = [...pos];
        }
      });
    }

    return env;
  }

  /**
   * Get full environment with ALL targets (for export/snapshot)
   */
  getFullEnv() {
    if (!this._baseEnv) return null;

    return {
      airports: JSON.parse(JSON.stringify(this._baseEnv.airports)),
      sams: JSON.parse(JSON.stringify(this._baseEnv.sams)),
      targets: JSON.parse(JSON.stringify(this._allTargets))
    };
  }

  // ===========================================================================
  // NAVIGATION
  // ===========================================================================

  /**
   * Advance to next segment
   * @returns {boolean} true if there are more segments, false if complete
   */
  advanceSegment() {
    if (this._currentIndex < this._segments.length - 1) {
      this._currentIndex++;
      console.log(`[SegmentManager] Advanced to segment ${this._currentIndex}`);
      return true;
    }
    console.log(`[SegmentManager] All segments complete`);
    return false;
  }

  /**
   * Reset to segment 0
   */
  resetToStart() {
    this._currentIndex = 0;
    console.log(`[SegmentManager] Reset to segment 0`);
  }

  /**
   * Jump to specific segment
   */
  setCurrentIndex(index) {
    this._currentIndex = Math.max(0, Math.min(index, this._segments.length - 1));
  }

  // ===========================================================================
  // ACCEPT SOLUTION
  // ===========================================================================

  /**
   * Accept solution for current segment and advance
   * @param {Object} solution - The solver solution { routes, sequences }
   * @param {Object} droneConfigs - Current UI droneConfigs (snapshot to save)
   * @returns {Object|null} Next segment's droneConfigs, or null if all complete
   */
  acceptSolution(solution, droneConfigs) {
    const seg = this._segments[this._currentIndex];
    if (!seg) return null;

    // Update current segment with solution and droneConfigs snapshot
    this._segments[this._currentIndex] = {
      ...seg,
      solution: JSON.parse(JSON.stringify(solution)),
      droneConfigs: JSON.parse(JSON.stringify(droneConfigs))
    };

    console.log(`[SegmentManager] Accepted segment ${this._currentIndex}`);

    // Advance to next segment
    if (this.advanceSegment()) {
      // Return next segment's droneConfigs for UI update
      return this.getDroneConfigsForSegment(this._currentIndex);
    }

    // All complete
    return null;
  }

  /**
   * Update segment with additional data (cutPositions, visitedTargets, etc.)
   */
  updateSegment(index, updates) {
    const seg = this._segments[index];
    if (!seg) return;

    this._segments[index] = {
      ...seg,
      ...updates
    };
  }

  // ===========================================================================
  // EXPORT TO JSON
  // ===========================================================================

  /**
   * Export current state to JSON format
   * @param {Object} baseEnvOverride - Optional override for base environment
   * @returns {Object} JSON-serializable object
   */
  exportToJson(baseEnvOverride = null) {
    const baseEnv = baseEnvOverride || this._baseEnv;

    const result = {
      schema: "isr_env_v2",
      is_segmented: true,
      segment_count: this._segments.length,
      airports: JSON.parse(JSON.stringify(baseEnv.airports)),
      sams: JSON.parse(JSON.stringify(baseEnv.sams)),
      drone_configs: this._segments[0]?.droneConfigs
        ? JSON.parse(JSON.stringify(this._segments[0].droneConfigs))
        : {},
      segments: []
    };

    // Build cumulative frozen data as we iterate
    let cumulativeFrozenTargets = [];
    let cumulativeFrozenTrajectories = {};

    this._segments.forEach((seg, idx) => {
      // Build segment export
      const segExport = {
        index: idx,
        frozenTargets: [...cumulativeFrozenTargets],
        frozenTrajectories: JSON.parse(JSON.stringify(cumulativeFrozenTrajectories)),
        activeTargets: [...seg.activeTargetIds],
        droneConfigs: JSON.parse(JSON.stringify(seg.droneConfigs)),
        startPositions: seg.startPositions ? JSON.parse(JSON.stringify(seg.startPositions)) : null,
        cutDistance: seg.cutDistance,
        cutPositions: seg.cutPositions ? JSON.parse(JSON.stringify(seg.cutPositions)) : null,
        lostDrones: [...seg.lostDrones],
        env: {
          airports: JSON.parse(JSON.stringify(baseEnv.airports)),
          sams: JSON.parse(JSON.stringify(baseEnv.sams)),
          targets: this._allTargets.filter(t =>
            cumulativeFrozenTargets.includes(t.id) || seg.activeTargetIds.includes(t.id)
          ),
          // Include drone_configs in env for reliable per-segment configs on re-import
          drone_configs: JSON.parse(JSON.stringify(seg.droneConfigs))
        }
      };

      result.segments.push(segExport);

      // Update cumulative data for next segment
      // Add this segment's solved targets to frozen
      if (seg.solution?.routes) {
        Object.entries(seg.solution.routes).forEach(([did, routeData]) => {
          const route = routeData.route || [];
          route.forEach(wp => {
            if (String(wp).startsWith('T') && !cumulativeFrozenTargets.includes(wp)) {
              cumulativeFrozenTargets.push(wp);
            }
          });

          // Add trajectory to frozen
          const traj = routeData.trajectory || [];
          if (traj.length > 0) {
            if (!cumulativeFrozenTrajectories[did]) {
              cumulativeFrozenTrajectories[did] = [...traj];
            } else {
              // Concatenate, avoiding duplicate junction points
              const existing = cumulativeFrozenTrajectories[did];
              const lastPoint = existing[existing.length - 1];
              const firstPoint = traj[0];
              const isDupe = Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
                            Math.abs(lastPoint[1] - firstPoint[1]) < 0.001;
              cumulativeFrozenTrajectories[did] = isDupe
                ? existing.concat(traj.slice(1))
                : existing.concat(traj);
            }
          }
        });
      }
    });

    return result;
  }

  // ===========================================================================
  // DEBUG
  // ===========================================================================

  getDebugInfo() {
    return {
      isActive: this._isActive,
      currentIndex: this._currentIndex,
      totalSegments: this._segments.length,
      allTargets: this._allTargets.map(t => t.id),
      segments: this._segments.map(seg => ({
        index: seg.index,
        enabledDrones: Object.entries(seg.droneConfigs || {})
          .filter(([,c]) => c.enabled).map(([d]) => `D${d}`),
        activeTargets: seg.activeTargetIds,
        frozenTargets: seg.frozenTargetIds,
        hasSolution: !!seg.solution?.routes && Object.keys(seg.solution.routes).length > 0
      }))
    };
  }
}

// ===========================================================================
// UI SYNC HELPER
// ===========================================================================

/**
 * Sync UI checkboxes/inputs with droneConfigs object
 * Call after import or accept to update UI
 */
function syncUiCheckboxes(droneConfigs) {
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

  console.log('[syncUiCheckboxes] UI synced with droneConfigs');
}

// ===========================================================================
// EXPORTS
// ===========================================================================

if (typeof window !== 'undefined') {
  window.SegmentManager = SegmentManager;
  window.syncUiCheckboxes = syncUiCheckboxes;
  window.syncUiWithDroneConfigs = syncUiCheckboxes;  // Alias for isr.js compatibility
}
