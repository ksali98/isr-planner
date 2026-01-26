/**
 * SegmentedImportConverter.js - Converts segmentedImport workflow data to v2 format
 *
 * The segmentedImport system stored:
 * - _allTargets - all targets for entire mission
 * - _segmentTargets - targets per segment (for progressive reveal)
 * - _baseEnv - base environment
 * - _segmentCuts - array of { cutDistance, visitedTargets, dronePositions }
 * - _segmentDroneConfigs - drone configs per segment
 *
 * This converter creates v2 segments from this data structure.
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md
 */

/**
 * Get utilities from window or require
 */
function getUtils() {
  if (typeof window !== 'undefined' && window.SegmentUtilsV2) {
    return window.SegmentUtilsV2;
  }
  if (typeof require !== 'undefined') {
    return require('./utils.js');
  }
  throw new Error('Utils not available');
}

/**
 * Convert segmentedImport data to v2 format segments
 *
 * This creates "skeleton" segments that have:
 * - Correct boundaries (startDist/endDist)
 * - Correct drone_configs
 * - Correct targets (frozen/active)
 * - Correct synthetic_starts
 * - EMPTY trajectories (to be filled when solving)
 *
 * @param {SegmentedImportManager} segmentedImport - The segmentedImport instance
 * @param {object[]} solvedRoutes - Optional array of solved routes per segment
 * @returns {object[]} Array of v2 format segments
 */
function convertSegmentedImport(segmentedImport, solvedRoutes = []) {
  const utils = getUtils();
  const v2Segments = [];

  const totalSegments = segmentedImport.getTotalSegments();

  for (let i = 0; i < totalSegments; i++) {
    const prevV2 = i > 0 ? v2Segments[i - 1] : null;

    const v2Seg = convertSegmentedImportSegment(
      segmentedImport,
      i,
      prevV2,
      solvedRoutes[i] || null,
      utils
    );

    v2Segments.push(v2Seg);
  }

  return v2Segments;
}

/**
 * Convert a single segment from segmentedImport
 *
 * @param {SegmentedImportManager} segmentedImport
 * @param {number} segmentIndex
 * @param {object|null} prevV2
 * @param {object|null} solvedRoutes - Solved routes for this segment if available
 * @param {object} utils
 * @returns {object} V2 format segment
 */
function convertSegmentedImportSegment(segmentedImport, segmentIndex, prevV2, solvedRoutes, utils) {
  // Get cut distance
  const cutDistance = segmentedImport.getCutDistanceForSegment(segmentIndex);

  // Determine startDist/endDist
  const startDist = prevV2?.endDist ?? 0;

  // Get drone configs for this segment
  const droneConfigs = buildDroneConfigsFromSegmentedImport(segmentedImport, segmentIndex);

  // Get cut positions for synthetic starts
  const cutPositions = segmentedImport.getCutPositionForSegment(segmentIndex);
  const syntheticStarts = buildSyntheticStartsFromCutPositions(cutPositions, segmentIndex);

  // Build targets structure
  const { frozen, active, all } = buildTargetsFromSegmentedImport(segmentedImport, segmentIndex, prevV2);

  // Get environment
  const fullEnv = segmentedImport.getFullEnv() || {};

  // Build trajectories (empty or from solved routes)
  const { trajectories, maxEndDist } = buildTrajectoriesFromSolvedRoutes(
    solvedRoutes,
    prevV2,
    droneConfigs,
    startDist,
    utils
  );

  // For unsolved segments, endDist should equal startDist (no distance)
  // For solved segments, use maxEndDist
  const endDist = solvedRoutes ? maxEndDist : startDist;

  // Build cut positions at end
  const cutPositionsAtEnd = solvedRoutes
    ? buildCutPositionsAtEnd(trajectories, droneConfigs)
    : null;

  return {
    index: segmentIndex,
    timestamp: Date.now(),
    drone_configs: droneConfigs,
    waypoints: {
      airports: utils.deepCopy(fullEnv.airports || []),
      synthetic_starts: syntheticStarts
    },
    targets: {
      frozen: frozen,
      active: active,
      all: all
    },
    sams: utils.deepCopy(fullEnv.sams || []),
    trajectories: trajectories,
    startDist: startDist,
    endDist: endDist,
    cutPositionsAtEnd: cutPositionsAtEnd
  };
}

/**
 * Build drone configs from segmentedImport
 */
function buildDroneConfigsFromSegmentedImport(segmentedImport, segmentIndex) {
  const utils = getUtils();

  // Get segment-specific configs if available
  const segmentConfigs = segmentedImport.getDroneConfigsForSegment?.(segmentIndex);

  if (segmentConfigs) {
    const configs = utils.deepCopy(segmentConfigs);

    // Ensure all configs have enabled field
    Object.keys(configs).forEach(did => {
      if (configs[did].enabled === undefined) {
        configs[did].enabled = true;
      }
    });

    return configs;
  }

  // Fall back to base env configs
  const baseEnv = segmentedImport.getFullEnv?.() || {};
  const baseConfigs = utils.deepCopy(baseEnv.drone_configs || {});

  Object.keys(baseConfigs).forEach(did => {
    if (baseConfigs[did].enabled === undefined) {
      baseConfigs[did].enabled = true;
    }
  });

  return baseConfigs;
}

/**
 * Build synthetic starts from cut positions
 */
function buildSyntheticStartsFromCutPositions(cutPositions, segmentIndex) {
  const syntheticStarts = {};

  if (segmentIndex === 0 || !cutPositions) {
    return syntheticStarts;
  }

  Object.entries(cutPositions).forEach(([droneId, pos]) => {
    if (pos && pos.length === 2) {
      syntheticStarts[droneId] = {
        id: `D${droneId}_START`,
        x: pos[0],
        y: pos[1]
      };
    }
  });

  return syntheticStarts;
}

/**
 * Build targets structure from segmentedImport
 */
function buildTargetsFromSegmentedImport(segmentedImport, segmentIndex, prevV2) {
  const utils = getUtils();

  // Get visited targets for this segment
  const visitedIds = new Set(segmentedImport.getVisitedTargetsForSegment(segmentIndex) || []);

  // Also include targets frozen in previous v2 segment
  if (prevV2?.targets?.frozen) {
    prevV2.targets.frozen.forEach(t => visitedIds.add(t.id));
  }

  // Get all targets
  const allTargets = segmentedImport.getAllTargets?.() || [];

  // Build frozen and active
  const frozen = [];
  const active = [];

  allTargets.forEach(t => {
    const copy = utils.deepCopy(t);
    if (visitedIds.has(t.id)) {
      frozen.push(copy);
    } else {
      active.push(copy);
    }
  });

  return {
    frozen: frozen,
    active: active,
    all: utils.deepCopy(allTargets)
  };
}

/**
 * Build trajectories from solved routes
 */
function buildTrajectoriesFromSolvedRoutes(solvedRoutes, prevV2, droneConfigs, startDist, utils) {
  const trajectories = {};
  let maxEndDist = startDist;

  if (!solvedRoutes || !solvedRoutes.routes) {
    // No routes - create empty trajectories for all drones
    Object.keys(droneConfigs).forEach(droneId => {
      const prevTraj = prevV2?.trajectories?.[droneId];
      trajectories[droneId] = {
        render_full: prevTraj?.render_full || [],
        delta: [],
        frozenEndIndex: prevTraj?.render_full?.length - 1 ?? -1,
        route: [],
        deltaDistance: 0,
        endState: prevTraj?.endState || null
      };
    });

    return { trajectories, maxEndDist };
  }

  // Build trajectories from solved routes
  Object.entries(solvedRoutes.routes).forEach(([droneId, routeData]) => {
    const config = droneConfigs[droneId];
    const isDisabled = config?.enabled === false;

    const oldTrajectory = routeData.trajectory || [];
    const prevFull = prevV2?.trajectories?.[droneId]?.render_full || [];

    if (isDisabled) {
      trajectories[droneId] = {
        render_full: prevFull.length > 0 ? [...prevFull] : [],
        delta: [],
        frozenEndIndex: prevFull.length - 1,
        route: [],
        deltaDistance: 0,
        endState: prevV2?.trajectories?.[droneId]?.endState || null
      };
    } else {
      let delta = [...oldTrajectory];

      // De-duplicate splice point
      if (prevFull.length > 0 && delta.length > 0) {
        if (utils.pointsEqual(prevFull[prevFull.length - 1], delta[0])) {
          delta = delta.slice(1);
        }
      }

      const render_full = [...prevFull, ...delta];
      const deltaDistance = utils.computePolylineLength(delta);

      const endPosition = delta.length > 0
        ? [...delta[delta.length - 1]]
        : (prevFull.length > 0 ? [...prevFull[prevFull.length - 1]] : null);

      trajectories[droneId] = {
        render_full: render_full,
        delta: delta,
        frozenEndIndex: prevFull.length - 1,
        route: routeData.route || [],
        deltaDistance: deltaDistance,
        endState: {
          position: endPosition,
          fuel_remaining: routeData.fuel_remaining ?? 0
        }
      };

      maxEndDist = Math.max(maxEndDist, startDist + deltaDistance);
    }
  });

  // Handle drones not in routes
  Object.keys(droneConfigs).forEach(droneId => {
    if (!trajectories[droneId]) {
      const prevTraj = prevV2?.trajectories?.[droneId];
      trajectories[droneId] = {
        render_full: prevTraj?.render_full || [],
        delta: [],
        frozenEndIndex: prevTraj?.render_full?.length - 1 ?? -1,
        route: [],
        deltaDistance: 0,
        endState: prevTraj?.endState || null
      };
    }
  });

  return { trajectories, maxEndDist };
}

/**
 * Build cut positions at end from trajectories
 */
function buildCutPositionsAtEnd(trajectories, droneConfigs) {
  const positions = {};

  Object.entries(trajectories).forEach(([droneId, traj]) => {
    const config = droneConfigs[droneId];
    if (config?.enabled !== false && traj.endState?.position) {
      positions[droneId] = [...traj.endState.position];
    }
  });

  return Object.keys(positions).length > 0 ? positions : null;
}

/**
 * Convert segmentedImport to v2 MissionReplay
 *
 * @param {SegmentedImportManager} segmentedImport
 * @param {MissionReplay} newMissionReplay - v2 MissionReplay instance
 * @param {object[]} solvedRoutes - Array of solved routes per segment
 */
function migrateSegmentedImportToV2(segmentedImport, newMissionReplay, solvedRoutes = []) {
  const v2Segments = convertSegmentedImport(segmentedImport, solvedRoutes);

  newMissionReplay.clear();

  for (const segment of v2Segments) {
    newMissionReplay.append(segment);
  }

  // Set current to the segment that was being worked on
  const currentIdx = segmentedImport.getCurrentSegmentIndex();
  if (currentIdx > 0 && currentIdx < newMissionReplay.count()) {
    newMissionReplay.setCurrent(currentIdx);
  }
}

/**
 * Create a v2 segment from segmentedImport when a segment is solved
 *
 * This is used during the solving workflow to create each segment
 * as it's accepted.
 *
 * @param {SegmentedImportManager} segmentedImport
 * @param {number} segmentIndex
 * @param {object} solverResult - Result from solver
 * @param {object|null} prevV2 - Previous v2 segment
 * @returns {object} V2 format segment
 */
function createV2SegmentFromSolverResult(segmentedImport, segmentIndex, solverResult, prevV2) {
  const utils = getUtils();

  return convertSegmentedImportSegment(
    segmentedImport,
    segmentIndex,
    prevV2,
    solverResult,
    utils
  );
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    convertSegmentedImport,
    convertSegmentedImportSegment,
    migrateSegmentedImportToV2,
    createV2SegmentFromSolverResult,
    buildDroneConfigsFromSegmentedImport,
    buildSyntheticStartsFromCutPositions,
    buildTargetsFromSegmentedImport,
    buildTrajectoriesFromSolvedRoutes,
    buildCutPositionsAtEnd
  };
}

// Also export for browser
if (typeof window !== 'undefined') {
  window.SegmentedImportConverterV2 = {
    convertSegmentedImport,
    convertSegmentedImportSegment,
    migrateSegmentedImportToV2,
    createV2SegmentFromSolverResult,
    buildDroneConfigsFromSegmentedImport,
    buildSyntheticStartsFromCutPositions,
    buildTargetsFromSegmentedImport,
    buildTrajectoriesFromSolvedRoutes,
    buildCutPositionsAtEnd
  };
}
