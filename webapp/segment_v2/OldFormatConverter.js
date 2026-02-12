/**
 * OldFormatConverter.js - Converts old missionReplay segments to v2 format
 *
 * The old format stored:
 * - solution.routes[droneId].trajectory - segment-only trajectory
 * - cutDistance - cumulative from mission start
 * - No explicit startDist/endDist
 * - No render_full vs delta separation
 *
 * The new v2 format requires:
 * - trajectories[droneId].render_full - full trajectory from mission start
 * - trajectories[droneId].delta - this segment only
 * - startDist/endDist - explicit boundaries
 * - frozenEndIndex - where frozen portion ends
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md
 */

const EPS = 0.001;

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
 * Convert an array of old MissionReplay segments to v2 format
 *
 * @param {object[]} oldSegments - Array of old Segment objects from missionReplay.getSegments()
 * @returns {object[]} Array of v2 format segments
 */
function convertMissionReplaySegments(oldSegments) {
  const utils = getUtils();
  const v2Segments = [];

  for (let i = 0; i < oldSegments.length; i++) {
    const oldSeg = oldSegments[i];
    const prevV2 = i > 0 ? v2Segments[i - 1] : null;

    const v2Seg = convertSingleSegment(oldSeg, prevV2, i, utils);
    v2Segments.push(v2Seg);
  }

  return v2Segments;
}

/**
 * Convert a single old segment to v2 format
 *
 * @param {object} oldSeg - Old segment from missionReplay
 * @param {object|null} prevV2 - Previous v2 segment (for render_full concatenation)
 * @param {number} index - Segment index
 * @param {object} utils - Utility functions
 * @returns {object} V2 format segment
 */
function convertSingleSegment(oldSeg, prevV2, index, utils) {
  // Determine start/end distances
  const startDist = prevV2?.endDist ?? 0;

  // Build drone_configs from old segment
  const droneConfigs = buildDroneConfigs(oldSeg);

  // Build trajectories
  const { trajectories, maxEndDist } = buildTrajectories(
    oldSeg,
    prevV2,
    droneConfigs,
    startDist,
    utils
  );

  // Build synthetic starts from cut positions
  const syntheticStarts = buildSyntheticStarts(oldSeg, index);

  // Build targets structure
  const { frozen, active, all } = buildTargets(oldSeg, prevV2);

  // Build cut positions at end
  const cutPositionsAtEnd = buildCutPositionsAtEnd(trajectories, droneConfigs);

  return {
    index: index,
    timestamp: oldSeg.timestamp ? new Date(oldSeg.timestamp).getTime() : Date.now(),
    drone_configs: droneConfigs,
    waypoints: {
      airports: utils.deepCopy(oldSeg.env?.airports || []),
      synthetic_starts: syntheticStarts
    },
    targets: {
      frozen: frozen,
      active: active,
      all: all
    },
    sams: utils.deepCopy(oldSeg.env?.sams || []),
    trajectories: trajectories,
    startDist: startDist,
    endDist: maxEndDist,
    cutPositionsAtEnd: cutPositionsAtEnd
  };
}

/**
 * Build drone_configs from old segment
 */
function buildDroneConfigs(oldSeg) {
  const utils = getUtils();

  // Prefer segment-level drone_configs if available
  const baseConfigs = oldSeg.drone_configs || oldSeg.env?.drone_configs || {};
  const configs = utils.deepCopy(baseConfigs);

  // Handle lostDrones by setting enabled=false
  const lostDrones = oldSeg.lostDrones || [];
  lostDrones.forEach(droneId => {
    const did = String(droneId);
    if (configs[did]) {
      configs[did].enabled = false;
    }
  });

  // Ensure all configs have enabled field
  Object.keys(configs).forEach(did => {
    if (configs[did].enabled === undefined) {
      configs[did].enabled = true;
    }
  });

  return configs;
}

/**
 * Build trajectories from old segment's solution
 */
function buildTrajectories(oldSeg, prevV2, droneConfigs, startDist, utils) {
  const trajectories = {};
  let maxEndDist = startDist;

  const routes = oldSeg.solution?.routes || {};

  Object.entries(routes).forEach(([droneId, routeData]) => {
    const config = droneConfigs[droneId];
    const isDisabled = config?.enabled === false;

    // Get trajectory from old format
    const oldTrajectory = routeData.trajectory || [];

    // Get previous render_full for concatenation
    const prevFull = prevV2?.trajectories?.[droneId]?.render_full || [];

    if (isDisabled) {
      // DISABLED DRONE: empty delta, preserve render_full
      trajectories[droneId] = {
        render_full: prevFull.length > 0 ? [...prevFull] : [],
        delta: [],
        frozenEndIndex: prevFull.length - 1,
        route: [],
        deltaDistance: 0,
        endState: prevV2?.trajectories?.[droneId]?.endState || null
      };
    } else {
      // ENABLED DRONE: build render_full with de-duplication
      let delta = [...oldTrajectory];

      // De-duplicate splice point
      if (prevFull.length > 0 && delta.length > 0) {
        const prevLast = prevFull[prevFull.length - 1];
        const newFirst = delta[0];
        if (utils.pointsEqual(prevLast, newFirst)) {
          delta = delta.slice(1);
        }
      }

      const render_full = [...prevFull, ...delta];
      const deltaDistance = utils.computePolylineLength(delta);

      // Determine end position
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

  // Handle drones that exist in droneConfigs but not in routes
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
 * Build synthetic starts from old segment's cut positions
 */
function buildSyntheticStarts(oldSeg, index) {
  const syntheticStarts = {};

  if (index === 0) {
    // Segment 0 doesn't have synthetic starts
    return syntheticStarts;
  }

  // Use cutPositions if available, otherwise fall back to cutPosition
  const cutPositions = oldSeg.cutPositions || {};
  const legacyCutPosition = oldSeg.cutPosition;

  if (Object.keys(cutPositions).length > 0) {
    Object.entries(cutPositions).forEach(([droneId, pos]) => {
      if (pos && pos.length === 2) {
        syntheticStarts[droneId] = {
          id: `D${droneId}_START`,
          x: pos[0],
          y: pos[1]
        };
      }
    });
  } else if (legacyCutPosition && legacyCutPosition.length === 2) {
    // Legacy single cut position - assume drone 1
    syntheticStarts['1'] = {
      id: 'D1_START',
      x: legacyCutPosition[0],
      y: legacyCutPosition[1]
    };
  }

  return syntheticStarts;
}

/**
 * Build targets structure from old segment
 */
function buildTargets(oldSeg, prevV2) {
  const utils = getUtils();

  // All targets from env
  const allTargets = oldSeg.env?.targets || [];

  // Visited targets - union of previous frozen + current segment's visited
  const visitedIds = new Set();

  // Add previously frozen targets
  if (prevV2?.targets?.frozen) {
    prevV2.targets.frozen.forEach(t => visitedIds.add(t.id));
  }

  // Add this segment's visited targets
  if (oldSeg.visitedTargets) {
    oldSeg.visitedTargets.forEach(id => visitedIds.add(id));
  }

  // Build frozen and active arrays
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
 * Convert a live missionReplay instance to v2 MissionReplay
 *
 * @param {MissionReplay} oldMissionReplay - The old MissionReplay instance
 * @param {MissionReplay} newMissionReplay - The new v2 MissionReplay instance
 */
function migrateToV2(oldMissionReplay, newMissionReplay) {
  const oldSegments = oldMissionReplay.getSegments();
  const v2Segments = convertMissionReplaySegments(oldSegments);

  newMissionReplay.clear();

  for (const segment of v2Segments) {
    newMissionReplay.append(segment);
  }

  // Restore current index
  const currentIdx = oldMissionReplay.getCurrentSegmentIndex();
  if (currentIdx > 0 && currentIdx < newMissionReplay.count()) {
    newMissionReplay.setCurrent(currentIdx);
  }
}

/**
 * Convert old segment format (from exported JSON) to v2
 * This handles the JSON export format which may have slightly different structure
 *
 * @param {object} exportedSegment - Segment from exported JSON
 * @param {object|null} prevV2 - Previous v2 segment
 * @param {number} index - Segment index
 * @returns {object} V2 format segment
 */
function convertExportedSegment(exportedSegment, prevV2, index) {
  const utils = getUtils();

  // Exported segments have solution.routes structure
  // Create a mock old segment structure
  const mockOldSeg = {
    solution: exportedSegment.solution || { routes: exportedSegment.routes || {} },
    env: exportedSegment.env,
    cutDistance: exportedSegment.cutDistance,
    cutPosition: exportedSegment.cutPosition,
    cutPositions: exportedSegment.cutPositions,
    lostDrones: exportedSegment.lostDrones,
    visitedTargets: exportedSegment.visitedTargets || exportedSegment.visited_targets,
    drone_configs: exportedSegment.drone_configs,
    timestamp: exportedSegment.timestamp
  };

  return convertSingleSegment(mockOldSeg, prevV2, index, utils);
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    convertMissionReplaySegments,
    convertSingleSegment,
    convertExportedSegment,
    migrateToV2,
    buildDroneConfigs,
    buildTrajectories,
    buildSyntheticStarts,
    buildTargets,
    buildCutPositionsAtEnd
  };
}

// Also export for browser
if (typeof window !== 'undefined') {
  window.OldFormatConverterV2 = {
    convertMissionReplaySegments,
    convertSingleSegment,
    convertExportedSegment,
    migrateToV2,
    buildDroneConfigs,
    buildTrajectories,
    buildSyntheticStarts,
    buildTargets,
    buildCutPositionsAtEnd
  };
}
