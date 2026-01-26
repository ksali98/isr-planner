/**
 * SegmentBuilder.js - Builds segments from solver results
 *
 * Handles:
 * - Trajectory concatenation with de-duplication
 * - Disabled drone invariants
 * - SAM engulfment escape point calculation
 * - Synthetic start generation
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md for full specification
 */

const EPS_BUILDER = window.SegmentUtilsV2?.EPS || 0.001;  // Use from utils.js if available
const SAM_MARGIN_BUILDER = window.SegmentUtilsV2?.SAM_MARGIN || 0.5;  // Use from utils.js if available

/**
 * Compute the length of a polyline
 * @param {number[][]} points - Array of [x, y] points
 * @returns {number} Total length
 */
function computePolylineLength(points) {
  if (!points || points.length < 2) return 0;
  let length = 0;
  for (let i = 1; i < points.length; i++) {
    length += Math.hypot(
      points[i][0] - points[i - 1][0],
      points[i][1] - points[i - 1][1]
    );
  }
  return length;
}

/**
 * Deep copy an object (JSON serializable)
 * @param {*} obj
 * @returns {*}
 */
function deepCopy(obj) {
  if (obj === null || obj === undefined) return obj;
  return JSON.parse(JSON.stringify(obj));
}

/**
 * Check if two points are equal within epsilon
 * @param {number[]} p1 - [x, y]
 * @param {number[]} p2 - [x, y]
 * @returns {boolean}
 */
function pointsEqual(p1, p2) {
  if (!p1 || !p2) return false;
  return Math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < EPS_BUILDER;
}

/**
 * Compute escape point if cut position is inside a SAM zone
 *
 * @param {number[]} cutPos - [x, y] cut position
 * @param {object} sam - { pos: [x, y], range: number }
 * @returns {number[]|null} Escape point [x, y] or null if not inside SAM
 */
function computeEscapePoint(cutPos, sam) {
  const [cx, cy] = cutPos;
  const [sx, sy] = sam.pos;
  const dx = cx - sx;
  const dy = cy - sy;
  const dist = Math.hypot(dx, dy);

  if (dist >= sam.range) {
    // Not inside SAM, no escape needed
    return null;
  }

  if (dist < 0.001) {
    // Exactly at SAM center - pick deterministic direction (+x)
    return [sx + sam.range + SAM_MARGIN_BUILDER, sy];
  }

  // Nearest exit point: along radial line from SAM center through cut position
  const scale = (sam.range + SAM_MARGIN_BUILDER) / dist;
  return [sx + dx * scale, sy + dy * scale];
}

/**
 * Check if a point is inside any SAM zone
 * @param {number[]} pos - [x, y]
 * @param {object[]} sams - Array of SAM objects
 * @returns {object|null} The engulfing SAM or null
 */
function findEngulfingSam(pos, sams) {
  if (!sams || !pos) return null;

  for (const sam of sams) {
    const dist = Math.hypot(pos[0] - sam.pos[0], pos[1] - sam.pos[1]);
    if (dist < sam.range) {
      return sam;
    }
  }
  return null;
}

/**
 * Build synthetic starts for a segment
 *
 * @param {object} cutPositions - { droneId: [x, y] } positions at cut
 * @param {object[]} sams - Current SAM configuration
 * @param {object} droneConfigs - Drone configurations
 * @returns {object} { droneId: { id, x, y, cut_position? } }
 */
function buildSyntheticStarts(cutPositions, sams, droneConfigs) {
  const syntheticStarts = {};

  if (!cutPositions) return syntheticStarts;

  Object.entries(cutPositions).forEach(([droneId, cutPos]) => {
    const config = droneConfigs?.[droneId];
    if (config?.enabled === false) {
      // Disabled drones don't get synthetic starts
      return;
    }

    const syntheticId = `D${droneId}_START`;
    let startX = cutPos[0];
    let startY = cutPos[1];
    let originalCutPos = null;

    // Check for SAM engulfment
    const engulfingSam = findEngulfingSam(cutPos, sams);
    if (engulfingSam) {
      const escapePoint = computeEscapePoint(cutPos, engulfingSam);
      if (escapePoint) {
        startX = escapePoint[0];
        startY = escapePoint[1];
        originalCutPos = cutPos;  // Store original for display
      }
    }

    syntheticStarts[droneId] = {
      id: syntheticId,
      x: startX,
      y: startY
    };

    if (originalCutPos) {
      syntheticStarts[droneId].cut_position = originalCutPos;
    }
  });

  return syntheticStarts;
}

/**
 * Build trajectories for a new segment from solver results
 *
 * @param {object} solverResult - { routes: { droneId: { trajectory, route, fuel_remaining } } }
 * @param {object|null} prevSegment - Previous segment (for render_full concatenation)
 * @param {object} droneConfigs - Current drone configurations
 * @param {number} startDist - Where this segment starts (global distance)
 * @returns {object} { trajectories, maxEndDist }
 */
function buildTrajectories(solverResult, prevSegment, droneConfigs, startDist) {
  const trajectories = {};
  let maxEndDist = startDist;

  Object.entries(solverResult.routes || {}).forEach(([droneId, routeData]) => {
    const config = droneConfigs?.[droneId];
    const newDelta = routeData.trajectory || [];

    if (config?.enabled === false) {
      // ========================================
      // DISABLED DRONE INVARIANTS
      // ========================================
      // Must not extend trajectory, route must be empty
      const prevTraj = prevSegment?.trajectories?.[droneId];

      trajectories[droneId] = {
        render_full: prevTraj?.render_full || [],
        delta: [],                    // NO delta for disabled drone
        frozenEndIndex: prevTraj?.render_full?.length - 1 ?? -1,
        route: [],                    // Empty route
        deltaDistance: 0,             // Zero distance
        endState: prevTraj?.endState || null
      };
    } else {
      // ========================================
      // ENABLED DRONE: Build render_full with de-duplication
      // ========================================
      const prevFull = prevSegment?.trajectories?.[droneId]?.render_full || [];

      // DE-DUPLICATE splice point
      let delta = [...newDelta];
      if (prevFull.length > 0 && delta.length > 0) {
        const prevLast = prevFull[prevFull.length - 1];
        const newFirst = delta[0];
        if (pointsEqual(prevLast, newFirst)) {
          delta = delta.slice(1);  // Drop duplicate first point
        }
      }

      // DE-DUPLICATE consecutive points within delta
      // Solver may return trajectories with duplicate consecutive points
      if (delta.length > 1) {
        const dedupedDelta = [delta[0]];
        for (let i = 1; i < delta.length; i++) {
          if (!pointsEqual(delta[i], delta[i - 1])) {
            dedupedDelta.push(delta[i]);
          }
        }
        delta = dedupedDelta;
      }

      const render_full = prevFull.concat(delta);
      const deltaDistance = computePolylineLength(delta);

      // Determine end position
      const endPosition = delta.length > 0
        ? delta[delta.length - 1]
        : (prevFull.length > 0 ? prevFull[prevFull.length - 1] : null);

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

  return { trajectories, maxEndDist };
}

/**
 * Compute cut positions at the end of a segment
 *
 * @param {object} trajectories - { droneId: { endState: { position } } }
 * @param {object} droneConfigs - Drone configurations
 * @returns {object|null} { droneId: [x, y] } or null if segment 0
 */
function computeCutPositions(trajectories, droneConfigs) {
  const positions = {};

  Object.entries(trajectories).forEach(([droneId, traj]) => {
    const config = droneConfigs?.[droneId];
    if (config?.enabled !== false && traj.endState?.position) {
      positions[droneId] = [...traj.endState.position];
    }
  });

  return Object.keys(positions).length > 0 ? positions : null;
}

/**
 * Build a complete segment from solver results
 *
 * @param {object} params
 * @param {object} params.solverResult - Solver output
 * @param {object|null} params.prevSegment - Previous segment
 * @param {object} params.droneConfigs - Current drone configs
 * @param {object} params.env - Environment (airports, targets, sams)
 * @param {string[]} params.visitedTargetIds - IDs of targets visited so far
 * @param {object|null} params.cutPositions - Cut positions for synthetic starts
 * @returns {object} The new segment
 */
function buildSegment({
  solverResult,
  prevSegment,
  droneConfigs,
  env,
  visitedTargetIds = [],
  cutPositions = null
}) {
  const newIndex = prevSegment ? prevSegment.index + 1 : 0;
  const startDist = prevSegment?.endDist ?? 0;

  // Build trajectories with de-duplication
  const { trajectories, maxEndDist } = buildTrajectories(
    solverResult,
    prevSegment,
    droneConfigs,
    startDist
  );

  // Build synthetic starts (handles SAM engulfment)
  const syntheticStarts = buildSyntheticStarts(
    cutPositions,
    env.sams || [],
    droneConfigs
  );

  // Separate targets into frozen and active
  const visitedSet = new Set(visitedTargetIds);
  const allTargets = env.targets || [];

  // Find target by ID from env
  const findTarget = (id) => {
    return allTargets.find(t => t.id === id);
  };

  const frozenTargets = visitedTargetIds
    .map(id => findTarget(id))
    .filter(t => t != null)
    .map(t => deepCopy(t));

  const activeTargets = allTargets
    .filter(t => !visitedSet.has(t.id))
    .map(t => deepCopy(t));

  // Build drone_configs with fuel_remaining from solver result or previous segment
  const finalDroneConfigs = deepCopy(droneConfigs);
  Object.keys(finalDroneConfigs).forEach(droneId => {
    const config = finalDroneConfigs[droneId];
    // Ensure fuel_capacity exists (default to 100 if missing)
    if (typeof config.fuel_capacity !== 'number') {
      config.fuel_capacity = 100;
    }
    // Get fuel_remaining from solver result, or fallback to previous segment's endState, or fuel_capacity
    const solverFuel = solverResult.routes?.[droneId]?.fuel_remaining;
    const prevFuel = prevSegment?.trajectories?.[droneId]?.endState?.fuel_remaining;
    if (typeof solverFuel === 'number') {
      config.fuel_remaining = solverFuel;
    } else if (typeof prevFuel === 'number') {
      config.fuel_remaining = prevFuel;
    } else if (typeof config.fuel_remaining !== 'number') {
      config.fuel_remaining = config.fuel_capacity;  // Default to full tank
    }
  });

  // Build the segment
  const segment = {
    index: newIndex,
    timestamp: Date.now(),
    drone_configs: finalDroneConfigs,
    waypoints: {
      airports: deepCopy(env.airports || []),
      synthetic_starts: syntheticStarts
    },
    targets: {
      frozen: frozenTargets,
      active: activeTargets,
      all: deepCopy(allTargets)
    },
    sams: deepCopy(env.sams || []),
    trajectories: trajectories,
    startDist: startDist,
    endDist: maxEndDist,
    cutPositionsAtEnd: computeCutPositions(trajectories, droneConfigs)
  };

  return segment;
}

/**
 * Get the position on a trajectory at a given distance
 *
 * @param {number[][]} trajectory - Array of [x, y] points
 * @param {number} distance - Distance along trajectory
 * @returns {number[]|null} [x, y] position or null
 */
function getPositionAtDistance(trajectory, distance) {
  if (!trajectory || trajectory.length === 0) return null;
  if (trajectory.length === 1) return [...trajectory[0]];
  if (distance <= 0) return [...trajectory[0]];

  let traveled = 0;
  for (let i = 1; i < trajectory.length; i++) {
    const segmentLength = Math.hypot(
      trajectory[i][0] - trajectory[i - 1][0],
      trajectory[i][1] - trajectory[i - 1][1]
    );

    if (traveled + segmentLength >= distance) {
      // Interpolate within this segment
      const remaining = distance - traveled;
      const t = segmentLength > 0 ? remaining / segmentLength : 0;
      return [
        trajectory[i - 1][0] + t * (trajectory[i][0] - trajectory[i - 1][0]),
        trajectory[i - 1][1] + t * (trajectory[i][1] - trajectory[i - 1][1])
      ];
    }

    traveled += segmentLength;
  }

  // Past end - return last point
  return [...trajectory[trajectory.length - 1]];
}

/**
 * Get cut positions from a segment's trajectories at a given distance from segment start
 *
 * @param {object} segment - The segment
 * @param {number} distanceFromSegmentStart - Distance from segment.startDist
 * @returns {object} { droneId: [x, y] }
 */
function getCutPositionsAtDistance(segment, distanceFromSegmentStart) {
  const positions = {};

  Object.entries(segment.trajectories || {}).forEach(([droneId, traj]) => {
    const config = segment.drone_configs?.[droneId];
    if (config?.enabled === false) return;

    const pos = getPositionAtDistance(traj.delta, distanceFromSegmentStart);
    if (pos) {
      positions[droneId] = pos;
    }
  });

  return positions;
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    EPS: EPS_BUILDER,
    SAM_MARGIN: SAM_MARGIN_BUILDER,
    computePolylineLength,
    deepCopy,
    pointsEqual,
    computeEscapePoint,
    findEngulfingSam,
    buildSyntheticStarts,
    buildTrajectories,
    computeCutPositions,
    buildSegment,
    getPositionAtDistance,
    getCutPositionsAtDistance
  };
}

// Also export for ES6 modules / browser
if (typeof window !== 'undefined') {
  window.SegmentBuilderV2 = {
    EPS: EPS_BUILDER,
    SAM_MARGIN: SAM_MARGIN_BUILDER,
    computePolylineLength,
    deepCopy,
    pointsEqual,
    computeEscapePoint,
    findEngulfingSam,
    buildSyntheticStarts,
    buildTrajectories,
    computeCutPositions,
    buildSegment,
    getPositionAtDistance,
    getCutPositionsAtDistance
  };
}
