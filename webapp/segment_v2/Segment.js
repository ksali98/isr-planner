/**
 * Segment.js - Self-contained segment data structure
 *
 * Each segment stores EVERYTHING needed to display and animate that segment.
 * No reconstruction from previous segments required.
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md for full specification
 */

const EPS_SEGMENT = window.SegmentUtilsV2?.EPS || 0.001;  // Use from utils.js if available

/**
 * Validate a trajectory object for a single drone
 * @param {string} droneId
 * @param {object} traj - The trajectory object
 * @param {object} config - The drone config
 * @returns {string[]} Array of error messages (empty if valid)
 */
function validateTrajectory(droneId, traj, config) {
  const errors = [];

  // Required fields
  if (!Array.isArray(traj.render_full)) {
    errors.push(`Drone ${droneId}: render_full must be an array`);
  }
  if (!Array.isArray(traj.delta)) {
    errors.push(`Drone ${droneId}: delta must be an array`);
  }
  if (typeof traj.frozenEndIndex !== 'number') {
    errors.push(`Drone ${droneId}: frozenEndIndex must be a number`);
  }
  if (!Array.isArray(traj.route)) {
    errors.push(`Drone ${droneId}: route must be an array`);
  }
  if (typeof traj.deltaDistance !== 'number') {
    errors.push(`Drone ${droneId}: deltaDistance must be a number`);
  }

  // Skip further validation if basic structure is wrong
  if (errors.length > 0) return errors;

  const isDisabled = config?.enabled === false;

  if (isDisabled) {
    // DISABLED DRONE INVARIANTS
    if (traj.delta.length > 0) {
      errors.push(`Disabled drone ${droneId} must have empty delta`);
    }
    if (traj.route.length > 0) {
      errors.push(`Disabled drone ${droneId} must have empty route`);
    }
    if (traj.deltaDistance !== 0) {
      errors.push(`Disabled drone ${droneId} must have deltaDistance=0`);
    }
  } else {
    // ENABLED DRONE INVARIANTS
    // Note: delta.length can be:
    // - 0: No movement yet (imported segment before Run Planner)
    // - 1: Drone at final position (e.g., at airport, no further movement)
    // - >= 2: Normal trajectory with start and end points
    // All are valid for enabled drones
  }

  // No duplicate consecutive points in delta (within epsilon)
  for (let i = 1; i < traj.delta.length; i++) {
    const d = Math.hypot(
      traj.delta[i][0] - traj.delta[i - 1][0],
      traj.delta[i][1] - traj.delta[i - 1][1]
    );
    if (d < EPS_SEGMENT) {
      errors.push(`Drone ${droneId} delta has duplicate point at index ${i}`);
    }
  }

  // frozenEndIndex consistency
  if (traj.frozenEndIndex >= traj.render_full.length && traj.render_full.length > 0) {
    errors.push(`Drone ${droneId} frozenEndIndex ${traj.frozenEndIndex} out of bounds (render_full.length=${traj.render_full.length})`);
  }

  // endState validation
  if (traj.endState) {
    if (!Array.isArray(traj.endState.position) || traj.endState.position.length !== 2) {
      errors.push(`Drone ${droneId} endState.position must be [x, y]`);
    }
    if (typeof traj.endState.fuel_remaining !== 'number') {
      errors.push(`Drone ${droneId} endState.fuel_remaining must be a number`);
    }
  }

  return errors;
}

/**
 * Validate a complete segment
 * @param {object} segment - The segment to validate
 * @param {object|null} prevSegment - Previous segment (for boundary validation)
 * @returns {string[]} Array of error messages (empty if valid)
 */
function validateSegment(segment, prevSegment = null) {
  const errors = [];

  // ========================================
  // Required fields
  // ========================================
  if (typeof segment.index !== 'number') {
    errors.push('index must be a number');
  }
  if (typeof segment.timestamp !== 'number') {
    errors.push('timestamp must be a number');
  }
  if (!segment.drone_configs || typeof segment.drone_configs !== 'object') {
    errors.push('drone_configs must be an object');
  }
  if (!segment.waypoints || typeof segment.waypoints !== 'object') {
    errors.push('waypoints must be an object');
  }
  if (!segment.targets || typeof segment.targets !== 'object') {
    errors.push('targets must be an object');
  }
  if (!Array.isArray(segment.sams)) {
    errors.push('sams must be an array');
  }
  if (!segment.trajectories || typeof segment.trajectories !== 'object') {
    errors.push('trajectories must be an object');
  }
  if (typeof segment.startDist !== 'number') {
    errors.push('startDist must be a number');
  }
  // endDist can be null (for last/open segment)
  if (segment.endDist !== null && typeof segment.endDist !== 'number') {
    errors.push('endDist must be a number or null');
  }

  // Skip further validation if basic structure is wrong
  if (errors.length > 0) return errors;

  // ========================================
  // Boundary validation
  // ========================================
  if (segment.index > 0) {
    if (prevSegment) {
      if (segment.startDist !== prevSegment.endDist) {
        errors.push(`startDist ${segment.startDist} != prev.endDist ${prevSegment.endDist}`);
      }
      if (segment.index !== prevSegment.index + 1) {
        errors.push(`index ${segment.index} != prev.index + 1 (${prevSegment.index + 1})`);
      }
    }
  } else {
    if (segment.startDist !== 0) {
      errors.push('Segment 0 must have startDist=0');
    }
    if (segment.index !== 0) {
      errors.push('First segment must have index=0');
    }
  }

  if (segment.endDist !== null && segment.endDist <= segment.startDist) {
    errors.push(`endDist ${segment.endDist} must be > startDist ${segment.startDist}`);
  }

  // ========================================
  // Waypoints validation
  // ========================================
  if (!Array.isArray(segment.waypoints.airports)) {
    errors.push('waypoints.airports must be an array');
  }
  if (!segment.waypoints.synthetic_starts || typeof segment.waypoints.synthetic_starts !== 'object') {
    errors.push('waypoints.synthetic_starts must be an object');
  }

  // ========================================
  // Targets validation
  // ========================================
  if (!Array.isArray(segment.targets.frozen)) {
    errors.push('targets.frozen must be an array');
  }
  if (!Array.isArray(segment.targets.active)) {
    errors.push('targets.active must be an array');
  }
  if (!Array.isArray(segment.targets.all)) {
    errors.push('targets.all must be an array');
  }

  // Verify all = frozen + active
  const frozenIds = new Set(segment.targets.frozen.map(t => t.id));
  const activeIds = new Set(segment.targets.active.map(t => t.id));
  const allIds = new Set(segment.targets.all.map(t => t.id));

  // Check for overlap between frozen and active
  frozenIds.forEach(id => {
    if (activeIds.has(id)) {
      errors.push(`Target ${id} appears in both frozen and active`);
    }
  });

  // Check that all contains exactly frozen + active
  const expectedAllIds = new Set([...frozenIds, ...activeIds]);
  if (allIds.size !== expectedAllIds.size) {
    errors.push(`targets.all size (${allIds.size}) != frozen + active (${expectedAllIds.size})`);
  }
  expectedAllIds.forEach(id => {
    if (!allIds.has(id)) {
      errors.push(`Target ${id} missing from targets.all`);
    }
  });

  // ========================================
  // Per-drone trajectory validation
  // ========================================
  Object.entries(segment.trajectories).forEach(([droneId, traj]) => {
    const config = segment.drone_configs[droneId];
    const trajErrors = validateTrajectory(droneId, traj, config);
    errors.push(...trajErrors);
  });

  // ========================================
  // Drone config validation
  // ========================================
  Object.entries(segment.drone_configs).forEach(([droneId, config]) => {
    if (typeof config.enabled !== 'boolean') {
      errors.push(`Drone ${droneId} config.enabled must be a boolean`);
    }
    if (typeof config.fuel_capacity !== 'number') {
      errors.push(`Drone ${droneId} config.fuel_capacity must be a number`);
    }
    if (typeof config.fuel_remaining !== 'number') {
      errors.push(`Drone ${droneId} config.fuel_remaining must be a number`);
    }
  });

  return errors;
}

/**
 * Create a new segment with validation
 * @param {object} data - Segment data
 * @param {object|null} prevSegment - Previous segment for boundary validation
 * @returns {object} The validated segment
 * @throws {Error} If validation fails
 */
function createSegment(data, prevSegment = null) {
  const errors = validateSegment(data, prevSegment);
  if (errors.length > 0) {
    throw new Error(`Invalid segment:\n${errors.join('\n')}`);
  }
  return { ...data };
}

/**
 * Create an empty/default segment for segment 0
 * @param {object} env - Environment with airports, targets, sams
 * @param {object} droneConfigs - Drone configurations
 * @returns {object} A minimal segment 0
 */
function createInitialSegment(env, droneConfigs) {
  const segment = {
    index: 0,
    timestamp: Date.now(),
    drone_configs: JSON.parse(JSON.stringify(droneConfigs)),
    waypoints: {
      airports: JSON.parse(JSON.stringify(env.airports || [])),
      synthetic_starts: {}
    },
    targets: {
      frozen: [],
      active: JSON.parse(JSON.stringify(env.targets || [])),
      all: JSON.parse(JSON.stringify(env.targets || []))
    },
    sams: JSON.parse(JSON.stringify(env.sams || [])),
    trajectories: {},
    startDist: 0,
    endDist: null,
    cutPositionsAtEnd: null
  };

  // Initialize empty trajectories for each drone
  Object.keys(droneConfigs).forEach(droneId => {
    segment.trajectories[droneId] = {
      render_full: [],
      delta: [],
      frozenEndIndex: -1,
      route: [],
      deltaDistance: 0,
      endState: null
    };
  });

  return segment;
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    EPS: EPS_SEGMENT,
    validateTrajectory,
    validateSegment,
    createSegment,
    createInitialSegment
  };
}

// Also export for ES6 modules / browser
if (typeof window !== 'undefined') {
  window.SegmentV2 = {
    EPS: EPS_SEGMENT,
    validateTrajectory,
    validateSegment,
    createSegment,
    createInitialSegment
  };
}
