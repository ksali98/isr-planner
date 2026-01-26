/**
 * SegmentIO.js - Import/export functionality for segmented missions
 *
 * Handles:
 * - Export to v2.0 JSON format
 * - Import from v2.0 format
 * - Import from v1.x legacy format (migration)
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md for format specification
 */

// Get builder utilities
const getBuilder = () => {
  if (typeof window !== 'undefined' && window.SegmentBuilderV2) {
    return window.SegmentBuilderV2;
  }
  if (typeof require !== 'undefined') {
    return require('./SegmentBuilder.js');
  }
  throw new Error('SegmentBuilder not available');
};

/**
 * Export mission to v2.0 JSON format
 *
 * @param {MissionReplay} missionReplay - The mission replay instance
 * @param {object} metadata - Optional metadata (mission name, etc.)
 * @returns {object} JSON-serializable mission object
 */
function exportMission(missionReplay, metadata = {}) {
  const segments = missionReplay.getAllSegments();

  return {
    version: '2.0',
    exportedAt: Date.now(),
    metadata: {
      name: metadata.name || 'Untitled Mission',
      description: metadata.description || '',
      ...metadata
    },
    segments: segments.map(seg => ({
      index: seg.index,
      timestamp: seg.timestamp,
      drone_configs: seg.drone_configs,
      waypoints: seg.waypoints,
      targets: seg.targets,
      sams: seg.sams,
      trajectories: seg.trajectories,
      startDist: seg.startDist,
      endDist: seg.endDist,
      cutPositionsAtEnd: seg.cutPositionsAtEnd
    }))
  };
}

/**
 * Export mission to JSON string
 *
 * @param {MissionReplay} missionReplay
 * @param {object} metadata
 * @returns {string} JSON string
 */
function exportMissionToString(missionReplay, metadata = {}) {
  return JSON.stringify(exportMission(missionReplay, metadata), null, 2);
}

/**
 * Import mission from v2.0 JSON format
 *
 * @param {MissionReplay} missionReplay - Target mission replay instance
 * @param {object} data - Parsed JSON data
 * @throws {Error} If format is invalid
 */
function importMissionV2(missionReplay, data) {
  if (!data.segments || !Array.isArray(data.segments)) {
    throw new Error('Invalid v2.0 format: missing segments array');
  }

  missionReplay.clear();

  for (const segment of data.segments) {
    missionReplay.append(segment);
  }
}

/**
 * Convert legacy v1.x segment to v2.0 format
 *
 * Legacy format had:
 * - routes instead of trajectories
 * - cutDistance instead of startDist/endDist
 * - No frozen/active target separation
 * - No synthetic_starts structure
 *
 * @param {object} legacySeg - Legacy segment
 * @param {object|null} prevV2Seg - Previous v2 segment (for boundary computation)
 * @param {number} index - Segment index
 * @returns {object} V2 format segment
 */
function convertLegacySegment(legacySeg, prevV2Seg, index) {
  const builder = getBuilder();

  // Compute startDist/endDist from legacy cutDistance
  const startDist = prevV2Seg?.endDist ?? 0;

  // Build trajectories from legacy routes
  const trajectories = {};
  let maxDelta = 0;

  Object.entries(legacySeg.routes || legacySeg.solution?.routes || {}).forEach(([droneId, routeData]) => {
    const trajectory = routeData.trajectory || [];
    const prevTraj = prevV2Seg?.trajectories?.[droneId];
    const prevFull = prevTraj?.render_full || [];

    // Check if drone is disabled
    const isDisabled = legacySeg.drone_configs?.[droneId]?.enabled === false ||
                       legacySeg.lostDrones?.includes(droneId) ||
                       legacySeg.lostDrones?.includes(parseInt(droneId));

    if (isDisabled) {
      trajectories[droneId] = {
        render_full: prevFull,
        delta: [],
        frozenEndIndex: prevFull.length - 1,
        route: [],
        deltaDistance: 0,
        endState: prevTraj?.endState || null
      };
    } else {
      // De-duplicate splice point
      let delta = [...trajectory];
      if (prevFull.length > 0 && delta.length > 0) {
        if (builder.pointsEqual(prevFull[prevFull.length - 1], delta[0])) {
          delta = delta.slice(1);
        }
      }

      const render_full = prevFull.concat(delta);
      const deltaDistance = builder.computePolylineLength(delta);

      trajectories[droneId] = {
        render_full: render_full,
        delta: delta,
        frozenEndIndex: prevFull.length - 1,
        route: routeData.route || [],
        deltaDistance: deltaDistance,
        endState: {
          position: delta.length > 0 ? delta[delta.length - 1] : (prevFull.length > 0 ? prevFull[prevFull.length - 1] : null),
          fuel_remaining: routeData.fuel_remaining ?? 0
        }
      };

      maxDelta = Math.max(maxDelta, deltaDistance);
    }
  });

  const endDist = startDist + maxDelta;

  // Build synthetic starts from legacy cut positions
  const syntheticStarts = {};
  if (legacySeg.cutPositions || legacySeg.checkpointPositions) {
    const cutPos = legacySeg.cutPositions || legacySeg.checkpointPositions;
    Object.entries(cutPos).forEach(([droneId, pos]) => {
      syntheticStarts[droneId] = {
        id: `D${droneId}_START`,
        x: pos[0],
        y: pos[1]
      };
    });
  }

  // Build targets structure
  const allTargets = legacySeg.env?.targets || [];
  const visitedIds = new Set(legacySeg.visited_targets || legacySeg.visitedTargets || []);

  // If we have previous segment, include its frozen targets
  if (prevV2Seg?.targets?.frozen) {
    prevV2Seg.targets.frozen.forEach(t => visitedIds.add(t.id));
  }

  const frozenTargets = allTargets
    .filter(t => visitedIds.has(t.id))
    .map(t => builder.deepCopy(t));

  const activeTargets = allTargets
    .filter(t => !visitedIds.has(t.id))
    .map(t => builder.deepCopy(t));

  // Build drone configs
  const droneConfigs = builder.deepCopy(legacySeg.drone_configs || legacySeg.droneConfigs || {});

  // Handle lostDrones by setting enabled=false
  (legacySeg.lostDrones || []).forEach(droneId => {
    const did = String(droneId);
    if (droneConfigs[did]) {
      droneConfigs[did].enabled = false;
    }
  });

  // Compute cut positions at end
  const cutPositionsAtEnd = {};
  Object.entries(trajectories).forEach(([droneId, traj]) => {
    if (droneConfigs[droneId]?.enabled !== false && traj.endState?.position) {
      cutPositionsAtEnd[droneId] = [...traj.endState.position];
    }
  });

  return {
    index: index,
    timestamp: legacySeg.timestamp || Date.now(),
    drone_configs: droneConfigs,
    waypoints: {
      airports: builder.deepCopy(legacySeg.env?.airports || []),
      synthetic_starts: syntheticStarts
    },
    targets: {
      frozen: frozenTargets,
      active: activeTargets,
      all: builder.deepCopy(allTargets)
    },
    sams: builder.deepCopy(legacySeg.env?.sams || []),
    trajectories: trajectories,
    startDist: startDist,
    endDist: endDist,
    cutPositionsAtEnd: Object.keys(cutPositionsAtEnd).length > 0 ? cutPositionsAtEnd : null
  };
}

/**
 * Import mission from legacy v1.x format
 *
 * @param {MissionReplay} missionReplay - Target mission replay instance
 * @param {object} data - Parsed JSON data (legacy format)
 */
function importMissionV1(missionReplay, data) {
  const legacySegments = data.segments || [];

  if (legacySegments.length === 0) {
    throw new Error('No segments found in legacy format');
  }

  missionReplay.clear();

  let prevV2Seg = null;

  for (let i = 0; i < legacySegments.length; i++) {
    const legacySeg = legacySegments[i];
    const v2Seg = convertLegacySegment(legacySeg, prevV2Seg, i);
    missionReplay.append(v2Seg);
    prevV2Seg = v2Seg;
  }
}

/**
 * Detect format version from parsed JSON
 *
 * @param {object} data - Parsed JSON data
 * @returns {string} 'v2.0', 'v1.x', or 'unknown'
 */
function detectVersion(data) {
  if (data.version === '2.0') {
    return 'v2.0';
  }

  // Check for v2.0 structure without version tag
  if (data.segments?.[0]?.trajectories && data.segments?.[0]?.startDist !== undefined) {
    return 'v2.0';
  }

  // Check for v1.x structure
  if (data.segments?.[0]?.routes || data.segments?.[0]?.solution?.routes) {
    return 'v1.x';
  }

  // Single segment legacy format
  if (data.routes || data.solution?.routes) {
    return 'v1.x-single';
  }

  return 'unknown';
}

/**
 * Import mission from any supported format
 *
 * @param {MissionReplay} missionReplay - Target mission replay instance
 * @param {object|string} data - JSON data (object or string)
 * @returns {object} { version: string, segmentCount: number }
 */
function importMission(missionReplay, data) {
  // Parse if string
  if (typeof data === 'string') {
    data = JSON.parse(data);
  }

  const version = detectVersion(data);

  switch (version) {
    case 'v2.0':
      importMissionV2(missionReplay, data);
      break;

    case 'v1.x':
      importMissionV1(missionReplay, data);
      break;

    case 'v1.x-single':
      // Wrap single segment in array and import
      importMissionV1(missionReplay, { segments: [data] });
      break;

    default:
      throw new Error(`Unknown mission format. Cannot import.`);
  }

  return {
    version: version,
    segmentCount: missionReplay.count()
  };
}

/**
 * Create a download link for mission export
 *
 * @param {MissionReplay} missionReplay
 * @param {string} filename
 * @param {object} metadata
 */
function downloadMission(missionReplay, filename = 'mission.json', metadata = {}) {
  const jsonStr = exportMissionToString(missionReplay, metadata);
  const blob = new Blob([jsonStr], { type: 'application/json' });
  const url = URL.createObjectURL(blob);

  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    exportMission,
    exportMissionToString,
    importMissionV2,
    convertLegacySegment,
    importMissionV1,
    detectVersion,
    importMission,
    downloadMission
  };
}

// Also export for ES6 modules / browser
if (typeof window !== 'undefined') {
  window.SegmentIOV2 = {
    exportMission,
    exportMissionToString,
    importMissionV2,
    convertLegacySegment,
    importMissionV1,
    detectVersion,
    importMission,
    downloadMission
  };
}
