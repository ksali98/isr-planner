/**
 * Converter.js - Unified converter for migrating to v2 segment format
 *
 * This module provides a single entry point for converting from:
 * 1. Old missionReplay segments
 * 2. SegmentedImport workflow
 * 3. Exported JSON files (v1 format)
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md
 */

/**
 * Get converters from window or require
 */
function getOldFormatConverter() {
  if (typeof window !== 'undefined' && window.OldFormatConverterV2) {
    return window.OldFormatConverterV2;
  }
  if (typeof require !== 'undefined') {
    return require('./OldFormatConverter.js');
  }
  throw new Error('OldFormatConverter not available');
}

function getSegmentedImportConverter() {
  if (typeof window !== 'undefined' && window.SegmentedImportConverterV2) {
    return window.SegmentedImportConverterV2;
  }
  if (typeof require !== 'undefined') {
    return require('./SegmentedImportConverter.js');
  }
  throw new Error('SegmentedImportConverter not available');
}

function getMissionReplay() {
  if (typeof window !== 'undefined' && window.MissionReplayV2) {
    return window.MissionReplayV2;
  }
  if (typeof require !== 'undefined') {
    return require('./MissionReplay.js').MissionReplay;
  }
  throw new Error('MissionReplay not available');
}

/**
 * Detect what kind of data we're dealing with
 *
 * @param {object} options
 * @returns {string} 'missionReplay' | 'segmentedImport' | 'exportedJson' | 'unknown'
 */
function detectSourceType(options) {
  const { missionReplay, segmentedImport, jsonData } = options;

  if (missionReplay && typeof missionReplay.getSegments === 'function') {
    return 'missionReplay';
  }

  if (segmentedImport && typeof segmentedImport.isActive === 'function' && segmentedImport.isActive()) {
    return 'segmentedImport';
  }

  if (jsonData) {
    // Check if it's already v2 format
    if (jsonData.version === '2.0') {
      return 'v2Json';
    }

    // Check for v1 exported format
    if (jsonData.segments && Array.isArray(jsonData.segments)) {
      if (jsonData.segments[0]?.solution?.routes || jsonData.segments[0]?.routes) {
        return 'exportedJson';
      }
    }

    // Check for segmentInfo format
    if (jsonData.type === 'segmented' && jsonData.segmentInfo) {
      return 'segmentInfoJson';
    }
  }

  return 'unknown';
}

/**
 * Convert from any supported source to v2 MissionReplay
 *
 * @param {object} options
 * @param {object} options.missionReplay - Old MissionReplay instance
 * @param {object} options.segmentedImport - SegmentedImportManager instance
 * @param {object} options.jsonData - Exported JSON data
 * @param {object[]} options.solvedRoutes - Solved routes for segmentedImport
 * @returns {MissionReplay} New v2 MissionReplay instance
 */
function convertToV2(options = {}) {
  const MissionReplay = getMissionReplay();
  const oldConverter = getOldFormatConverter();
  const segmentedConverter = getSegmentedImportConverter();

  const sourceType = detectSourceType(options);
  const newMissionReplay = new MissionReplay();

  switch (sourceType) {
    case 'missionReplay': {
      oldConverter.migrateToV2(options.missionReplay, newMissionReplay);
      break;
    }

    case 'segmentedImport': {
      segmentedConverter.migrateSegmentedImportToV2(
        options.segmentedImport,
        newMissionReplay,
        options.solvedRoutes || []
      );
      break;
    }

    case 'v2Json': {
      // Already v2, just import directly
      newMissionReplay.fromJSON(options.jsonData);
      break;
    }

    case 'exportedJson': {
      // Convert v1 exported JSON
      const oldSegments = options.jsonData.segments;
      const v2Segments = oldConverter.convertMissionReplaySegments(
        oldSegments.map((seg, i) => ({
          index: i,
          solution: seg.solution || { routes: seg.routes || {} },
          env: seg.env,
          cutDistance: seg.cutDistance,
          cutPosition: seg.cutPosition,
          cutPositions: seg.cutPositions,
          lostDrones: seg.lostDrones,
          visitedTargets: seg.visitedTargets || seg.visited_targets,
          drone_configs: seg.drone_configs,
          timestamp: seg.timestamp
        }))
      );

      for (const segment of v2Segments) {
        newMissionReplay.append(segment);
      }
      break;
    }

    case 'segmentInfoJson': {
      // This would need a SegmentedImportManager to be created first
      throw new Error('segmentInfoJson format requires SegmentedImportManager. Load into segmentedImport first, then convert.');
    }

    default:
      throw new Error(`Unknown source type: ${sourceType}. Cannot convert.`);
  }

  return newMissionReplay;
}

/**
 * Create a v2 segment from current solving state
 *
 * This is used during the Accept workflow to create a new segment
 * from a solver result.
 *
 * @param {object} options
 * @param {object} options.solverResult - Result from solver
 * @param {object} options.env - Current environment
 * @param {object} options.droneConfigs - Current drone configs
 * @param {string[]} options.visitedTargetIds - IDs of visited targets
 * @param {object|null} options.cutPositions - Cut positions for synthetic starts
 * @param {object|null} options.prevSegment - Previous v2 segment
 * @returns {object} New v2 segment
 */
function createSegmentFromSolver(options) {
  // Import SegmentBuilder
  let SegmentBuilder;
  if (typeof window !== 'undefined' && window.SegmentBuilderV2) {
    SegmentBuilder = window.SegmentBuilderV2;
  } else if (typeof require !== 'undefined') {
    SegmentBuilder = require('./SegmentBuilder.js');
  } else {
    throw new Error('SegmentBuilder not available');
  }

  return SegmentBuilder.buildSegment({
    solverResult: options.solverResult,
    prevSegment: options.prevSegment,
    droneConfigs: options.droneConfigs,
    env: options.env,
    visitedTargetIds: options.visitedTargetIds || [],
    cutPositions: options.cutPositions
  });
}

/**
 * Convert existing state to v2 and create initial segment
 *
 * This is for the case where we have a solved mission but haven't
 * been using the segment system yet.
 *
 * @param {object} options
 * @param {object} options.routes - Current routes from solver
 * @param {object} options.env - Environment
 * @param {object} options.droneConfigs - Drone configs
 * @returns {MissionReplay} New v2 MissionReplay with initial segment
 */
function createInitialMission(options) {
  const MissionReplay = getMissionReplay();
  const newMissionReplay = new MissionReplay();

  const segment = createSegmentFromSolver({
    solverResult: { routes: options.routes },
    prevSegment: null,
    droneConfigs: options.droneConfigs,
    env: options.env,
    visitedTargetIds: [],
    cutPositions: null
  });

  newMissionReplay.append(segment);

  return newMissionReplay;
}

/**
 * Migration helper - check if migration is needed
 *
 * @param {object} missionReplay - Old or new MissionReplay
 * @returns {boolean} True if this is old format needing migration
 */
function needsMigration(missionReplay) {
  if (!missionReplay) return false;

  // Check if it has the new v2 structure
  const segment = missionReplay.getSegment?.(0);
  if (!segment) return false;

  // v2 segments have trajectories with render_full/delta
  if (segment.trajectories && segment.trajectories[Object.keys(segment.trajectories)[0]]?.render_full) {
    return false;  // Already v2
  }

  // v1 segments have solution.routes
  if (segment.solution?.routes) {
    return true;  // Needs migration
  }

  return false;
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    detectSourceType,
    convertToV2,
    createSegmentFromSolver,
    createInitialMission,
    needsMigration
  };
}

// Also export for browser
if (typeof window !== 'undefined') {
  window.ConverterV2 = {
    detectSourceType,
    convertToV2,
    createSegmentFromSolver,
    createInitialMission,
    needsMigration
  };
}
