/**
 * index.js - Main entry point for segment_v2 module
 *
 * This module provides the v2 segment system for the ISR planner.
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md for full specification
 *
 * Usage in browser (load via script tags):
 *   <script src="segment_v2/utils.js"></script>
 *   <script src="segment_v2/Segment.js"></script>
 *   <script src="segment_v2/MissionReplay.js"></script>
 *   <script src="segment_v2/SegmentBuilder.js"></script>
 *   <script src="segment_v2/SegmentIO.js"></script>
 *   <script src="segment_v2/index.js"></script>
 *
 * Then access via:
 *   window.SegmentSystemV2.MissionReplay
 *   window.SegmentSystemV2.buildSegment()
 *   etc.
 *
 * Usage in Node.js:
 *   const SegmentSystem = require('./segment_v2');
 *   const { MissionReplay, buildSegment } = SegmentSystem;
 */

// For Node.js / CommonJS
if (typeof module !== 'undefined' && module.exports) {
  const utils = require('./utils.js');
  const Segment = require('./Segment.js');
  const { MissionReplay } = require('./MissionReplay.js');
  const SegmentBuilder = require('./SegmentBuilder.js');
  const SegmentIO = require('./SegmentIO.js');

  module.exports = {
    // Utils
    ...utils,

    // Segment validation and creation
    ...Segment,

    // MissionReplay class
    MissionReplay,

    // Builder functions
    ...SegmentBuilder,

    // IO functions
    ...SegmentIO
  };
}

// For browser
if (typeof window !== 'undefined') {
  // Collect all exports from individual modules
  window.SegmentSystemV2 = {
    // Utils
    ...(window.SegmentUtilsV2 || {}),

    // Segment validation and creation
    ...(window.SegmentV2 || {}),

    // MissionReplay class
    MissionReplay: window.MissionReplayV2,

    // Builder functions
    ...(window.SegmentBuilderV2 || {}),

    // IO functions
    ...(window.SegmentIOV2 || {})
  };

  // Log successful load
  console.log('[SegmentSystemV2] Loaded successfully');
}
