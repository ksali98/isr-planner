/**
 * utils.js - Shared utility functions for segment system
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md
 */

const EPS = 0.001;  // Epsilon for coordinate comparisons
const SAM_MARGIN = 0.5;  // Safety buffer for SAM escape

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
 * @param {number} epsilon - Comparison epsilon (default EPS)
 * @returns {boolean}
 */
function pointsEqual(p1, p2, epsilon = EPS) {
  if (!p1 || !p2) return false;
  return Math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < epsilon;
}

/**
 * Distance between two points
 * @param {number[]} p1 - [x, y]
 * @param {number[]} p2 - [x, y]
 * @returns {number}
 */
function distance(p1, p2) {
  if (!p1 || !p2) return Infinity;
  return Math.hypot(p1[0] - p2[0], p1[1] - p2[1]);
}

/**
 * Interpolate between two points
 * @param {number[]} p1 - [x, y] start
 * @param {number[]} p2 - [x, y] end
 * @param {number} t - Interpolation factor (0-1)
 * @returns {number[]} [x, y]
 */
function lerp(p1, p2, t) {
  return [
    p1[0] + t * (p2[0] - p1[0]),
    p1[1] + t * (p2[1] - p1[1])
  ];
}

/**
 * Get position on a polyline at a given distance from start
 * @param {number[][]} polyline - Array of [x, y] points
 * @param {number} targetDist - Distance from start
 * @returns {{ position: number[], index: number, t: number } | null}
 */
function getPositionOnPolyline(polyline, targetDist) {
  if (!polyline || polyline.length === 0) return null;
  if (polyline.length === 1) {
    return { position: [...polyline[0]], index: 0, t: 0 };
  }
  if (targetDist <= 0) {
    return { position: [...polyline[0]], index: 0, t: 0 };
  }

  let traveled = 0;
  for (let i = 1; i < polyline.length; i++) {
    const segLen = distance(polyline[i - 1], polyline[i]);

    if (traveled + segLen >= targetDist) {
      const remaining = targetDist - traveled;
      const t = segLen > 0 ? remaining / segLen : 0;
      return {
        position: lerp(polyline[i - 1], polyline[i], t),
        index: i - 1,
        t: t
      };
    }

    traveled += segLen;
  }

  // Past end
  return {
    position: [...polyline[polyline.length - 1]],
    index: polyline.length - 2,
    t: 1
  };
}

/**
 * Split a polyline at a given distance
 * @param {number[][]} polyline - Array of [x, y] points
 * @param {number} splitDist - Distance at which to split
 * @returns {{ before: number[][], splitPoint: number[], after: number[][] }}
 */
function splitPolylineAtDistance(polyline, splitDist) {
  if (!polyline || polyline.length === 0) {
    return { before: [], splitPoint: null, after: [] };
  }

  const result = getPositionOnPolyline(polyline, splitDist);
  if (!result) {
    return { before: [...polyline], splitPoint: null, after: [] };
  }

  const { position, index, t } = result;
  const before = polyline.slice(0, index + 1);

  // Add split point if not coincident with existing point
  if (!pointsEqual(before[before.length - 1], position)) {
    before.push(position);
  }

  const after = [position, ...polyline.slice(index + 1)];

  // Clean up after array - remove duplicate start if coincident
  if (after.length > 1 && pointsEqual(after[0], after[1])) {
    after.shift();
  }

  return { before, splitPoint: position, after };
}

/**
 * Remove duplicate consecutive points from a polyline
 * @param {number[][]} polyline
 * @param {number} epsilon
 * @returns {number[][]}
 */
function removeDuplicatePoints(polyline, epsilon = EPS) {
  if (!polyline || polyline.length === 0) return [];
  if (polyline.length === 1) return [[...polyline[0]]];

  const result = [[...polyline[0]]];
  for (let i = 1; i < polyline.length; i++) {
    if (!pointsEqual(polyline[i], result[result.length - 1], epsilon)) {
      result.push([...polyline[i]]);
    }
  }
  return result;
}

/**
 * Concatenate two polylines with de-duplication at join point
 * @param {number[][]} poly1
 * @param {number[][]} poly2
 * @returns {number[][]}
 */
function concatenatePolylines(poly1, poly2) {
  if (!poly1 || poly1.length === 0) return poly2 ? [...poly2] : [];
  if (!poly2 || poly2.length === 0) return [...poly1];

  const result = [...poly1];
  let startIdx = 0;

  // Skip first point of poly2 if it matches last point of poly1
  if (pointsEqual(poly1[poly1.length - 1], poly2[0])) {
    startIdx = 1;
  }

  for (let i = startIdx; i < poly2.length; i++) {
    result.push([...poly2[i]]);
  }

  return result;
}

/**
 * Compute escape point if position is inside a SAM zone
 * @param {number[]} pos - [x, y] position
 * @param {object} sam - { pos: [x, y], range: number }
 * @returns {number[]|null} Escape point or null if not inside SAM
 */
function computeEscapePoint(pos, sam) {
  const [cx, cy] = pos;
  const [sx, sy] = sam.pos;
  const dx = cx - sx;
  const dy = cy - sy;
  const dist = Math.hypot(dx, dy);

  if (dist >= sam.range) {
    return null;  // Not inside SAM
  }

  if (dist < 0.001) {
    // Exactly at SAM center - pick deterministic direction (+x)
    return [sx + sam.range + SAM_MARGIN, sy];
  }

  // Nearest exit point: along radial line from SAM center through position
  const scale = (sam.range + SAM_MARGIN) / dist;
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
 * Clamp a value between min and max
 * @param {number} value
 * @param {number} min
 * @param {number} max
 * @returns {number}
 */
function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    EPS,
    SAM_MARGIN,
    computePolylineLength,
    deepCopy,
    pointsEqual,
    distance,
    lerp,
    getPositionOnPolyline,
    splitPolylineAtDistance,
    removeDuplicatePoints,
    concatenatePolylines,
    computeEscapePoint,
    findEngulfingSam,
    clamp
  };
}

// Also export for ES6 modules / browser
if (typeof window !== 'undefined') {
  window.SegmentUtilsV2 = {
    EPS,
    SAM_MARGIN,
    computePolylineLength,
    deepCopy,
    pointsEqual,
    distance,
    lerp,
    getPositionOnPolyline,
    splitPolylineAtDistance,
    removeDuplicatePoints,
    concatenatePolylines,
    computeEscapePoint,
    findEngulfingSam,
    clamp
  };
}
