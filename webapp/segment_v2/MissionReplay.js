/**
 * MissionReplay.js - Single source of truth for segment management
 *
 * This class manages the segment array with controlled access.
 * UI must not mutate internal arrays directly.
 *
 * @see docs/SEGMENT_SYSTEM_REDESIGN.md for full specification
 */

// Import validation (will be available via window.SegmentV2 in browser)
const getValidateSegment = () => {
  if (typeof window !== 'undefined' && window.SegmentV2) {
    return window.SegmentV2.validateSegment;
  }
  if (typeof require !== 'undefined') {
    return require('./Segment.js').validateSegment;
  }
  // Fallback - no validation
  return () => [];
};

/**
 * MissionReplay - Manages segment storage with controlled access
 *
 * Usage rules:
 * - Never expose internal arrays directly
 * - All mutations go through append()
 * - Index changes go through setCurrent() or resetToStart()
 * - UI reads via getSegment(), current(), count()
 */
class MissionReplay {
  constructor() {
    /** @private */
    this._segments = [];
    /** @private */
    this._currentIndex = 0;
  }

  // ========================================
  // READ operations
  // ========================================

  /**
   * Get the number of segments
   * @returns {number}
   */
  count() {
    return this._segments.length;
  }

  /**
   * Check if there are any segments
   * @returns {boolean}
   */
  hasSegments() {
    return this._segments.length > 0;
  }

  /**
   * Get a segment by index
   * @param {number} index
   * @returns {object|null} The segment or null if out of bounds
   */
  getSegment(index) {
    if (index < 0 || index >= this._segments.length) {
      return null;
    }
    return this._segments[index];
  }

  /**
   * Get the first segment
   * @returns {object|null}
   */
  first() {
    return this.getSegment(0);
  }

  /**
   * Get the last segment
   * @returns {object|null}
   */
  last() {
    return this.getSegment(this._segments.length - 1);
  }

  /**
   * Get the current segment (being animated/displayed)
   * @returns {object|null}
   */
  current() {
    return this.getSegment(this._currentIndex);
  }

  /**
   * Get the current segment index
   * @returns {number}
   */
  currentIndex() {
    return this._currentIndex;
  }

  /**
   * Check if currently at the last segment
   * @returns {boolean}
   */
  isAtLastSegment() {
    return this._currentIndex === this._segments.length - 1;
  }

  /**
   * Check if there's a next segment available
   * @returns {boolean}
   */
  hasNextSegment() {
    return this._currentIndex < this._segments.length - 1;
  }

  /**
   * Get all segments (readonly iteration)
   * Note: Returns the actual array - caller should not mutate
   * @returns {object[]}
   */
  getAllSegments() {
    return this._segments;
  }

  /**
   * Get segment count for display
   * @returns {number}
   */
  getSegmentCount() {
    return this._segments.length;
  }

  // ========================================
  // WRITE operations (controlled)
  // ========================================

  /**
   * Append a new segment
   * @param {object} segment - The segment to add
   * @throws {Error} If boundaries are not monotonic or validation fails
   */
  append(segment) {
    const validateSegment = getValidateSegment();

    // Validate monotonic boundaries
    if (this._segments.length > 0) {
      const last = this._segments[this._segments.length - 1];

      if (segment.startDist !== last.endDist) {
        throw new Error(
          `Segment boundaries not monotonic: new.startDist (${segment.startDist}) != last.endDist (${last.endDist})`
        );
      }

      if (segment.index !== last.index + 1) {
        throw new Error(
          `Segment index not sequential: new.index (${segment.index}) != last.index + 1 (${last.index + 1})`
        );
      }
    } else {
      // First segment must start at 0
      if (segment.index !== 0) {
        throw new Error(`First segment must have index=0, got ${segment.index}`);
      }
      if (segment.startDist !== 0) {
        throw new Error(`First segment must have startDist=0, got ${segment.startDist}`);
      }
    }

    // Validate segment structure
    const prevSegment = this._segments.length > 0 ? this._segments[this._segments.length - 1] : null;
    const errors = validateSegment(segment, prevSegment);
    if (errors.length > 0) {
      throw new Error(`Invalid segment:\n${errors.join('\n')}`);
    }

    this._segments.push(segment);
  }

  /**
   * Update the last segment (e.g., to set endDist when accepting)
   * @param {object} updates - Fields to update
   * @throws {Error} If no segments exist
   */
  updateLast(updates) {
    if (this._segments.length === 0) {
      throw new Error('No segments to update');
    }
    const last = this._segments[this._segments.length - 1];
    Object.assign(last, updates);
  }

  /**
   * Replace a segment at a specific index (use with caution)
   * @param {number} index
   * @param {object} segment
   * @throws {Error} If index is out of bounds
   */
  replaceSegment(index, segment) {
    if (index < 0 || index >= this._segments.length) {
      throw new Error(`Invalid segment index: ${index}`);
    }
    this._segments[index] = segment;
  }

  /**
   * Set the current segment index
   * @param {number} index
   * @throws {Error} If index is out of bounds
   */
  setCurrent(index) {
    if (index < 0 || index >= this._segments.length) {
      throw new Error(`Invalid segment index: ${index} (have ${this._segments.length} segments)`);
    }
    this._currentIndex = index;
  }

  /**
   * Move to the next segment
   * @returns {boolean} True if moved, false if already at last
   */
  moveToNext() {
    if (this._currentIndex < this._segments.length - 1) {
      this._currentIndex++;
      return true;
    }
    return false;
  }

  /**
   * Move to the previous segment
   * @returns {boolean} True if moved, false if already at first
   */
  moveToPrevious() {
    if (this._currentIndex > 0) {
      this._currentIndex--;
      return true;
    }
    return false;
  }

  // ========================================
  // RESET operations
  // ========================================

  /**
   * Reset to start (segment 0) - preserves mission history
   * Animation state is cleared separately by the reset() function
   */
  resetToStart() {
    this._currentIndex = 0;
  }

  /**
   * Clear all segments - use for new mission
   */
  clear() {
    this._segments = [];
    this._currentIndex = 0;
  }

  /**
   * Truncate segments after a given index
   * Used when cutting mid-mission to remove future segments
   * @param {number} afterIndex - Keep segments 0..afterIndex, remove rest
   */
  truncateAfter(afterIndex) {
    if (afterIndex < 0) {
      this.clear();
      return;
    }
    if (afterIndex < this._segments.length - 1) {
      this._segments = this._segments.slice(0, afterIndex + 1);
    }
    if (this._currentIndex > afterIndex) {
      this._currentIndex = afterIndex;
    }
  }

  // ========================================
  // QUERY operations
  // ========================================

  /**
   * Find segment containing a global distance
   * @param {number} globalDist
   * @returns {object|null} The segment or null
   */
  findSegmentAtDistance(globalDist) {
    for (const segment of this._segments) {
      if (segment.endDist === null) {
        // Last/open segment
        if (globalDist >= segment.startDist) {
          return segment;
        }
      } else {
        if (globalDist >= segment.startDist && globalDist < segment.endDist) {
          return segment;
        }
      }
    }
    return null;
  }

  /**
   * Get total mission distance (endDist of last segment)
   * @returns {number}
   */
  getTotalDistance() {
    const last = this.last();
    if (!last) return 0;
    return last.endDist ?? last.startDist;
  }

  /**
   * Get all visited target IDs (from frozen targets across segments)
   * @returns {string[]}
   */
  getAllVisitedTargetIds() {
    const visited = new Set();
    for (const segment of this._segments) {
      if (segment.targets?.frozen) {
        segment.targets.frozen.forEach(t => visited.add(t.id));
      }
    }
    return Array.from(visited);
  }

  /**
   * Get all drones that are disabled at or before a given segment
   * @param {number} segmentIndex
   * @returns {Set<string>}
   */
  getDisabledDronesAtSegment(segmentIndex) {
    const disabled = new Set();
    for (let i = 0; i <= segmentIndex && i < this._segments.length; i++) {
      const segment = this._segments[i];
      Object.entries(segment.drone_configs || {}).forEach(([droneId, config]) => {
        if (config.enabled === false) {
          disabled.add(droneId);
        }
      });
    }
    return disabled;
  }

  // ========================================
  // SERIALIZATION
  // ========================================

  /**
   * Export to JSON-serializable object
   * @returns {object}
   */
  toJSON() {
    return {
      version: '2.0',
      segments: this._segments,
      currentIndex: this._currentIndex
    };
  }

  /**
   * Import from JSON object
   * @param {object} data
   * @throws {Error} If data is invalid
   */
  fromJSON(data) {
    if (!data || !Array.isArray(data.segments)) {
      throw new Error('Invalid mission data: segments array required');
    }

    this.clear();

    // Add each segment with validation
    for (const segment of data.segments) {
      this.append(segment);
    }

    // Restore current index
    if (typeof data.currentIndex === 'number') {
      this._currentIndex = Math.min(data.currentIndex, this._segments.length - 1);
      this._currentIndex = Math.max(0, this._currentIndex);
    }
  }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { MissionReplay };
}

// Also export for ES6 modules / browser
if (typeof window !== 'undefined') {
  window.MissionReplayV2 = MissionReplay;
}
