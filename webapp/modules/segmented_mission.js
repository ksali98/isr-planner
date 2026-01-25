/**
 * Segmented Mission Module - Clean implementation for multi-segment missions.
 *
 * DESIGN PRINCIPLES (from SYSTEM_ARCHITECTURE.md):
 * 1. Single source of truth - SegmentedMission is the ONLY place segment data lives
 * 2. Simple data flow - No splicing/extraction gymnastics
 * 3. Predictable behavior - Same code path for fresh creation and import replay
 * 4. Clean separation - Generation logic separate from playback logic
 *
 * KEY CONCEPTS:
 * - Segment: A solved portion of the mission (from start/cut to next cut/end)
 * - Cut: A point where the mission was paused and replanned
 * - Frozen trajectory: Trajectory portion that is immutable after a cut
 * - Combined trajectory: Full trajectory built by concatenating segment trajectories
 */

import state from './state.js';

// =============================================================================
// Segment Class - Immutable data structure for one solved segment
// =============================================================================

class Segment {
    constructor(data) {
        this.index = data.index ?? 0;

        // What the solver returned (raw, unmodified)
        this.solution = data.solution ?? { routes: {}, sequences: {} };

        // Environment state for THIS segment
        this.environment = data.environment ?? {
            targets: [],
            airports: [],
            sams: [],
            drone_configs: {}
        };

        // Cut info (null for final segment)
        this.cut = data.cut ?? null;

        // Drones lost AT THE END of this segment (disabled at the cut)
        this.lostDrones = data.lostDrones ?? [];

        // Drones added AT THE START of this segment
        this.addedDrones = data.addedDrones ?? [];

        // Targets visited in this segment
        this.visitedTargets = data.visitedTargets ?? [];

        // Timestamp when segment was created
        this.timestamp = data.timestamp ?? Date.now();

        // Freeze to prevent accidental mutation
        Object.freeze(this);
    }

    /**
     * Get trajectory for a specific drone in this segment.
     * @param {string} droneId - Drone ID
     * @returns {Array} Trajectory points [[x,y], ...]
     */
    getTrajectory(droneId) {
        return this.solution.routes?.[droneId]?.trajectory ?? [];
    }

    /**
     * Get route (waypoint sequence) for a specific drone.
     * @param {string} droneId - Drone ID
     * @returns {Array} Route waypoints ['A1', 'T1', 'T2', 'A1']
     */
    getRoute(droneId) {
        return this.solution.routes?.[droneId]?.route ?? [];
    }

    /**
     * Check if drone was active in this segment.
     * @param {string} droneId - Drone ID
     * @returns {boolean}
     */
    hasDrone(droneId) {
        return !!this.solution.routes?.[droneId];
    }

    /**
     * Get cut position for a drone.
     * @param {string} droneId - Drone ID
     * @returns {Array|null} [x, y] or null
     */
    getCutPosition(droneId) {
        return this.cut?.positions?.[droneId] ?? null;
    }
}

// =============================================================================
// SegmentedMission Class - Main container for multi-segment missions
// =============================================================================

class SegmentedMission {
    constructor() {
        this.clear();
    }

    /**
     * Clear all mission data.
     */
    clear() {
        // Base environment (targets, airports, SAMs visible at start)
        this._baseEnvironment = {
            targets: [],
            airports: [],
            sams: [],
            drone_configs: {}
        };

        // Array of segments
        this._segments = [];

        // Current segment index (for fresh creation workflow)
        this._currentIndex = 0;

        // Whether mission is active
        this._isActive = false;

        // Computed data (built on demand, not stored in JSON)
        this._computed = {
            combinedTrajectories: {},
            cutMarkers: [],
            totalDistance: 0,
            isDirty: true
        };
    }

    // =========================================================================
    // State Accessors
    // =========================================================================

    isActive() {
        return this._isActive;
    }

    getCurrentIndex() {
        return this._currentIndex;
    }

    getSegmentCount() {
        return this._segments.length;
    }

    getSegment(index) {
        return this._segments[index] ?? null;
    }

    getCurrentSegment() {
        return this.getSegment(this._currentIndex);
    }

    getBaseEnvironment() {
        return JSON.parse(JSON.stringify(this._baseEnvironment));
    }

    getAllTargets() {
        return JSON.parse(JSON.stringify(this._baseEnvironment.targets));
    }

    // =========================================================================
    // Computed Data (On-demand concatenation)
    // =========================================================================

    /**
     * Build combined trajectory for a drone across all segments.
     * This is the key simplification - no splicing at solve time,
     * just concatenate at animation time.
     * @param {string} droneId - Drone ID
     * @returns {Array} Combined trajectory [[x,y], ...]
     */
    buildCombinedTrajectory(droneId) {
        let combined = [];

        for (const segment of this._segments) {
            const traj = segment.getTrajectory(droneId);
            if (traj.length === 0) continue;

            if (combined.length === 0) {
                combined = [...traj];
            } else {
                // Check if trajectories join (last point == first point)
                const lastPoint = combined[combined.length - 1];
                const firstPoint = traj[0];
                const joins = this._pointsMatch(lastPoint, firstPoint);

                if (joins) {
                    // Skip duplicate join point
                    combined = combined.concat(traj.slice(1));
                } else {
                    combined = combined.concat(traj);
                }
            }
        }

        return combined;
    }

    /**
     * Get all combined trajectories (cached).
     * @returns {Object} { droneId: [[x,y], ...] }
     */
    getCombinedTrajectories() {
        if (this._computed.isDirty) {
            this._recompute();
        }
        return JSON.parse(JSON.stringify(this._computed.combinedTrajectories));
    }

    /**
     * Get cut markers for display.
     * @returns {Array} [{ label: 'C1', position: [x,y], distance: 45.5, lostDrones: [] }, ...]
     */
    getCutMarkers() {
        if (this._computed.isDirty) {
            this._recompute();
        }
        return JSON.parse(JSON.stringify(this._computed.cutMarkers));
    }

    /**
     * Get total mission distance.
     * @returns {number}
     */
    getTotalDistance() {
        if (this._computed.isDirty) {
            this._recompute();
        }
        return this._computed.totalDistance;
    }

    /**
     * Recompute cached data.
     * @private
     */
    _recompute() {
        // Build combined trajectories for all drones
        const droneIds = new Set();
        for (const segment of this._segments) {
            Object.keys(segment.solution.routes || {}).forEach(id => droneIds.add(id));
        }

        this._computed.combinedTrajectories = {};
        for (const droneId of droneIds) {
            this._computed.combinedTrajectories[droneId] = this.buildCombinedTrajectory(droneId);
        }

        // Build cut markers
        this._computed.cutMarkers = [];
        let cutNumber = 1;
        for (const segment of this._segments) {
            if (segment.cut) {
                // Use first available drone's cut position for marker
                const positions = segment.cut.positions || {};
                const firstDroneId = Object.keys(positions)[0];
                const position = positions[firstDroneId];

                this._computed.cutMarkers.push({
                    label: `C${cutNumber}`,
                    position: position,
                    distance: segment.cut.distance,
                    lostDrones: [...segment.lostDrones],
                    segmentIndex: segment.index
                });
                cutNumber++;
            }
        }

        // Calculate total distance (max across all drones)
        this._computed.totalDistance = 0;
        for (const traj of Object.values(this._computed.combinedTrajectories)) {
            const dist = this._calculateTrajectoryLength(traj);
            this._computed.totalDistance = Math.max(this._computed.totalDistance, dist);
        }

        this._computed.isDirty = false;
    }

    /**
     * Mark computed data as needing refresh.
     */
    _markDirty() {
        this._computed.isDirty = true;
    }

    // =========================================================================
    // Fresh Creation (Generation) Flow
    // =========================================================================

    /**
     * Initialize for fresh mission creation.
     * @param {Object} environment - Initial environment
     */
    initFresh(environment) {
        this.clear();
        this._baseEnvironment = JSON.parse(JSON.stringify(environment));
        this._isActive = true;
        this._currentIndex = 0;
    }

    /**
     * Add a new segment with solver results.
     * @param {Object} data - Segment data
     * @param {Object} data.solution - Solver output { routes, sequences }
     * @param {Object} data.environment - Environment state for this segment
     * @param {Object} [data.cut] - Cut info { positions: {droneId: [x,y]}, distance: number }
     * @param {Array} [data.lostDrones] - Drones disabled at this cut
     * @param {Array} [data.addedDrones] - Drones added at this segment start
     * @param {Array} [data.visitedTargets] - Targets visited in this segment
     * @returns {Segment} Created segment
     */
    addSegment(data) {
        const segment = new Segment({
            index: this._segments.length,
            solution: data.solution,
            environment: data.environment,
            cut: data.cut || null,
            lostDrones: data.lostDrones || [],
            addedDrones: data.addedDrones || [],
            visitedTargets: data.visitedTargets || [],
            timestamp: Date.now()
        });

        this._segments.push(segment);
        this._currentIndex = segment.index;
        this._markDirty();

        return segment;
    }

    /**
     * Record a cut at current animation position.
     * @param {Object} cutData - Cut information
     * @param {Object} cutData.positions - Per-drone positions { droneId: [x, y] }
     * @param {number} cutData.distance - Distance traveled at cut point
     * @param {Array} [cutData.lostDrones] - Drones being disabled
     * @returns {boolean} Success
     */
    recordCut(cutData) {
        const currentSegment = this._segments[this._currentIndex];
        if (!currentSegment) return false;

        // Update current segment with cut info
        // Note: Since Segment is frozen, we need to replace it
        const updatedData = {
            index: currentSegment.index,
            solution: currentSegment.solution,
            environment: currentSegment.environment,
            cut: {
                positions: cutData.positions,
                distance: cutData.distance
            },
            lostDrones: cutData.lostDrones || [],
            addedDrones: currentSegment.addedDrones,
            visitedTargets: currentSegment.visitedTargets,
            timestamp: currentSegment.timestamp
        };

        this._segments[this._currentIndex] = new Segment(updatedData);
        this._markDirty();

        return true;
    }

    /**
     * Perform a cut at current animation state.
     * This is the main entry point for the cut workflow.
     * @param {Object} animationState - Current animation state { drones: { droneId: { distanceTraveled, totalDistance, ... } } }
     * @param {Object} routes - Current routes { droneId: { trajectory, route } }
     * @param {Object} environment - Current environment with targets
     * @returns {Object} Cut result { positions, distance, visitedTargets, truncatedTrajectories }
     */
    performCut(animationState, routes, environment) {
        const drones = animationState.drones || {};
        const positions = {};
        const truncatedTrajectories = {};
        const visitedTargets = [];
        let cutDistance = 0;

        // Get cut distance from first animating drone (all fly at same speed)
        const firstDroneState = Object.values(drones)[0];
        if (firstDroneState) {
            cutDistance = firstDroneState.distanceTraveled || 0;
        }

        // Process each drone
        Object.entries(drones).forEach(([droneId, droneState]) => {
            const trajectory = routes[droneId]?.trajectory || [];
            if (trajectory.length < 2) return;

            const currentDist = droneState.distanceTraveled || 0;
            const totalDist = droneState.totalDistance || 0;
            const finished = totalDist > 0 && currentDist >= (totalDist - 0.0001);

            // Split trajectory at current distance
            const { prefixPoints, suffixPoints, splitPoint } = this._splitTrajectoryAtDistance(trajectory, currentDist);

            // Store cut position for drones still in-flight
            if (!finished && splitPoint) {
                positions[droneId] = [splitPoint[0], splitPoint[1]];
            }

            // Store truncated trajectory
            truncatedTrajectories[droneId] = {
                prefix: prefixPoints,
                suffix: suffixPoints,
                splitPoint,
                finished
            };

            // Calculate visited targets for this drone
            const route = routes[droneId]?.route || [];
            const droneVisited = this._calculateVisitedTargets(
                route, trajectory, currentDist, environment.targets || []
            );
            droneVisited.forEach(tid => {
                if (!visitedTargets.includes(tid)) {
                    visitedTargets.push(tid);
                }
            });
        });

        // Record the cut
        this.recordCut({
            positions,
            distance: cutDistance,
            lostDrones: []
        });

        // Update visited targets in current segment
        const currentSegment = this._segments[this._currentIndex];
        if (currentSegment) {
            this._segments[this._currentIndex] = new Segment({
                ...currentSegment,
                visitedTargets
            });
        }

        return {
            positions,
            distance: cutDistance,
            visitedTargets,
            truncatedTrajectories
        };
    }

    /**
     * Split a trajectory at a given distance.
     * @param {Array} trajectory - Points [[x,y], ...]
     * @param {number} distance - Distance to split at
     * @returns {Object} { prefixPoints, suffixPoints, splitPoint }
     */
    _splitTrajectoryAtDistance(trajectory, distance) {
        if (!trajectory || trajectory.length < 2) {
            return { prefixPoints: [], suffixPoints: [], splitPoint: null };
        }

        // Build cumulative distances
        const cumDist = [0];
        for (let i = 1; i < trajectory.length; i++) {
            const dx = trajectory[i][0] - trajectory[i - 1][0];
            const dy = trajectory[i][1] - trajectory[i - 1][1];
            cumDist.push(cumDist[i - 1] + Math.sqrt(dx * dx + dy * dy));
        }

        const totalLength = cumDist[cumDist.length - 1];
        if (distance >= totalLength) {
            return {
                prefixPoints: [...trajectory],
                suffixPoints: [],
                splitPoint: trajectory[trajectory.length - 1]
            };
        }

        // Find segment where split occurs
        let splitIdx = 0;
        for (let i = 1; i < cumDist.length; i++) {
            if (cumDist[i] >= distance) {
                splitIdx = i;
                break;
            }
        }

        // Interpolate split point
        const segmentStartDist = cumDist[splitIdx - 1];
        const segmentLength = cumDist[splitIdx] - segmentStartDist;
        const t = segmentLength > 0 ? (distance - segmentStartDist) / segmentLength : 0;

        const p1 = trajectory[splitIdx - 1];
        const p2 = trajectory[splitIdx];
        const splitPoint = [
            p1[0] + t * (p2[0] - p1[0]),
            p1[1] + t * (p2[1] - p1[1])
        ];

        // Build prefix and suffix
        const prefixPoints = trajectory.slice(0, splitIdx);
        prefixPoints.push(splitPoint);

        const suffixPoints = [splitPoint, ...trajectory.slice(splitIdx)];

        return { prefixPoints, suffixPoints, splitPoint };
    }

    /**
     * Calculate which targets have been visited given route, trajectory, and distance.
     * @param {Array} route - Route waypoints ['A1', 'T1', 'T2', 'A1']
     * @param {Array} trajectory - Trajectory points [[x,y], ...]
     * @param {number} currentDistance - Distance traveled
     * @param {Array} targets - Target objects [{ id, x, y }, ...]
     * @returns {Array} Visited target IDs
     */
    _calculateVisitedTargets(route, trajectory, currentDistance, targets) {
        const visited = [];
        const PROXIMITY_THRESHOLD = 20.0;

        if (!trajectory || trajectory.length < 2) return visited;

        // Build cumulative distances
        const cumDist = [0];
        for (let i = 1; i < trajectory.length; i++) {
            const dx = trajectory[i][0] - trajectory[i - 1][0];
            const dy = trajectory[i][1] - trajectory[i - 1][1];
            cumDist.push(cumDist[i - 1] + Math.sqrt(dx * dx + dy * dy));
        }

        route.forEach(waypoint => {
            if (!String(waypoint).startsWith('T')) return;

            const target = targets.find(t => t.id === waypoint);
            if (!target) return;

            // Find closest trajectory point to this target
            let minDist = Infinity;
            let closestIdx = -1;
            trajectory.forEach((pt, idx) => {
                const dist = Math.sqrt(
                    Math.pow(pt[0] - target.x, 2) +
                    Math.pow(pt[1] - target.y, 2)
                );
                if (dist < minDist) {
                    minDist = dist;
                    closestIdx = idx;
                }
            });

            // Target is visited if:
            // 1. Closest point is within threshold
            // 2. We've traveled past that point
            const targetDist = closestIdx >= 0 ? cumDist[closestIdx] : Infinity;
            if (closestIdx >= 0 && minDist < PROXIMITY_THRESHOLD && currentDistance >= targetDist) {
                visited.push(waypoint);
            }
        });

        return visited;
    }

    /**
     * Advance to next segment after a cut.
     * @returns {number} New segment index
     */
    advanceSegment() {
        this._currentIndex = this._segments.length;
        return this._currentIndex;
    }

    // =========================================================================
    // Import/Export (Usage Flow)
    // =========================================================================

    /**
     * Import from JSON data.
     * Supports both isr_segmented_v2 and isr_env_v2 schemas.
     * @param {Object} data - Parsed JSON
     * @returns {Object|null} Initial drone configs for UI, or null on failure
     */
    importFromJson(data) {
        this.clear();

        // Handle different schema versions
        const segments = data.segments || [];
        if (segments.length === 0) {
            console.warn('[SegmentedMission] No segments in JSON');
            return null;
        }

        const schema = data.schema || 'unknown';
        console.log(`[SegmentedMission] Importing ${segments.length} segments (schema: ${schema})`);

        // Extract base environment from top-level or first segment
        const seg0Env = segments[0]?.env || segments[0]?.environment || {};
        this._baseEnvironment = {
            airports: JSON.parse(JSON.stringify(data.airports || seg0Env.airports || [])),
            sams: JSON.parse(JSON.stringify(data.sams || seg0Env.sams || [])),
            targets: [],
            drone_configs: JSON.parse(JSON.stringify(data.drone_configs || seg0Env.drone_configs || {}))
        };

        // Collect all unique targets from all segments
        const targetMap = new Map();
        segments.forEach(seg => {
            const targets = seg.environment?.targets || seg.env?.targets || [];
            targets.forEach(t => {
                if (!targetMap.has(t.id)) {
                    targetMap.set(t.id, JSON.parse(JSON.stringify(t)));
                }
            });
        });
        this._baseEnvironment.targets = Array.from(targetMap.values());

        // Import each segment
        segments.forEach((segData, idx) => {
            // Get drone configs - check multiple possible keys
            const droneConfigs = segData.droneConfigs ||
                                segData.drone_configs ||
                                segData.env?.drone_configs ||
                                segData.environment?.drone_configs ||
                                {};

            // Normalize drone configs (ensure all 5 drones have entries)
            const normalizedConfigs = {};
            for (let d = 1; d <= 5; d++) {
                const did = String(d);
                const cfg = droneConfigs[did] || {};
                normalizedConfigs[did] = {
                    enabled: cfg.enabled ?? false,
                    fuel_budget: cfg.fuel_budget ?? 150,
                    start_airport: cfg.start_airport || `A${d}`,
                    end_airport: cfg.end_airport || `A${d}`,
                    target_access: cfg.target_access || { a: true, b: true, c: true, d: true, e: true }
                };
            }

            // Get targets for this segment
            const segTargets = segData.environment?.targets || segData.env?.targets || [];

            // Derive lostDrones if not explicit (by comparing with previous segment)
            let lostDrones = segData.lostDrones ? [...segData.lostDrones] : [];
            if (idx > 0 && lostDrones.length === 0) {
                const prevSeg = this._segments[idx - 1];
                if (prevSeg) {
                    const prevConfigs = prevSeg.environment.drone_configs;
                    Object.entries(prevConfigs).forEach(([did, cfg]) => {
                        if (cfg.enabled && !normalizedConfigs[did]?.enabled) {
                            lostDrones.push(did);
                        }
                    });
                }
            }

            const segment = new Segment({
                index: idx,
                solution: segData.solution || { routes: {}, sequences: {} },
                environment: {
                    targets: segTargets,
                    airports: segData.environment?.airports || segData.env?.airports || this._baseEnvironment.airports,
                    sams: segData.environment?.sams || segData.env?.sams || this._baseEnvironment.sams,
                    drone_configs: normalizedConfigs
                },
                cut: segData.cutPositions ? {
                    positions: segData.cutPositions,
                    distance: segData.cutDistance || 0
                } : null,
                lostDrones,
                addedDrones: segData.addedDrones || [],
                visitedTargets: segData.visitedTargets || segData.frozenTargets || [],
                timestamp: segData.timestamp || Date.now()
            });

            this._segments.push(segment);

            const enabled = Object.entries(normalizedConfigs)
                .filter(([, c]) => c.enabled)
                .map(([d]) => `D${d}`)
                .join(',');
            console.log(`[SegmentedMission] Segment ${idx}: [${enabled || 'none'}] enabled, lostDrones=[${lostDrones.join(',')}]`);
        });

        this._currentIndex = 0;
        this._isActive = true;
        this._markDirty();

        console.log(`[SegmentedMission] Import complete: ${this._segments.length} segments, ${this._baseEnvironment.targets.length} total targets`);

        // Return segment 0's drone configs for UI initialization
        return this.getDroneConfigsForSegment(0);
    }

    /**
     * Export to JSON format.
     * @returns {Object} JSON-serializable object
     */
    exportToJson() {
        const result = {
            schema: 'isr_segmented_v2',
            is_segmented: true,
            segment_count: this._segments.length,
            airports: JSON.parse(JSON.stringify(this._baseEnvironment.airports)),
            sams: JSON.parse(JSON.stringify(this._baseEnvironment.sams)),
            drone_configs: this._segments[0]?.environment.drone_configs
                ? JSON.parse(JSON.stringify(this._segments[0].environment.drone_configs))
                : {},
            segments: []
        };

        // Track cumulative frozen state
        let cumulativeFrozenTargets = [];

        this._segments.forEach((segment, idx) => {
            const segExport = {
                index: idx,
                solution: JSON.parse(JSON.stringify(segment.solution)),
                environment: JSON.parse(JSON.stringify(segment.environment)),
                droneConfigs: JSON.parse(JSON.stringify(segment.environment.drone_configs)),
                frozenTargets: [...cumulativeFrozenTargets],
                activeTargets: segment.environment.targets.map(t => t.id),
                cutPositions: segment.cut?.positions || null,
                cutDistance: segment.cut?.distance || null,
                lostDrones: [...segment.lostDrones],
                addedDrones: [...segment.addedDrones],
                visitedTargets: [...segment.visitedTargets],
                timestamp: segment.timestamp
            };

            result.segments.push(segExport);

            // Update cumulative frozen targets for next segment
            segment.visitedTargets.forEach(tid => {
                if (!cumulativeFrozenTargets.includes(tid)) {
                    cumulativeFrozenTargets.push(tid);
                }
            });
        });

        return result;
    }

    // =========================================================================
    // Environment Builders (for solver and display)
    // =========================================================================

    /**
     * Get environment for DISPLAY (drawing the canvas).
     * Includes all targets visible at current segment.
     * @returns {Object} Environment for rendering
     */
    getDisplayEnvironment() {
        const segment = this.getCurrentSegment();
        if (!segment) {
            return JSON.parse(JSON.stringify(this._baseEnvironment));
        }

        return {
            airports: JSON.parse(JSON.stringify(this._baseEnvironment.airports)),
            sams: JSON.parse(JSON.stringify(this._baseEnvironment.sams)),
            targets: JSON.parse(JSON.stringify(segment.environment.targets)),
            drone_configs: JSON.parse(JSON.stringify(segment.environment.drone_configs))
        };
    }

    /**
     * Get environment for SOLVER.
     * Only active (unfrozen) targets, with synthetic starts for segment > 0.
     * @param {Object} uiDroneConfigs - Current UI drone configs (source of truth)
     * @param {Array} visitedTargets - Targets already visited
     * @returns {Object} Environment for solver
     */
    getSolverEnvironment(uiDroneConfigs, visitedTargets = []) {
        const visitedSet = new Set(visitedTargets);
        const activeTargets = this._baseEnvironment.targets.filter(t => !visitedSet.has(t.id));

        const env = {
            airports: JSON.parse(JSON.stringify(this._baseEnvironment.airports)),
            sams: JSON.parse(JSON.stringify(this._baseEnvironment.sams)),
            targets: JSON.parse(JSON.stringify(activeTargets)),
            drone_configs: JSON.parse(JSON.stringify(uiDroneConfigs)),
            synthetic_starts: {}
        };

        // For segment > 0, add synthetic start positions
        if (this._currentIndex > 0) {
            const prevSegment = this._segments[this._currentIndex - 1];
            if (prevSegment?.cut?.positions) {
                Object.entries(prevSegment.cut.positions).forEach(([droneId, pos]) => {
                    if (uiDroneConfigs[droneId]?.enabled && Array.isArray(pos)) {
                        const nodeId = `D${droneId}_START`;
                        env.synthetic_starts[nodeId] = { id: nodeId, x: pos[0], y: pos[1] };

                        // Update drone's start_airport to synthetic start
                        if (env.drone_configs[droneId]) {
                            env.drone_configs[droneId].start_airport = nodeId;
                        }
                    }
                });
            }
        }

        return env;
    }

    /**
     * Get drone configs for a specific segment.
     * @param {number} index - Segment index
     * @returns {Object|null} Drone configs
     */
    getDroneConfigsForSegment(index) {
        const segment = this._segments[index];
        if (!segment) return null;
        return JSON.parse(JSON.stringify(segment.environment.drone_configs));
    }

    /**
     * Get visited targets up to a segment index.
     * @param {number} upToIndex - Segment index (exclusive)
     * @returns {Array} Target IDs
     */
    getVisitedTargets(upToIndex) {
        const visited = [];
        for (let i = 0; i < upToIndex && i < this._segments.length; i++) {
            this._segments[i].visitedTargets.forEach(tid => {
                if (!visited.includes(tid)) {
                    visited.push(tid);
                }
            });
        }
        return visited;
    }

    // =========================================================================
    // Animation Support
    // =========================================================================

    /**
     * Get starting distance for animation (based on current segment).
     * For segment 0, this is 0. For later segments, it's the cumulative cut distance.
     * @returns {number}
     */
    getStartingDistance() {
        let totalDistance = 0;
        for (let i = 0; i < this._currentIndex && i < this._segments.length; i++) {
            totalDistance += this._segments[i].cut?.distance || 0;
        }
        return totalDistance;
    }

    /**
     * Get animation configuration for current segment.
     * @param {Object} routes - Routes object { droneId: { trajectory, route } }
     * @returns {Object} { droneConfigs: {...}, startDistance, trajectories }
     */
    getAnimationConfig(routes) {
        const segment = this.getCurrentSegment();
        if (!segment) {
            return {
                droneConfigs: {},
                startDistance: 0,
                trajectories: routes
            };
        }

        return {
            droneConfigs: JSON.parse(JSON.stringify(segment.environment.drone_configs)),
            startDistance: this.getStartingDistance(),
            trajectories: routes
        };
    }

    /**
     * Get frozen trajectories up to current segment.
     * Used for rendering the "completed" portion of the mission.
     * @returns {Object} { droneId: [[x,y], ...] }
     */
    getFrozenTrajectories() {
        const frozen = {};

        for (let i = 0; i < this._currentIndex && i < this._segments.length; i++) {
            const segment = this._segments[i];
            Object.entries(segment.solution.routes || {}).forEach(([droneId, routeData]) => {
                const traj = routeData.trajectory || [];
                if (traj.length === 0) return;

                if (!frozen[droneId]) {
                    frozen[droneId] = [...traj];
                } else {
                    // Concatenate, avoiding duplicate junction points
                    const existing = frozen[droneId];
                    const lastPoint = existing[existing.length - 1];
                    const firstPoint = traj[0];
                    const isDupe = this._pointsMatch(lastPoint, firstPoint);
                    frozen[droneId] = isDupe
                        ? existing.concat(traj.slice(1))
                        : existing.concat(traj);
                }
            });
        }

        return frozen;
    }

    /**
     * Build environment for animation rendering.
     * Includes frozen trajectory markers and visited target markers.
     * @returns {Object}
     */
    getAnimationEnvironment() {
        const displayEnv = this.getDisplayEnvironment();
        const frozenTraj = this.getFrozenTrajectories();
        const cutMarkers = this.getCutMarkers();
        const visitedTargets = this.getVisitedTargets(this._currentIndex);

        return {
            ...displayEnv,
            frozenTrajectories: frozenTraj,
            cutMarkers,
            visitedTargets
        };
    }

    // =========================================================================
    // Checkpoint Solving Support
    // =========================================================================

    /**
     * Get synthetic start positions for checkpoint solving.
     * Uses cut positions from previous segment.
     * @returns {Object|null} { droneId: { id, x, y } } or null
     */
    getSyntheticStarts() {
        if (this._currentIndex === 0) return null;

        const prevSegment = this._segments[this._currentIndex - 1];
        if (!prevSegment?.cut?.positions) return null;

        const starts = {};
        Object.entries(prevSegment.cut.positions).forEach(([droneId, pos]) => {
            const nodeId = `D${droneId}_START`;
            starts[nodeId] = {
                id: nodeId,
                x: pos[0],
                y: pos[1],
                droneId
            };
        });

        return Object.keys(starts).length > 0 ? starts : null;
    }

    /**
     * Check if we're in checkpoint mode (segment > 0).
     * @returns {boolean}
     */
    isCheckpointMode() {
        return this._currentIndex > 0;
    }

    /**
     * Get drones that were lost (disabled) at the last cut.
     * @returns {Array} Drone IDs
     */
    getLostDrones() {
        if (this._currentIndex === 0) return [];

        const prevSegment = this._segments[this._currentIndex - 1];
        return prevSegment?.lostDrones || [];
    }

    /**
     * Get drones that should be active for current segment.
     * Excludes drones lost in previous segments.
     * @param {Object} baseConfigs - Base drone configs from UI
     * @returns {Array} Active drone IDs
     */
    getActiveDroneIds(baseConfigs) {
        const enabled = Object.entries(baseConfigs)
            .filter(([, cfg]) => cfg.enabled)
            .map(([id]) => id);

        // Remove drones lost in previous segments
        const allLost = [];
        for (let i = 0; i < this._currentIndex && i < this._segments.length; i++) {
            (this._segments[i].lostDrones || []).forEach(did => {
                if (!allLost.includes(did)) allLost.push(did);
            });
        }

        return enabled.filter(id => !allLost.includes(id));
    }

    // =========================================================================
    // Navigation
    // =========================================================================

    /**
     * Reset to segment 0.
     */
    resetToStart() {
        this._currentIndex = 0;
    }

    /**
     * Set current segment index.
     * @param {number} index
     */
    setCurrentIndex(index) {
        this._currentIndex = Math.max(0, Math.min(index, this._segments.length - 1));
    }

    // =========================================================================
    // Utility Functions
    // =========================================================================

    /**
     * Check if two points match (within tolerance).
     * @private
     */
    _pointsMatch(p1, p2, tolerance = 0.001) {
        if (!p1 || !p2) return false;
        return Math.abs(p1[0] - p2[0]) < tolerance && Math.abs(p1[1] - p2[1]) < tolerance;
    }

    /**
     * Calculate trajectory length.
     * @private
     */
    _calculateTrajectoryLength(trajectory) {
        if (!trajectory || trajectory.length < 2) return 0;

        let length = 0;
        for (let i = 1; i < trajectory.length; i++) {
            const dx = trajectory[i][0] - trajectory[i - 1][0];
            const dy = trajectory[i][1] - trajectory[i - 1][1];
            length += Math.sqrt(dx * dx + dy * dy);
        }
        return length;
    }

    /**
     * Get debug information.
     * @returns {Object}
     */
    getDebugInfo() {
        return {
            isActive: this._isActive,
            currentIndex: this._currentIndex,
            segmentCount: this._segments.length,
            baseTargetCount: this._baseEnvironment.targets.length,
            segments: this._segments.map(seg => ({
                index: seg.index,
                hasCut: !!seg.cut,
                lostDrones: seg.lostDrones,
                addedDrones: seg.addedDrones,
                visitedTargets: seg.visitedTargets,
                droneCount: Object.keys(seg.solution.routes || {}).length
            }))
        };
    }
}

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * Check if JSON data is a segmented mission.
 * @param {Object} data - Parsed JSON
 * @returns {boolean}
 */
export function isSegmentedMission(data) {
    return (
        (data.schema === 'isr_segmented_v2' || data.schema === 'isr_env_v2') &&
        data.is_segmented === true
    ) || (
        Array.isArray(data.segments) && data.segments.length > 0
    );
}

/**
 * Create a new SegmentedMission instance.
 * @returns {SegmentedMission}
 */
export function createSegmentedMission() {
    return new SegmentedMission();
}

// =============================================================================
// Module Exports
// =============================================================================

export { Segment, SegmentedMission };

export default {
    Segment,
    SegmentedMission,
    isSegmentedMission,
    createSegmentedMission
};
