/**
 * Animation Module - Drone animation and playback.
 *
 * This module handles:
 * - Animation playback (play, pause, stop)
 * - Drone position interpolation along trajectories
 * - Target visit detection
 * - Speed control
 *
 * Phase 4 Migration Notes:
 * - Animation code should be extracted from isr.js
 * - Key functions to migrate:
 *   - animateMission()
 *   - updateAnimationFrame()
 *   - interpolatePosition()
 *   - detectTargetVisits()
 */

import state from './state.js';
import renderer from './renderer.js';

// =============================================================================
// Animation State
// =============================================================================

let animationFrameId = null;
let lastTimestamp = 0;

// =============================================================================
// Interpolation
// =============================================================================

/**
 * Interpolate position along trajectory at given progress.
 * @param {Array} trajectory - Array of [x, y] points
 * @param {number} progress - Progress from 0 to 1
 * @returns {{x: number, y: number}} Interpolated position
 */
function interpolatePosition(trajectory, progress) {
    if (!trajectory || trajectory.length === 0) {
        return { x: 0, y: 0 };
    }

    if (trajectory.length === 1) {
        return { x: trajectory[0][0], y: trajectory[0][1] };
    }

    // Clamp progress
    progress = Math.max(0, Math.min(1, progress));

    // Calculate total trajectory length
    let totalLength = 0;
    const segmentLengths = [];

    for (let i = 1; i < trajectory.length; i++) {
        const dx = trajectory[i][0] - trajectory[i-1][0];
        const dy = trajectory[i][1] - trajectory[i-1][1];
        const segmentLength = Math.sqrt(dx * dx + dy * dy);
        segmentLengths.push(segmentLength);
        totalLength += segmentLength;
    }

    if (totalLength === 0) {
        return { x: trajectory[0][0], y: trajectory[0][1] };
    }

    // Find position at progress
    const targetDistance = progress * totalLength;
    let accumulatedDistance = 0;

    for (let i = 0; i < segmentLengths.length; i++) {
        if (accumulatedDistance + segmentLengths[i] >= targetDistance) {
            // Interpolate within this segment
            const segmentProgress = (targetDistance - accumulatedDistance) / segmentLengths[i];
            const x = trajectory[i][0] + (trajectory[i+1][0] - trajectory[i][0]) * segmentProgress;
            const y = trajectory[i][1] + (trajectory[i+1][1] - trajectory[i][1]) * segmentProgress;
            return { x, y };
        }
        accumulatedDistance += segmentLengths[i];
    }

    // Return end position
    const lastPoint = trajectory[trajectory.length - 1];
    return { x: lastPoint[0], y: lastPoint[1] };
}

/**
 * Calculate trajectory length.
 * @param {Array} trajectory - Array of [x, y] points
 * @returns {number} Total length
 */
function calculateTrajectoryLength(trajectory) {
    if (!trajectory || trajectory.length < 2) return 0;

    let length = 0;
    for (let i = 1; i < trajectory.length; i++) {
        const dx = trajectory[i][0] - trajectory[i-1][0];
        const dy = trajectory[i][1] - trajectory[i-1][1];
        length += Math.sqrt(dx * dx + dy * dy);
    }
    return length;
}

// =============================================================================
// Target Visit Detection
// =============================================================================

/**
 * Check if a target has been visited at current positions.
 * @param {Object} target - Target object with x, y
 * @param {Object} dronePositions - Drone positions
 * @param {number} threshold - Visit threshold distance
 * @returns {boolean} Whether target was visited
 */
function isTargetVisited(target, dronePositions, threshold = 10) {
    for (const pos of Object.values(dronePositions)) {
        const dx = pos.x - target.x;
        const dy = pos.y - target.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        if (distance < threshold) {
            return true;
        }
    }
    return false;
}

// =============================================================================
// Animation Control
// =============================================================================

/**
 * Start animation playback.
 */
export function play() {
    if (state.getState('animation.isPlaying')) return;

    state.setState('animation.isPlaying', true);
    lastTimestamp = performance.now();
    animationFrameId = requestAnimationFrame(animationLoop);
}

/**
 * Pause animation playback.
 */
export function pause() {
    state.setState('animation.isPlaying', false);
    if (animationFrameId) {
        cancelAnimationFrame(animationFrameId);
        animationFrameId = null;
    }
}

/**
 * Stop animation and reset to beginning.
 */
export function stop() {
    pause();
    state.setState('animation.currentTime', 0);
    updateDronePositions(0);
    renderer.requestRender();
}

/**
 * Set animation speed.
 * @param {number} speed - Speed multiplier (1.0 = normal)
 */
export function setSpeed(speed) {
    state.setState('animation.speed', speed);
}

/**
 * Seek to specific time.
 * @param {number} time - Time in seconds
 */
export function seek(time) {
    state.setState('animation.currentTime', time);
    updateDronePositions(time);
    renderer.requestRender();
}

// =============================================================================
// Animation Loop
// =============================================================================

/**
 * Main animation loop.
 * @param {number} timestamp - Current timestamp
 */
function animationLoop(timestamp) {
    if (!state.getState('animation.isPlaying')) return;

    const deltaTime = (timestamp - lastTimestamp) / 1000;  // Convert to seconds
    lastTimestamp = timestamp;

    const speed = state.getState('animation.speed') || 1.0;
    const currentTime = state.getState('animation.currentTime') + deltaTime * speed;

    // Get max duration from trajectories
    const maxDuration = getMaxDuration();

    if (currentTime >= maxDuration) {
        // Animation complete
        state.setState('animation.currentTime', maxDuration);
        state.setState('animation.isPlaying', false);
        updateDronePositions(maxDuration);
        renderer.requestRender();
        return;
    }

    state.setState('animation.currentTime', currentTime);
    updateDronePositions(currentTime);
    renderer.requestRender();

    animationFrameId = requestAnimationFrame(animationLoop);
}

/**
 * Update drone positions based on current time.
 * @param {number} time - Current animation time
 */
function updateDronePositions(time) {
    const solution = state.getSolution();
    const routes = solution.routes || {};
    const dronePositions = {};

    const maxDuration = getMaxDuration();
    const progress = maxDuration > 0 ? time / maxDuration : 0;

    for (const [droneId, routeData] of Object.entries(routes)) {
        const trajectory = routeData.trajectory || [];
        if (trajectory.length > 0) {
            dronePositions[droneId] = interpolatePosition(trajectory, progress);
        }
    }

    state.setState('animation.dronePositions', dronePositions);
}

/**
 * Get maximum animation duration based on trajectories.
 * @returns {number} Duration in seconds
 */
function getMaxDuration() {
    const solution = state.getSolution();
    const routes = solution.routes || {};

    let maxLength = 0;
    for (const routeData of Object.values(routes)) {
        const trajectory = routeData.trajectory || [];
        const length = calculateTrajectoryLength(trajectory);
        maxLength = Math.max(maxLength, length);
    }

    // Assume speed of 100 units per second
    const speed = 100;
    return maxLength / speed;
}

/**
 * Get current animation progress (0-1).
 * @returns {number} Progress
 */
export function getProgress() {
    const currentTime = state.getState('animation.currentTime') || 0;
    const maxDuration = getMaxDuration();
    return maxDuration > 0 ? currentTime / maxDuration : 0;
}

/**
 * Get current animation time.
 * @returns {number} Time in seconds
 */
export function getCurrentTime() {
    return state.getState('animation.currentTime') || 0;
}

/**
 * Get total animation duration.
 * @returns {number} Duration in seconds
 */
export function getDuration() {
    return getMaxDuration();
}

export default {
    play,
    pause,
    stop,
    setSpeed,
    seek,
    getProgress,
    getCurrentTime,
    getDuration
};
