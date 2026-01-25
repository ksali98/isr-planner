/**
 * Segment Manager UI Module - UI controls for segmented missions.
 *
 * This module handles:
 * - Segment navigation panel (previous/next, segment indicator)
 * - Segment status display (active targets, frozen targets, lost drones)
 * - Cut workflow UI (cut button, cut confirmation)
 * - Segment export/import UI integration
 *
 * Works in conjunction with segmented_mission.js for data management.
 */

import state from './state.js';
import renderer from './renderer.js';
import { SegmentedMission, isSegmentedMission } from './segmented_mission.js';

// =============================================================================
// Constants
// =============================================================================

const SEGMENT_COLORS = [
    '#3498db', // Blue
    '#2ecc71', // Green
    '#f39c12', // Orange
    '#9b59b6', // Purple
    '#e74c3c'  // Red
];

// =============================================================================
// State
// =============================================================================

// Active segmented mission instance
let _mission = null;

// UI callbacks
let _onSegmentChange = null;
let _onCutRequested = null;
let _onResetRequested = null;

// =============================================================================
// DOM Helpers
// =============================================================================

function $(id) {
    return document.getElementById(id);
}

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = String(text);
    return div.innerHTML;
}

// =============================================================================
// Mission Management
// =============================================================================

/**
 * Get or create the segmented mission instance.
 * @returns {SegmentedMission}
 */
export function getMission() {
    if (!_mission) {
        _mission = new SegmentedMission();
    }
    return _mission;
}

/**
 * Set the mission instance (e.g., after import).
 * @param {SegmentedMission} mission
 */
export function setMission(mission) {
    _mission = mission;
    updateUI();
}

/**
 * Clear the mission.
 */
export function clearMission() {
    if (_mission) {
        _mission.clear();
    }
    _mission = null;
    updateUI();
}

/**
 * Check if a segmented mission is active.
 * @returns {boolean}
 */
export function isActive() {
    return _mission?.isActive() || false;
}

// =============================================================================
// Callbacks
// =============================================================================

/**
 * Set callback for segment change events.
 * @param {Function} callback - (segmentIndex, segmentData) => void
 */
export function setOnSegmentChange(callback) {
    _onSegmentChange = callback;
}

/**
 * Set callback for cut request events.
 * @param {Function} callback - () => void
 */
export function setOnCutRequested(callback) {
    _onCutRequested = callback;
}

/**
 * Set callback for reset request events.
 * @param {Function} callback - () => void
 */
export function setOnResetRequested(callback) {
    _onResetRequested = callback;
}

// =============================================================================
// Navigation
// =============================================================================

/**
 * Navigate to previous segment.
 */
export function previousSegment() {
    if (!_mission || !_mission.isActive()) return;

    const currentIdx = _mission.getCurrentIndex();
    if (currentIdx > 0) {
        _mission.setCurrentIndex(currentIdx - 1);
        notifySegmentChange();
        updateUI();
    }
}

/**
 * Navigate to next segment.
 */
export function nextSegment() {
    if (!_mission || !_mission.isActive()) return;

    const currentIdx = _mission.getCurrentIndex();
    const count = _mission.getSegmentCount();
    if (currentIdx < count - 1) {
        _mission.setCurrentIndex(currentIdx + 1);
        notifySegmentChange();
        updateUI();
    }
}

/**
 * Navigate to specific segment.
 * @param {number} index
 */
export function goToSegment(index) {
    if (!_mission || !_mission.isActive()) return;

    _mission.setCurrentIndex(index);
    notifySegmentChange();
    updateUI();
}

/**
 * Reset to first segment.
 */
export function resetToStart() {
    if (!_mission || !_mission.isActive()) return;

    _mission.resetToStart();
    notifySegmentChange();
    updateUI();

    if (_onResetRequested) {
        _onResetRequested();
    }
}

/**
 * Notify listeners of segment change.
 */
function notifySegmentChange() {
    if (_onSegmentChange && _mission) {
        const idx = _mission.getCurrentIndex();
        const segment = _mission.getSegment(idx);
        _onSegmentChange(idx, segment);
    }
}

// =============================================================================
// Cut Workflow
// =============================================================================

/**
 * Request a cut at current animation position.
 */
export function requestCut() {
    if (_onCutRequested) {
        _onCutRequested();
    }
}

// =============================================================================
// UI Update
// =============================================================================

/**
 * Update all segment manager UI elements.
 */
export function updateUI() {
    updateNavigationPanel();
    updateStatusPanel();
    updateSegmentBadges();
}

/**
 * Update navigation controls (prev/next buttons, segment indicator).
 */
function updateNavigationPanel() {
    const prevBtn = $('segment-prev');
    const nextBtn = $('segment-next');
    const indicator = $('segment-indicator');

    if (!_mission || !_mission.isActive()) {
        if (prevBtn) prevBtn.disabled = true;
        if (nextBtn) nextBtn.disabled = true;
        if (indicator) indicator.textContent = '';
        return;
    }

    const currentIdx = _mission.getCurrentIndex();
    const count = _mission.getSegmentCount();

    if (prevBtn) {
        prevBtn.disabled = currentIdx <= 0;
    }

    if (nextBtn) {
        nextBtn.disabled = currentIdx >= count - 1;
    }

    if (indicator) {
        indicator.textContent = `Segment ${currentIdx + 1} / ${count}`;
    }
}

/**
 * Update status panel (active targets, frozen targets, lost drones).
 */
function updateStatusPanel() {
    const statusEl = $('segment-status');
    if (!statusEl) return;

    if (!_mission || !_mission.isActive()) {
        statusEl.innerHTML = '<span class="segment-inactive">No segmented mission active</span>';
        return;
    }

    const segment = _mission.getCurrentSegment();
    if (!segment) {
        statusEl.innerHTML = '<span class="segment-inactive">No current segment</span>';
        return;
    }

    const activeTargets = segment.environment.targets.map(t => t.id);
    const visitedTargets = segment.visitedTargets || [];
    const lostDrones = segment.lostDrones || [];
    const addedDrones = segment.addedDrones || [];

    let html = '<div class="segment-status-grid">';

    // Active targets
    html += '<div class="status-row">';
    html += '<span class="status-label">Active:</span>';
    html += `<span class="status-value">${activeTargets.length > 0 ? activeTargets.join(', ') : 'None'}</span>`;
    html += '</div>';

    // Visited targets (frozen)
    if (visitedTargets.length > 0) {
        html += '<div class="status-row visited">';
        html += '<span class="status-label">Visited:</span>';
        html += `<span class="status-value">${visitedTargets.join(', ')}</span>`;
        html += '</div>';
    }

    // Lost drones
    if (lostDrones.length > 0) {
        html += '<div class="status-row lost">';
        html += '<span class="status-label">Lost:</span>';
        html += `<span class="status-value">D${lostDrones.join(', D')}</span>`;
        html += '</div>';
    }

    // Added drones
    if (addedDrones.length > 0) {
        html += '<div class="status-row added">';
        html += '<span class="status-label">Added:</span>';
        html += `<span class="status-value">D${addedDrones.join(', D')}</span>`;
        html += '</div>';
    }

    // Cut info
    if (segment.cut) {
        html += '<div class="status-row cut">';
        html += '<span class="status-label">Cut at:</span>';
        html += `<span class="status-value">${segment.cut.distance?.toFixed(1) || 0} units</span>`;
        html += '</div>';
    }

    html += '</div>';
    statusEl.innerHTML = html;
}

/**
 * Update segment badges on targets (shows which segment each target was visited in).
 */
function updateSegmentBadges() {
    // This integrates with the renderer to show segment badges
    // The actual rendering happens in renderer.js based on state
    if (_mission && _mission.isActive()) {
        const badgeMap = {};
        const segments = [];
        for (let i = 0; i < _mission.getSegmentCount(); i++) {
            const seg = _mission.getSegment(i);
            if (seg) {
                segments.push(seg);
                (seg.visitedTargets || []).forEach(tid => {
                    if (!badgeMap[tid]) {
                        badgeMap[tid] = i;
                    }
                });
            }
        }
        // Store badge map in state for renderer to use
        state.setState('segmentBadges', badgeMap);
    } else {
        state.setState('segmentBadges', null);
    }

    renderer.requestRender();
}

// =============================================================================
// Segment Timeline
// =============================================================================

/**
 * Render a clickable segment timeline.
 * @param {string} containerId - Container element ID
 */
export function renderTimeline(containerId = 'segment-timeline') {
    const container = $(containerId);
    if (!container) return;

    if (!_mission || !_mission.isActive()) {
        container.innerHTML = '';
        return;
    }

    const count = _mission.getSegmentCount();
    const currentIdx = _mission.getCurrentIndex();

    let html = '<div class="segment-timeline">';

    for (let i = 0; i < count; i++) {
        const segment = _mission.getSegment(i);
        const color = SEGMENT_COLORS[i % SEGMENT_COLORS.length];
        const isActive = i === currentIdx;
        const hasCut = !!segment?.cut;
        const lostCount = (segment?.lostDrones || []).length;

        html += `<div class="timeline-segment ${isActive ? 'active' : ''}" `;
        html += `data-index="${i}" style="border-color: ${color}">`;
        html += `<span class="segment-number" style="background-color: ${color}">${i + 1}</span>`;

        // Show info icons
        if (hasCut) {
            html += '<span class="segment-icon cut" title="Has cut point">✂</span>';
        }
        if (lostCount > 0) {
            html += `<span class="segment-icon lost" title="${lostCount} drone(s) lost">⚠</span>`;
        }

        html += '</div>';

        // Add connector between segments (except last)
        if (i < count - 1) {
            html += '<div class="timeline-connector"></div>';
        }
    }

    html += '</div>';
    container.innerHTML = html;

    // Attach click handlers
    container.querySelectorAll('.timeline-segment').forEach(el => {
        el.addEventListener('click', () => {
            const index = parseInt(el.dataset.index, 10);
            goToSegment(index);
        });
    });
}

// =============================================================================
// Event Listeners
// =============================================================================

/**
 * Attach event listeners to segment manager UI elements.
 */
export function attachEventListeners() {
    const prevBtn = $('segment-prev');
    const nextBtn = $('segment-next');
    const resetBtn = $('segment-reset');
    const cutBtn = $('segment-cut');

    if (prevBtn) {
        prevBtn.addEventListener('click', previousSegment);
    }

    if (nextBtn) {
        nextBtn.addEventListener('click', nextSegment);
    }

    if (resetBtn) {
        resetBtn.addEventListener('click', resetToStart);
    }

    if (cutBtn) {
        cutBtn.addEventListener('click', requestCut);
    }
}

// =============================================================================
// Import/Export Integration
// =============================================================================

/**
 * Import segmented mission from JSON data.
 * @param {Object} data - Parsed JSON
 * @returns {Object|null} Initial drone configs for UI, or null on failure
 */
export function importFromJson(data) {
    if (!isSegmentedMission(data)) {
        console.warn('[SegmentManagerUI] Data is not a segmented mission');
        return null;
    }

    const mission = new SegmentedMission();
    const droneConfigs = mission.importFromJson(data);

    if (droneConfigs) {
        _mission = mission;
        updateUI();
        return droneConfigs;
    }

    return null;
}

/**
 * Export current mission to JSON.
 * @returns {Object|null}
 */
export function exportToJson() {
    if (!_mission || !_mission.isActive()) {
        return null;
    }
    return _mission.exportToJson();
}

// =============================================================================
// Debug
// =============================================================================

/**
 * Get debug info about current segment manager state.
 * @returns {Object}
 */
export function getDebugInfo() {
    return {
        isActive: isActive(),
        missionInfo: _mission?.getDebugInfo() || null
    };
}

// =============================================================================
// Module Exports
// =============================================================================

export default {
    // Mission management
    getMission,
    setMission,
    clearMission,
    isActive,

    // Callbacks
    setOnSegmentChange,
    setOnCutRequested,
    setOnResetRequested,

    // Navigation
    previousSegment,
    nextSegment,
    goToSegment,
    resetToStart,

    // Cut workflow
    requestCut,

    // UI
    updateUI,
    renderTimeline,
    attachEventListeners,

    // Import/Export
    importFromJson,
    exportToJson,

    // Debug
    getDebugInfo
};
