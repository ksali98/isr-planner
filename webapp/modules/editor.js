/**
 * Editor Module - Environment editing functionality.
 *
 * This module handles:
 * - Edit mode toggle
 * - Adding/removing airports, targets, SAMs
 * - Drag and drop positioning
 * - Element selection
 *
 * Phase 4 Migration Notes:
 * - Editor code should be extracted from isr.js
 * - Key functions to migrate:
 *   - toggleEditMode()
 *   - addAirport()
 *   - addTarget()
 *   - addSAM()
 *   - deleteElement()
 *   - handleDrag()
 *   - handleDrop()
 */

import state from './state.js';
import renderer from './renderer.js';

// =============================================================================
// Edit Mode
// =============================================================================

/**
 * Toggle edit mode.
 * @returns {boolean} New edit mode state
 */
export function toggleEditMode() {
    const current = state.isEditMode();
    state.setEditMode(!current);
    return !current;
}

/**
 * Enter edit mode.
 */
export function enterEditMode() {
    state.setEditMode(true);
}

/**
 * Exit edit mode.
 */
export function exitEditMode() {
    state.setEditMode(false);
    state.setState('ui.selectedElement', null);
    state.setState('ui.selectedTool', null);
}

// =============================================================================
// Tool Selection
// =============================================================================

/**
 * Select an editing tool.
 * @param {string} tool - Tool name ('airport', 'target', 'sam', 'delete', 'select')
 */
export function selectTool(tool) {
    state.setState('ui.selectedTool', tool);
}

/**
 * Get current tool.
 * @returns {string} Current tool name
 */
export function getCurrentTool() {
    return state.getState('ui.selectedTool');
}

// =============================================================================
// Element Creation
// =============================================================================

/**
 * Add an airport at position.
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {string} id - Optional airport ID
 * @returns {Object} Created airport
 */
export function addAirport(x, y, id = null) {
    const env = state.getEnvironment();
    const airports = env.airports || [];

    // Generate ID if not provided
    if (!id) {
        const existingIds = airports.map(a => a.id);
        let num = 1;
        while (existingIds.includes(`A${num}`)) num++;
        id = `A${num}`;
    }

    const airport = { id, x, y };
    airports.push(airport);

    state.setState('environment.airports', airports);
    renderer.requestRender();

    return airport;
}

/**
 * Add a target at position.
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {Object} options - Target options (type, priority)
 * @returns {Object} Created target
 */
export function addTarget(x, y, options = {}) {
    const env = state.getEnvironment();
    const targets = env.targets || [];

    // Generate ID
    const existingIds = targets.map(t => t.id);
    let num = 1;
    while (existingIds.includes(`T${num}`)) num++;
    const id = `T${num}`;

    const target = {
        id,
        x,
        y,
        type: options.type || 'a',
        priority: options.priority || 1
    };

    targets.push(target);
    state.setState('environment.targets', targets);
    renderer.requestRender();

    return target;
}

/**
 * Add a SAM at position.
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {number} range - SAM range
 * @returns {Object} Created SAM
 */
export function addSAM(x, y, range = 50) {
    const env = state.getEnvironment();
    const sams = env.sams || [];

    const sam = {
        pos: [x, y],
        range
    };

    sams.push(sam);
    state.setState('environment.sams', sams);
    renderer.requestRender();

    return sam;
}

// =============================================================================
// Element Selection
// =============================================================================

/**
 * Find element at position.
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 * @param {number} threshold - Selection threshold
 * @returns {Object|null} Found element or null
 */
export function findElementAt(x, y, threshold = 15) {
    const env = state.getEnvironment();

    // Check airports
    for (const airport of (env.airports || [])) {
        const dx = airport.x - x;
        const dy = airport.y - y;
        if (Math.sqrt(dx * dx + dy * dy) < threshold) {
            return { type: 'airport', element: airport };
        }
    }

    // Check targets
    for (const target of (env.targets || [])) {
        const dx = target.x - x;
        const dy = target.y - y;
        if (Math.sqrt(dx * dx + dy * dy) < threshold) {
            return { type: 'target', element: target };
        }
    }

    // Check SAMs
    for (const sam of (env.sams || [])) {
        const pos = sam.pos || sam.position;
        if (!pos) continue;
        const dx = pos[0] - x;
        const dy = pos[1] - y;
        if (Math.sqrt(dx * dx + dy * dy) < threshold) {
            return { type: 'sam', element: sam };
        }
    }

    return null;
}

/**
 * Select element at position.
 * @param {number} x - X coordinate
 * @param {number} y - Y coordinate
 */
export function selectAt(x, y) {
    const found = findElementAt(x, y);
    state.setState('ui.selectedElement', found);
    renderer.requestRender();
}

/**
 * Clear selection.
 */
export function clearSelection() {
    state.setState('ui.selectedElement', null);
    renderer.requestRender();
}

// =============================================================================
// Element Modification
// =============================================================================

/**
 * Move element to new position.
 * @param {Object} element - Element to move
 * @param {string} type - Element type
 * @param {number} x - New X coordinate
 * @param {number} y - New Y coordinate
 */
export function moveElement(element, type, x, y) {
    const env = state.getEnvironment();

    if (type === 'airport') {
        const airports = env.airports || [];
        const airport = airports.find(a => a.id === element.id);
        if (airport) {
            airport.x = x;
            airport.y = y;
            state.setState('environment.airports', airports);
        }
    } else if (type === 'target') {
        const targets = env.targets || [];
        const target = targets.find(t => t.id === element.id);
        if (target) {
            target.x = x;
            target.y = y;
            state.setState('environment.targets', targets);
        }
    } else if (type === 'sam') {
        const sams = env.sams || [];
        const idx = sams.indexOf(element);
        if (idx >= 0) {
            sams[idx].pos = [x, y];
            state.setState('environment.sams', sams);
        }
    }

    renderer.requestRender();
}

/**
 * Delete element.
 * @param {Object} element - Element to delete
 * @param {string} type - Element type
 */
export function deleteElement(element, type) {
    const env = state.getEnvironment();

    if (type === 'airport') {
        const airports = (env.airports || []).filter(a => a.id !== element.id);
        state.setState('environment.airports', airports);
    } else if (type === 'target') {
        const targets = (env.targets || []).filter(t => t.id !== element.id);
        state.setState('environment.targets', targets);
    } else if (type === 'sam') {
        const sams = (env.sams || []).filter(s => s !== element);
        state.setState('environment.sams', sams);
    }

    clearSelection();
    renderer.requestRender();
}

/**
 * Delete selected element.
 */
export function deleteSelected() {
    const selected = state.getState('ui.selectedElement');
    if (selected) {
        deleteElement(selected.element, selected.type);
    }
}

// =============================================================================
// Canvas Event Handlers
// =============================================================================

/**
 * Handle canvas click in edit mode.
 * @param {number} x - Canvas X coordinate
 * @param {number} y - Canvas Y coordinate
 */
export function handleClick(x, y) {
    if (!state.isEditMode()) return;

    const tool = getCurrentTool();

    switch (tool) {
        case 'airport':
            addAirport(x, y);
            break;
        case 'target':
            addTarget(x, y);
            break;
        case 'sam':
            addSAM(x, y);
            break;
        case 'delete':
            const found = findElementAt(x, y);
            if (found) {
                deleteElement(found.element, found.type);
            }
            break;
        case 'select':
        default:
            selectAt(x, y);
            break;
    }
}

export default {
    toggleEditMode,
    enterEditMode,
    exitEditMode,
    selectTool,
    getCurrentTool,
    addAirport,
    addTarget,
    addSAM,
    findElementAt,
    selectAt,
    clearSelection,
    moveElement,
    deleteElement,
    deleteSelected,
    handleClick
};
