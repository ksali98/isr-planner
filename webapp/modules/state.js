/**
 * State Module - Single source of truth for application state.
 *
 * This module manages all application state in a centralized location,
 * making it easier to track changes and debug state-related issues.
 *
 * Phase 4 Migration Notes:
 * - State is currently scattered throughout isr.js
 * - This module should centralize all state management
 * - Provides getter/setter functions with optional validation
 * - Emits events on state changes for reactive updates
 */

// =============================================================================
// Application State
// =============================================================================

const AppState = {
    // Environment state
    environment: {
        airports: [],
        targets: [],
        sams: [],
        polygons: [],
        boundary: null,
        synthetic_starts: {}
    },

    // Drone configuration
    droneConfigs: {},

    // Current solution
    solution: {
        routes: {},
        sequences: {},
        allocations: {},
        distance_matrix: null
    },

    // Segmented mission state
    segments: [],
    currentSegmentIndex: 0,
    frozenProgress: {},  // { droneId: { position: [x,y], visitedTargets: [] } }

    // Animation state
    animation: {
        isPlaying: false,
        currentTime: 0,
        speed: 1.0,
        dronePositions: {}
    },

    // UI state
    ui: {
        editMode: false,
        selectedTool: null,
        selectedElement: null,
        zoom: 1.0,
        pan: { x: 0, y: 0 }
    },

    // Mission metadata
    mission: {
        id: null,
        name: null,
        created: null,
        modified: null
    }
};

// =============================================================================
// State Listeners
// =============================================================================

const stateListeners = new Map();

/**
 * Subscribe to state changes.
 * @param {string} path - Dot-notation path to state property (e.g., "solution.routes")
 * @param {Function} callback - Called when state changes
 * @returns {Function} Unsubscribe function
 */
export function subscribe(path, callback) {
    if (!stateListeners.has(path)) {
        stateListeners.set(path, new Set());
    }
    stateListeners.get(path).add(callback);

    return () => stateListeners.get(path).delete(callback);
}

/**
 * Notify listeners of state change.
 * @param {string} path - State path that changed
 * @param {*} newValue - New value
 * @param {*} oldValue - Previous value
 */
function notifyListeners(path, newValue, oldValue) {
    const listeners = stateListeners.get(path);
    if (listeners) {
        listeners.forEach(callback => callback(newValue, oldValue, path));
    }
}

// =============================================================================
// State Getters
// =============================================================================

/**
 * Get state value at path.
 * @param {string} path - Dot-notation path (e.g., "environment.targets")
 * @returns {*} State value
 */
export function getState(path) {
    const parts = path.split('.');
    let value = AppState;
    for (const part of parts) {
        if (value === undefined || value === null) return undefined;
        value = value[part];
    }
    return value;
}

/**
 * Get a copy of the entire state (for debugging/serialization).
 * @returns {Object} Deep copy of state
 */
export function getFullState() {
    return JSON.parse(JSON.stringify(AppState));
}

// =============================================================================
// State Setters
// =============================================================================

/**
 * Set state value at path.
 * @param {string} path - Dot-notation path
 * @param {*} value - New value
 */
export function setState(path, value) {
    const parts = path.split('.');
    const lastKey = parts.pop();
    let target = AppState;

    for (const part of parts) {
        if (target[part] === undefined) {
            target[part] = {};
        }
        target = target[part];
    }

    const oldValue = target[lastKey];
    target[lastKey] = value;

    notifyListeners(path, value, oldValue);
}

/**
 * Merge object into state at path.
 * @param {string} path - Dot-notation path
 * @param {Object} updates - Object to merge
 */
export function mergeState(path, updates) {
    const current = getState(path) || {};
    setState(path, { ...current, ...updates });
}

// =============================================================================
// Convenience Accessors
// =============================================================================

export function getEnvironment() {
    return getState('environment');
}

export function setEnvironment(env) {
    setState('environment', env);
}

export function getDroneConfigs() {
    return getState('droneConfigs');
}

export function setDroneConfigs(configs) {
    setState('droneConfigs', configs);
}

export function getSolution() {
    return getState('solution');
}

export function setSolution(solution) {
    setState('solution', solution);
}

export function getSegments() {
    return getState('segments');
}

export function setSegments(segments) {
    setState('segments', segments);
}

export function isEditMode() {
    return getState('ui.editMode');
}

export function setEditMode(enabled) {
    setState('ui.editMode', enabled);
}

// =============================================================================
// State Reset
// =============================================================================

/**
 * Reset state to defaults.
 */
export function resetState() {
    AppState.environment = {
        airports: [],
        targets: [],
        sams: [],
        polygons: [],
        boundary: null,
        synthetic_starts: {}
    };
    AppState.droneConfigs = {};
    AppState.solution = {
        routes: {},
        sequences: {},
        allocations: {},
        distance_matrix: null
    };
    AppState.segments = [];
    AppState.currentSegmentIndex = 0;
    AppState.frozenProgress = {};
    AppState.animation = {
        isPlaying: false,
        currentTime: 0,
        speed: 1.0,
        dronePositions: {}
    };
    AppState.ui = {
        editMode: false,
        selectedTool: null,
        selectedElement: null,
        zoom: 1.0,
        pan: { x: 0, y: 0 }
    };
    AppState.mission = {
        id: null,
        name: null,
        created: null,
        modified: null
    };

    notifyListeners('*', AppState, null);
}

export default {
    subscribe,
    getState,
    setState,
    mergeState,
    getFullState,
    resetState,
    getEnvironment,
    setEnvironment,
    getDroneConfigs,
    setDroneConfigs,
    getSolution,
    setSolution,
    getSegments,
    setSegments,
    isEditMode,
    setEditMode
};
