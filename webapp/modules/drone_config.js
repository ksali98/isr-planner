/**
 * Drone Configuration Module - Manages drone settings and UI.
 *
 * This module handles:
 * - Drone configuration state (enabled, fuel, airports, target access)
 * - UI synchronization with checkboxes and inputs
 * - Configuration validation
 * - Event listener attachment
 *
 * Each drone has:
 * - enabled: boolean - Whether drone is active
 * - fuel_budget: number - Maximum fuel units
 * - start_airport: string - Starting airport ID
 * - end_airport: string - Ending airport ID (or "-" for flexible)
 * - target_access: object - { a: bool, b: bool, c: bool, d: bool, e: bool }
 */

import state from './state.js';
import renderer from './renderer.js';

// =============================================================================
// Constants
// =============================================================================

const MAX_DRONES = 5;
const DEFAULT_FUEL_BUDGET = 150;
const TARGET_TYPES = ['a', 'b', 'c', 'd', 'e'];

// =============================================================================
// Drone Config Structure
// =============================================================================

/**
 * Create default drone configuration.
 * @param {number} droneId - Drone number (1-5)
 * @param {string} defaultAirport - Default airport ID
 * @returns {Object} Drone configuration
 */
function createDefaultConfig(droneId, defaultAirport = 'A1') {
    const airportId = `A${droneId}`;
    return {
        enabled: true,
        fuel_budget: DEFAULT_FUEL_BUDGET,
        start_airport: airportId,
        end_airport: airportId,
        target_access: {
            a: true,
            b: true,
            c: true,
            d: true,
            e: true
        }
    };
}

/**
 * Validate drone configuration.
 * @param {Object} config - Configuration to validate
 * @returns {Object} { valid: boolean, errors: string[] }
 */
export function validateConfig(config) {
    const errors = [];

    if (typeof config.enabled !== 'boolean') {
        errors.push('enabled must be boolean');
    }

    if (typeof config.fuel_budget !== 'number' || config.fuel_budget < 0) {
        errors.push('fuel_budget must be non-negative number');
    }

    if (!config.start_airport || typeof config.start_airport !== 'string') {
        errors.push('start_airport must be a string');
    }

    if (!config.end_airport || typeof config.end_airport !== 'string') {
        errors.push('end_airport must be a string');
    }

    if (!config.target_access || typeof config.target_access !== 'object') {
        errors.push('target_access must be an object');
    }

    return {
        valid: errors.length === 0,
        errors
    };
}

// =============================================================================
// State Management
// =============================================================================

/**
 * Get all drone configurations.
 * @returns {Object} All drone configs keyed by drone ID
 */
export function getAllConfigs() {
    return state.getDroneConfigs();
}

/**
 * Get configuration for a specific drone.
 * @param {string|number} droneId - Drone ID
 * @returns {Object|null} Drone configuration or null
 */
export function getConfig(droneId) {
    const configs = state.getDroneConfigs();
    return configs[String(droneId)] || null;
}

/**
 * Set configuration for a specific drone.
 * @param {string|number} droneId - Drone ID
 * @param {Object} config - Configuration object
 */
export function setConfig(droneId, config) {
    const configs = state.getDroneConfigs();
    configs[String(droneId)] = { ...config };
    state.setDroneConfigs(configs);
}

/**
 * Update a single property of a drone's configuration.
 * @param {string|number} droneId - Drone ID
 * @param {string} property - Property name
 * @param {*} value - New value
 */
export function updateConfigProperty(droneId, property, value) {
    const config = getConfig(droneId);
    if (config) {
        config[property] = value;
        setConfig(droneId, config);
    }
}

/**
 * Check if a drone is enabled.
 * @param {string|number} droneId - Drone ID
 * @returns {boolean}
 */
export function isEnabled(droneId) {
    const config = getConfig(droneId);
    return config?.enabled ?? false;
}

/**
 * Get list of enabled drone IDs.
 * @returns {string[]} Array of drone IDs that are enabled
 */
export function getEnabledDroneIds() {
    const configs = getAllConfigs();
    return Object.entries(configs)
        .filter(([, cfg]) => cfg.enabled)
        .map(([id]) => id);
}

/**
 * Get list of disabled drone IDs.
 * @returns {string[]} Array of drone IDs that are disabled
 */
export function getDisabledDroneIds() {
    const configs = getAllConfigs();
    return Object.entries(configs)
        .filter(([, cfg]) => !cfg.enabled)
        .map(([id]) => id);
}

// =============================================================================
// Initialization
// =============================================================================

/**
 * Initialize drone configurations from environment.
 * Sets up default configs and syncs with UI.
 * @param {Object} env - Environment object with airports and drone_configs
 */
export function initFromEnvironment(env) {
    const airports = env?.airports || [];
    const airportIds = airports.map(a => String(a.id));
    const defaultAirport = airportIds[0] || 'A1';
    const savedConfigs = env?.drone_configs || {};

    const newConfigs = {};

    for (let did = 1; did <= MAX_DRONES; did++) {
        const idStr = String(did);
        const saved = savedConfigs[idStr] || {};

        // Use saved config if available, otherwise use defaults
        newConfigs[idStr] = {
            enabled: saved.enabled !== undefined ? saved.enabled : true,
            fuel_budget: saved.fuel_budget !== undefined ? saved.fuel_budget : DEFAULT_FUEL_BUDGET,
            start_airport: saved.start_airport || (airportIds.includes(`A${did}`) ? `A${did}` : defaultAirport),
            end_airport: saved.end_airport || (airportIds.includes(`A${did}`) ? `A${did}` : defaultAirport),
            target_access: saved.target_access || { a: true, b: true, c: true, d: true, e: true }
        };
    }

    state.setDroneConfigs(newConfigs);

    // Populate dropdowns and sync UI
    populateAirportDropdowns(airportIds);
    syncUiWithConfigs();
}

/**
 * Initialize drone configurations with defaults.
 */
export function initDefaults() {
    const newConfigs = {};
    for (let did = 1; did <= MAX_DRONES; did++) {
        newConfigs[String(did)] = createDefaultConfig(did);
    }
    state.setDroneConfigs(newConfigs);
}

// =============================================================================
// UI Synchronization
// =============================================================================

/**
 * Get DOM element by ID.
 * @param {string} id - Element ID
 * @returns {HTMLElement|null}
 */
function $(id) {
    return document.getElementById(id);
}

/**
 * Populate airport dropdown options.
 * @param {string[]} airportIds - Available airport IDs
 */
export function populateAirportDropdowns(airportIds) {
    for (let did = 1; did <= MAX_DRONES; did++) {
        // Start airport dropdown
        const startSel = $(`cfg-d${did}-start`);
        if (startSel) {
            startSel.innerHTML = '';
            airportIds.forEach(aid => {
                const opt = document.createElement('option');
                opt.value = aid;
                opt.textContent = aid;
                startSel.appendChild(opt);
            });
        }

        // End airport dropdown (includes "Any" option)
        const endSel = $(`cfg-d${did}-end`);
        if (endSel) {
            endSel.innerHTML = '';
            // Add flexible endpoint option first
            const flexOpt = document.createElement('option');
            flexOpt.value = '-';
            flexOpt.textContent = 'Any';
            endSel.appendChild(flexOpt);
            // Add airport options
            airportIds.forEach(aid => {
                const opt = document.createElement('option');
                opt.value = aid;
                opt.textContent = aid;
                endSel.appendChild(opt);
            });
        }
    }
}

/**
 * Sync UI elements with current drone configurations.
 */
export function syncUiWithConfigs() {
    const configs = getAllConfigs();

    for (let did = 1; did <= MAX_DRONES; did++) {
        const idStr = String(did);
        const cfg = configs[idStr];
        if (!cfg) continue;

        const cbEnabled = $(`cfg-d${did}-enabled`);
        const fuelInput = $(`cfg-d${did}-fuel`);
        const startSel = $(`cfg-d${did}-start`);
        const endSel = $(`cfg-d${did}-end`);

        if (cbEnabled) cbEnabled.checked = cfg.enabled;
        if (fuelInput) fuelInput.value = cfg.fuel_budget;
        if (startSel && cfg.start_airport) startSel.value = cfg.start_airport;
        if (endSel && cfg.end_airport) endSel.value = cfg.end_airport;

        // Target access checkboxes
        TARGET_TYPES.forEach(t => {
            const cb = $(`cfg-d${did}-type-${t}`);
            if (cb && cfg.target_access?.[t] !== undefined) {
                cb.checked = cfg.target_access[t];
            }
        });
    }
}

/**
 * Load drone configs from object and sync UI.
 * @param {Object} configs - Drone configurations object
 */
export function loadAndSyncConfigs(configs) {
    state.setDroneConfigs(JSON.parse(JSON.stringify(configs)));
    syncUiWithConfigs();
}

// =============================================================================
// Event Listeners
// =============================================================================

// Callback for when configuration changes (for invalidating mission)
let onConfigChangeCallback = null;

/**
 * Set callback for configuration changes.
 * @param {Function} callback - Function to call with change description
 */
export function setOnConfigChange(callback) {
    onConfigChangeCallback = callback;
}

/**
 * Notify that configuration changed.
 * @param {string} description - Description of the change
 */
function notifyConfigChange(description) {
    if (onConfigChangeCallback) {
        onConfigChangeCallback(description);
    }
    renderer.requestRender();
}

/**
 * Attach event listeners to drone configuration UI elements.
 */
export function attachEventListeners() {
    for (let did = 1; did <= MAX_DRONES; did++) {
        const idStr = String(did);

        const cbEnabled = $(`cfg-d${did}-enabled`);
        const fuelInput = $(`cfg-d${did}-fuel`);
        const startSel = $(`cfg-d${did}-start`);
        const endSel = $(`cfg-d${did}-end`);

        if (cbEnabled) {
            cbEnabled.addEventListener('change', () => {
                updateConfigProperty(idStr, 'enabled', cbEnabled.checked);
                notifyConfigChange(`Drone ${idStr} enabled changed`);
            });
        }

        if (fuelInput) {
            fuelInput.addEventListener('change', () => {
                const v = parseFloat(fuelInput.value || '0');
                updateConfigProperty(idStr, 'fuel_budget', isNaN(v) ? 0 : v);
                notifyConfigChange(`Drone ${idStr} fuel_budget changed`);
            });
        }

        if (startSel) {
            startSel.addEventListener('change', () => {
                updateConfigProperty(idStr, 'start_airport', startSel.value);
                notifyConfigChange(`Drone ${idStr} start_airport changed`);
            });
        }

        if (endSel) {
            endSel.addEventListener('change', () => {
                updateConfigProperty(idStr, 'end_airport', endSel.value);
                notifyConfigChange(`Drone ${idStr} end_airport changed`);
            });
        }

        TARGET_TYPES.forEach(t => {
            const cb = $(`cfg-d${did}-type-${t}`);
            if (!cb) return;
            cb.addEventListener('change', () => {
                const config = getConfig(idStr);
                if (config) {
                    config.target_access[t] = cb.checked;
                    setConfig(idStr, config);
                    notifyConfigChange(`Drone ${idStr} target_access changed`);
                }
            });
        });
    }
}

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * Check if a drone can access a target type.
 * @param {string|number} droneId - Drone ID
 * @param {string} targetType - Target type (a-e)
 * @returns {boolean}
 */
export function canAccessTargetType(droneId, targetType) {
    const config = getConfig(droneId);
    return config?.target_access?.[targetType.toLowerCase()] ?? true;
}

/**
 * Get total enabled drones count.
 * @returns {number}
 */
export function getEnabledCount() {
    return getEnabledDroneIds().length;
}

/**
 * Build drone configs object for API requests.
 * @returns {Object} Drone configs ready for API
 */
export function buildApiConfigs() {
    const configs = getAllConfigs();
    const result = {};

    Object.entries(configs).forEach(([id, cfg]) => {
        if (cfg.enabled) {
            result[id] = {
                enabled: true,
                fuel_budget: cfg.fuel_budget,
                start_airport: cfg.start_airport,
                end_airport: cfg.end_airport,
                target_access: { ...cfg.target_access }
            };
        }
    });

    return result;
}

/**
 * Clone current configurations.
 * @returns {Object} Deep copy of all configs
 */
export function cloneConfigs() {
    return JSON.parse(JSON.stringify(getAllConfigs()));
}

// =============================================================================
// Module Exports
// =============================================================================

export default {
    // State management
    getAllConfigs,
    getConfig,
    setConfig,
    updateConfigProperty,
    isEnabled,
    getEnabledDroneIds,
    getDisabledDroneIds,

    // Initialization
    initFromEnvironment,
    initDefaults,

    // UI synchronization
    populateAirportDropdowns,
    syncUiWithConfigs,
    loadAndSyncConfigs,

    // Event handling
    setOnConfigChange,
    attachEventListeners,

    // Utilities
    validateConfig,
    canAccessTargetType,
    getEnabledCount,
    buildApiConfigs,
    cloneConfigs,

    // Constants
    MAX_DRONES,
    DEFAULT_FUEL_BUDGET,
    TARGET_TYPES
};
