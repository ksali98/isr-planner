/**
 * Mission Control Module - API calls and mission management.
 *
 * This module handles:
 * - API communication with the backend
 * - Mission solving and optimization
 * - Post-optimization (insert missed, swap closer, crossing removal)
 *
 * Phase 4 Migration Notes:
 * - API call functions should be extracted from isr.js
 * - Key functions to migrate:
 *   - runPlanner()
 *   - solveWithAllocation()
 *   - insertMissed()
 *   - swapCloser()
 *   - crossingRemoval()
 *   - acceptSolution()
 */

import state from './state.js';

// =============================================================================
// Configuration
// =============================================================================

const API_BASE = '';  // Same origin

// =============================================================================
// API Helpers
// =============================================================================

/**
 * Make API request with JSON body.
 * @param {string} endpoint - API endpoint
 * @param {Object} body - Request body
 * @returns {Promise<Object>} Response data
 */
async function apiPost(endpoint, body) {
    const response = await fetch(`${API_BASE}${endpoint}`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(body)
    });

    if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
    }

    return response.json();
}

/**
 * Make GET API request.
 * @param {string} endpoint - API endpoint
 * @returns {Promise<Object>} Response data
 */
async function apiGet(endpoint) {
    const response = await fetch(`${API_BASE}${endpoint}`);

    if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
    }

    return response.json();
}

// =============================================================================
// Solving Functions
// =============================================================================

/**
 * Solve mission with target allocation.
 * @param {Object} options - Solve options
 * @returns {Promise<Object>} Solution data
 */
export async function solveWithAllocation(options = {}) {
    const env = state.getEnvironment();
    const droneConfigs = state.getDroneConfigs();

    const request = {
        env,
        drone_configs: droneConfigs,
        allocation_strategy: options.strategy || 'efficient',
        use_sam_aware_distances: options.useSamAwareDistances || false,
        post_optimize: options.postOptimize !== false,
        visited_targets: options.visitedTargets || [],
        is_checkpoint_replan: options.isCheckpointReplan || false,
        mission_id: options.missionId || null
    };

    const result = await apiPost('/api/solve_with_allocation', request);

    if (result.success) {
        state.setSolution({
            routes: result.routes,
            sequences: result.sequences || {},
            allocations: result.allocations || {},
            distance_matrix: result.distance_matrix
        });
    }

    return result;
}

/**
 * Basic solve (without allocation).
 * @param {Object} options - Solve options
 * @returns {Promise<Object>} Solution data
 */
export async function solve(options = {}) {
    const env = state.getEnvironment();
    const droneConfigs = state.getDroneConfigs();

    const request = {
        env,
        drone_configs: droneConfigs,
        allocation_strategy: options.strategy || 'efficient',
        mission_id: options.missionId || null
    };

    const result = await apiPost('/api/solve', request);

    if (result.success) {
        state.setSolution({
            routes: result.routes,
            sequences: result.sequences || {},
            allocations: result.allocations || {},
            distance_matrix: result.distance_matrix
        });
    }

    return result;
}

// =============================================================================
// Post-Optimization Functions
// =============================================================================

/**
 * Insert missed targets into routes.
 * @returns {Promise<Object>} Optimized solution
 */
export async function insertMissed() {
    const solution = state.getSolution();
    const env = state.getEnvironment();
    const droneConfigs = state.getDroneConfigs();

    const request = {
        solution: {
            routes: solution.routes,
            distance_matrix: solution.distance_matrix
        },
        env,
        drone_configs: droneConfigs
    };

    const result = await apiPost('/api/insert_missed_optimize', request);

    if (result.success) {
        state.mergeState('solution', {
            routes: result.routes
        });
    }

    return result;
}

/**
 * Swap targets to closer drone trajectories.
 * @param {Object} options - Swap options
 * @returns {Promise<Object>} Optimized solution
 */
export async function swapCloser(options = {}) {
    const solution = state.getSolution();
    const env = state.getEnvironment();
    const droneConfigs = state.getDroneConfigs();

    const request = {
        solution: {
            routes: solution.routes,
            distance_matrix: solution.distance_matrix
        },
        env,
        drone_configs: droneConfigs,
        visited_targets: options.visitedTargets || [],
        auto_iterate: options.autoIterate !== false,
        auto_regen: options.autoRegen !== false,
        debug: options.debug || false
    };

    const result = await apiPost('/api/trajectory_swap_optimize', request);

    if (result.success) {
        state.mergeState('solution', {
            routes: result.routes
        });
    }

    return result;
}

/**
 * Remove route crossings using 2-opt.
 * @returns {Promise<Object>} Optimized solution
 */
export async function crossingRemoval() {
    const solution = state.getSolution();
    const env = state.getEnvironment();
    const droneConfigs = state.getDroneConfigs();

    const request = {
        solution: {
            routes: solution.routes,
            distance_matrix: solution.distance_matrix
        },
        env,
        drone_configs: droneConfigs
    };

    const result = await apiPost('/api/crossing_removal_optimize', request);

    if (result.success) {
        state.mergeState('solution', {
            routes: result.routes
        });
    }

    return result;
}

// =============================================================================
// Coverage Statistics
// =============================================================================

/**
 * Get coverage statistics for current solution.
 * @returns {Promise<Object>} Coverage stats
 */
export async function getCoverageStats() {
    const solution = state.getSolution();
    const env = state.getEnvironment();

    const request = {
        solution: { routes: solution.routes },
        env
    };

    return apiPost('/api/coverage_stats', request);
}

// =============================================================================
// Distance Matrix
// =============================================================================

/**
 * Prepare distance matrix for environment.
 * @param {number} buffer - SAM buffer distance
 * @returns {Promise<Object>} Matrix preparation result
 */
export async function prepareMatrix(buffer = 0) {
    const env = state.getEnvironment();

    return apiPost('/api/prepare_matrix', { env, buffer });
}

/**
 * Get current matrix status.
 * @returns {Promise<Object>} Matrix status
 */
export async function getMatrixStatus() {
    return apiGet('/api/matrix_status');
}

/**
 * Clear cached distance matrix.
 * @returns {Promise<Object>} Result
 */
export async function clearMatrix() {
    return apiPost('/api/clear_matrix', {});
}

// =============================================================================
// Environment Management
// =============================================================================

/**
 * Save environment to server.
 * @returns {Promise<Object>} Save result
 */
export async function saveEnvironment() {
    const env = state.getEnvironment();
    return apiPost('/api/environment', env);
}

/**
 * Load environment from server.
 * @returns {Promise<Object>} Environment data
 */
export async function loadEnvironment() {
    const result = await apiGet('/api/environment');

    if (result.success && result.env) {
        state.setEnvironment(result.env);
    }

    return result;
}

export default {
    solveWithAllocation,
    solve,
    insertMissed,
    swapCloser,
    crossingRemoval,
    getCoverageStats,
    prepareMatrix,
    getMatrixStatus,
    clearMatrix,
    saveEnvironment,
    loadEnvironment
};
