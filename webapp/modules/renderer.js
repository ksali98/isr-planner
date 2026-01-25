/**
 * Renderer Module - Canvas rendering for the ISR Planner.
 *
 * This module handles all canvas-based rendering including:
 * - Environment elements (airports, targets, SAMs, polygons)
 * - Drone routes and trajectories
 * - Animation frames
 * - UI overlays
 *
 * Phase 4 Migration Notes:
 * - Rendering code should be extracted from isr.js
 * - Key functions to migrate:
 *   - drawEnvironment()
 *   - renderCanvas()
 *   - drawAirports()
 *   - drawTargets()
 *   - drawSAMs()
 *   - drawPolygons()
 *   - drawRoutes()
 *   - drawTrajectories()
 *   - drawDrones()
 */

import state from './state.js';

// =============================================================================
// Canvas Setup
// =============================================================================

let canvas = null;
let ctx = null;
let offscreenCanvas = null;
let offscreenCtx = null;

/**
 * Initialize the renderer with a canvas element.
 * @param {HTMLCanvasElement} canvasElement - The canvas to render to
 */
export function initRenderer(canvasElement) {
    canvas = canvasElement;
    ctx = canvas.getContext('2d');

    // Create offscreen canvas for double buffering
    offscreenCanvas = document.createElement('canvas');
    offscreenCanvas.width = canvas.width;
    offscreenCanvas.height = canvas.height;
    offscreenCtx = offscreenCanvas.getContext('2d');
}

/**
 * Resize canvas to match container.
 * @param {number} width - New width
 * @param {number} height - New height
 */
export function resizeCanvas(width, height) {
    canvas.width = width;
    canvas.height = height;
    offscreenCanvas.width = width;
    offscreenCanvas.height = height;
}

// =============================================================================
// Color Constants
// =============================================================================

const COLORS = {
    background: '#1a1a2e',
    grid: '#2d2d44',

    airport: '#4ecdc4',
    airportStroke: '#2d8b84',

    target: {
        a: '#ff6b6b',
        b: '#4ecdc4',
        c: '#45b7d1',
        default: '#ffd93d'
    },

    sam: {
        fill: 'rgba(255, 107, 107, 0.2)',
        stroke: '#ff6b6b'
    },

    polygon: {
        fill: 'rgba(255, 215, 0, 0.1)',
        stroke: '#ffd700'
    },

    drone: [
        '#ff6b6b',  // Drone 1 - Red
        '#4ecdc4',  // Drone 2 - Teal
        '#ffd93d',  // Drone 3 - Yellow
        '#45b7d1',  // Drone 4 - Blue
        '#96ceb4'   // Drone 5 - Green
    ],

    route: {
        frozen: 'rgba(255, 255, 255, 0.3)',
        active: 'rgba(255, 255, 255, 0.7)'
    }
};

// =============================================================================
// Drawing Functions
// =============================================================================

/**
 * Clear the canvas.
 */
function clearCanvas() {
    offscreenCtx.fillStyle = COLORS.background;
    offscreenCtx.fillRect(0, 0, offscreenCanvas.width, offscreenCanvas.height);
}

/**
 * Draw grid lines.
 */
function drawGrid() {
    const gridSize = 50;
    offscreenCtx.strokeStyle = COLORS.grid;
    offscreenCtx.lineWidth = 0.5;

    for (let x = 0; x <= offscreenCanvas.width; x += gridSize) {
        offscreenCtx.beginPath();
        offscreenCtx.moveTo(x, 0);
        offscreenCtx.lineTo(x, offscreenCanvas.height);
        offscreenCtx.stroke();
    }

    for (let y = 0; y <= offscreenCanvas.height; y += gridSize) {
        offscreenCtx.beginPath();
        offscreenCtx.moveTo(0, y);
        offscreenCtx.lineTo(offscreenCanvas.width, y);
        offscreenCtx.stroke();
    }
}

/**
 * Draw airports on the canvas.
 * @param {Array} airports - Array of airport objects
 */
function drawAirports(airports) {
    airports.forEach(airport => {
        const x = airport.x;
        const y = airport.y;
        const size = 15;

        // Draw runway symbol
        offscreenCtx.fillStyle = COLORS.airport;
        offscreenCtx.strokeStyle = COLORS.airportStroke;
        offscreenCtx.lineWidth = 2;

        offscreenCtx.beginPath();
        offscreenCtx.rect(x - size, y - size/4, size * 2, size/2);
        offscreenCtx.fill();
        offscreenCtx.stroke();

        // Draw label
        offscreenCtx.fillStyle = '#ffffff';
        offscreenCtx.font = '12px monospace';
        offscreenCtx.textAlign = 'center';
        offscreenCtx.fillText(airport.id || airport.label, x, y - size - 5);
    });
}

/**
 * Draw targets on the canvas.
 * @param {Array} targets - Array of target objects
 * @param {Set} visitedTargets - Set of visited target IDs
 */
function drawTargets(targets, visitedTargets = new Set()) {
    targets.forEach(target => {
        const x = target.x;
        const y = target.y;
        const type = (target.type || 'a').toLowerCase();
        const priority = target.priority || 1;
        const size = 8 + priority * 2;

        // Determine color based on type and visited status
        const baseColor = COLORS.target[type] || COLORS.target.default;
        const isVisited = visitedTargets.has(target.id);

        offscreenCtx.globalAlpha = isVisited ? 0.4 : 1.0;

        // Draw target circle
        offscreenCtx.beginPath();
        offscreenCtx.arc(x, y, size, 0, Math.PI * 2);
        offscreenCtx.fillStyle = baseColor;
        offscreenCtx.fill();

        // Draw border
        offscreenCtx.strokeStyle = '#ffffff';
        offscreenCtx.lineWidth = 2;
        offscreenCtx.stroke();

        // Draw visited checkmark
        if (isVisited) {
            offscreenCtx.strokeStyle = '#00ff00';
            offscreenCtx.lineWidth = 2;
            offscreenCtx.beginPath();
            offscreenCtx.moveTo(x - 4, y);
            offscreenCtx.lineTo(x - 1, y + 3);
            offscreenCtx.lineTo(x + 4, y - 3);
            offscreenCtx.stroke();
        }

        // Draw label
        offscreenCtx.fillStyle = '#ffffff';
        offscreenCtx.font = '10px monospace';
        offscreenCtx.textAlign = 'center';
        offscreenCtx.fillText(target.id || target.label, x, y - size - 5);

        offscreenCtx.globalAlpha = 1.0;
    });
}

/**
 * Draw SAM zones on the canvas.
 * @param {Array} sams - Array of SAM objects
 */
function drawSAMs(sams) {
    sams.forEach(sam => {
        const pos = sam.pos || sam.position;
        if (!pos || pos.length < 2) return;

        const x = pos[0];
        const y = pos[1];
        const range = sam.range || 50;

        // Draw SAM range circle
        offscreenCtx.beginPath();
        offscreenCtx.arc(x, y, range, 0, Math.PI * 2);
        offscreenCtx.fillStyle = COLORS.sam.fill;
        offscreenCtx.fill();
        offscreenCtx.strokeStyle = COLORS.sam.stroke;
        offscreenCtx.lineWidth = 2;
        offscreenCtx.stroke();

        // Draw SAM center
        offscreenCtx.beginPath();
        offscreenCtx.arc(x, y, 5, 0, Math.PI * 2);
        offscreenCtx.fillStyle = COLORS.sam.stroke;
        offscreenCtx.fill();
    });
}

/**
 * Draw route trajectories.
 * @param {Object} routes - Routes object keyed by drone ID
 * @param {boolean} isFrozen - Whether these are frozen (completed) routes
 */
function drawRoutes(routes, isFrozen = false) {
    Object.entries(routes).forEach(([droneId, routeData], index) => {
        const trajectory = routeData.trajectory || [];
        if (trajectory.length < 2) return;

        const color = COLORS.drone[index % COLORS.drone.length];

        offscreenCtx.beginPath();
        offscreenCtx.moveTo(trajectory[0][0], trajectory[0][1]);

        for (let i = 1; i < trajectory.length; i++) {
            offscreenCtx.lineTo(trajectory[i][0], trajectory[i][1]);
        }

        offscreenCtx.strokeStyle = isFrozen ? COLORS.route.frozen : color;
        offscreenCtx.lineWidth = isFrozen ? 2 : 3;
        offscreenCtx.globalAlpha = isFrozen ? 0.5 : 0.8;
        offscreenCtx.stroke();
        offscreenCtx.globalAlpha = 1.0;
    });
}

/**
 * Draw drones at their current positions.
 * @param {Object} dronePositions - Object mapping drone ID to {x, y}
 */
function drawDrones(dronePositions) {
    Object.entries(dronePositions).forEach(([droneId, pos], index) => {
        const x = pos.x;
        const y = pos.y;
        const color = COLORS.drone[index % COLORS.drone.length];

        // Draw drone triangle
        offscreenCtx.beginPath();
        offscreenCtx.moveTo(x, y - 12);
        offscreenCtx.lineTo(x - 8, y + 8);
        offscreenCtx.lineTo(x + 8, y + 8);
        offscreenCtx.closePath();

        offscreenCtx.fillStyle = color;
        offscreenCtx.fill();
        offscreenCtx.strokeStyle = '#ffffff';
        offscreenCtx.lineWidth = 2;
        offscreenCtx.stroke();
    });
}

// =============================================================================
// Main Render Function
// =============================================================================

/**
 * Render the full canvas.
 * @param {Object} options - Render options
 */
export function render(options = {}) {
    const env = state.getEnvironment();
    const solution = state.getSolution();
    const animation = state.getState('animation');
    const visitedTargets = new Set(options.visitedTargets || []);

    // Clear and draw background
    clearCanvas();
    drawGrid();

    // Draw environment elements
    if (env.sams) drawSAMs(env.sams);
    if (env.airports) drawAirports(env.airports);
    if (env.targets) drawTargets(env.targets, visitedTargets);

    // Draw routes
    if (solution.routes) {
        drawRoutes(solution.routes, options.isFrozen);
    }

    // Draw drones
    if (animation.dronePositions) {
        drawDrones(animation.dronePositions);
    }

    // Copy to visible canvas
    ctx.drawImage(offscreenCanvas, 0, 0);
}

/**
 * Request animation frame render.
 */
export function requestRender() {
    requestAnimationFrame(() => render());
}

export default {
    initRenderer,
    resizeCanvas,
    render,
    requestRender
};
