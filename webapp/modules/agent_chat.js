/**
 * Agent Chat Module - Agentic mode UI and communication.
 *
 * This module handles:
 * - Chat UI (message display, input handling)
 * - Communication with /api/agents/chat-v4 endpoint
 * - Applying agent-generated routes to the mission
 * - Agent trace visualization
 *
 * The agent uses LangGraph v4 multi-agent system with:
 * - Strategist: Understands high-level intent
 * - Mission Planner: Decomposes constraints
 * - Allocator: Edits target assignments
 * - Route Optimizer: Runs solver tools
 * - Critic: Evaluates solutions
 * - Responder: Explains decisions
 */

import state from './state.js';
import renderer from './renderer.js';

// =============================================================================
// Constants
// =============================================================================

const API_ENDPOINT = '/api/agents/chat-v4';

// =============================================================================
// State
// =============================================================================

// Store agent trace runs for the trace tab
let agentTraceRuns = [];

// Callbacks for route application
let onRoutesAppliedCallback = null;
let onGetPermissionsCallback = null;

// =============================================================================
// Configuration
// =============================================================================

/**
 * Set callback for when routes are applied.
 * @param {Function} callback - Function(routes, trajectories, points, fuel)
 */
export function setOnRoutesApplied(callback) {
    onRoutesAppliedCallback = callback;
}

/**
 * Set callback for getting UI permissions.
 * @param {Function} callback - Function returning { canSolve, isAnimating, isEditing }
 */
export function setGetPermissions(callback) {
    onGetPermissionsCallback = callback;
}

// =============================================================================
// DOM Helpers
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
 * Escape HTML to prevent XSS.
 * @param {string} text - Text to escape
 * @returns {string} Escaped HTML
 */
function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

// =============================================================================
// Chat UI
// =============================================================================

/**
 * Create a Q&A block for the chat history.
 * @param {string} question - User's question
 * @returns {HTMLElement} Q&A block element
 */
function createQABlock(question) {
    const timestamp = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

    const block = document.createElement('div');
    block.className = 'qa-block';
    block.innerHTML = `
        <div class="qa-question">
            <span class="qa-label">YOU [${timestamp}]</span>
            <span class="qa-text">${escapeHtml(question)}</span>
        </div>
        <div class="qa-response-wrapper">
            <div class="qa-response-label">AGENT RESPONSE</div>
            <div class="qa-response thinking">
                Thinking...
            </div>
        </div>
    `;
    return block;
}

/**
 * Append a system message to chat history.
 * @param {string} content - Message content
 * @param {string} type - Message type
 * @returns {string|null} Message ID or null
 */
export function appendSystemMessage(content, type = 'info') {
    const chatHistory = $('agent-chat-history');
    if (!chatHistory) return null;

    const msgId = 'msg-' + Date.now() + '-' + Math.random().toString(36).substring(2, 7);
    const msgDiv = document.createElement('div');
    msgDiv.id = msgId;
    msgDiv.className = `agent-message system-message ${type}`;
    msgDiv.innerHTML = content;

    chatHistory.appendChild(msgDiv);
    chatHistory.scrollTop = chatHistory.scrollHeight;
    return msgId;
}

/**
 * Remove a message by ID.
 * @param {string} msgId - Message ID
 */
export function removeMessage(msgId) {
    if (!msgId) return;
    const el = document.getElementById(msgId);
    if (el) el.remove();
}

/**
 * Clear chat history.
 */
export function clearHistory() {
    const chatHistory = $('agent-chat-history');
    if (chatHistory) {
        chatHistory.innerHTML = '';
    }
}

/**
 * Update response area with agent reply.
 * @param {HTMLElement} qaBlock - The Q&A block
 * @param {Object} data - Response data
 */
function updateResponseArea(qaBlock, data) {
    const responseArea = qaBlock.querySelector('.qa-response');
    if (!responseArea) return;

    if (data.reply) {
        // Escape model reply text
        let safeReply = escapeHtml(String(data.reply || ''));
        safeReply = safeReply.replace(/\n/g, '<br>');

        // Build badges HTML
        let badgesHtml = '';

        if (data.intent) {
            badgesHtml += ` <span class="agent-intent-badge">Intent: ${escapeHtml(String(data.intent))}</span>`;
        }

        if (Array.isArray(data.actions) && data.actions.length > 0) {
            const actionNames = data.actions.map(a => a?.type).filter(Boolean).join(', ');
            if (actionNames) {
                badgesHtml += ` <span class="agent-actions-badge">Actions: ${escapeHtml(actionNames)}</span>`;
            }
        }

        if (data.routes && Object.keys(data.routes).length > 0) {
            for (const [did, route] of Object.entries(data.routes)) {
                if (Array.isArray(route) && route.length > 0) {
                    badgesHtml += ` <span class="agent-route-badge">D${escapeHtml(String(did))}: ${route.map(x => escapeHtml(String(x))).join(' -> ')}</span>`;
                }
            }
        } else if (Array.isArray(data.route) && data.route.length > 0) {
            badgesHtml += ` <span class="agent-route-badge">Route: ${data.route.map(x => escapeHtml(String(x))).join(' -> ')}</span>`;
        }

        responseArea.innerHTML = safeReply + (badgesHtml ? '<br><br>' + badgesHtml : '');
        responseArea.classList.remove('thinking');
    } else {
        responseArea.innerHTML = 'No response from agent.';
        responseArea.classList.add('error');
    }
}

// =============================================================================
// API Communication
// =============================================================================

/**
 * Send message to agent and process response.
 * @param {string} message - User message
 * @param {Object} options - Additional options
 * @returns {Promise<Object>} Agent response
 */
export async function sendMessage(message, options = {}) {
    const env = state.getEnvironment();
    const droneConfigs = state.getDroneConfigs();
    const solution = state.getSolution();

    // Build existing solution context for agent memory
    const existingSolution = {};
    const sequences = solution.sequences || {};

    if (Object.keys(sequences).length > 0) {
        existingSolution.routes = {};
        existingSolution.allocation = {};

        for (const [did, seq] of Object.entries(sequences)) {
            if (seq) {
                const routeArr = seq.split(',').map(s => s.trim()).filter(s => s);
                existingSolution.routes[did] = routeArr;
                existingSolution.allocation[did] = routeArr.filter(wp => wp.startsWith('T'));
            }
        }
    }

    const requestBody = {
        message,
        env,
        drone_configs: droneConfigs,
        mission_id: state.getState('mission.id') || null,
        existing_solution: Object.keys(existingSolution).length > 0 ? existingSolution : null,
        ...options
    };

    const response = await fetch(API_ENDPOINT, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestBody)
    });

    if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
    }

    return response.json();
}

/**
 * Main send message handler with UI updates.
 */
export async function handleSendMessage() {
    const inputEl = $('agent-input');
    const chatHistory = $('agent-chat-history');
    const btnSend = $('btn-send-agent');

    if (!inputEl || !chatHistory) return;

    const message = inputEl.value.trim();
    if (!message) return;

    // Check environment
    const env = state.getEnvironment();
    if (!env || Object.keys(env).length === 0) {
        appendSystemMessage('No environment loaded. Import a JSON first.', 'warning');
        return;
    }

    if (!env.targets || env.targets.length === 0) {
        appendSystemMessage('Environment has no targets. Add/import targets before Agent Solve.', 'warning');
        return;
    }

    // Check permissions
    if (onGetPermissionsCallback) {
        const perms = onGetPermissionsCallback();
        if (perms?.isAnimating) {
            appendSystemMessage('Agent Solve disabled during animation. Pause or stop first.', 'warning');
            return;
        }
        if (perms?.isEditing) {
            appendSystemMessage('Agent Solve disabled during Edit mode. Accept or cancel edits first.', 'warning');
            return;
        }
        if (!perms?.canSolve) {
            appendSystemMessage('Agent Solve disabled in current mission state.', 'warning');
            return;
        }
    }

    // Create Q&A block
    const qaBlock = createQABlock(message);
    chatHistory.appendChild(qaBlock);
    inputEl.value = '';
    chatHistory.scrollTop = chatHistory.scrollHeight;

    // Disable button while processing
    if (btnSend) btnSend.disabled = true;

    try {
        const data = await sendMessage(message);

        // Store mission ID
        if (data.mission_id) {
            state.setState('mission.id', data.mission_id);
        }

        // Store trace events
        if (data.trace_events && Array.isArray(data.trace_events)) {
            agentTraceRuns.push({
                ts: Date.now(),
                user_message: message,
                mission_id: data.mission_id || null,
                trace_events: data.trace_events,
                trace: data.trace || null
            });
            renderTraceTab();
        }

        // Update UI with response
        updateResponseArea(qaBlock, data);

        // Apply routes if provided
        if (data.routes && Object.keys(data.routes).length > 0 && onRoutesAppliedCallback) {
            onRoutesAppliedCallback(data.routes, data.trajectories, data.points, data.fuel);
        }

        return data;
    } catch (err) {
        const responseArea = qaBlock.querySelector('.qa-response');
        if (responseArea) {
            responseArea.innerHTML = `Error: ${escapeHtml(err.message)}`;
            responseArea.classList.add('error');
        }
        throw err;
    } finally {
        if (btnSend) btnSend.disabled = false;
        chatHistory.scrollTop = chatHistory.scrollHeight;
    }
}

// =============================================================================
// Trace Tab
// =============================================================================

/**
 * Get all trace runs.
 * @returns {Array}
 */
export function getTraceRuns() {
    return [...agentTraceRuns];
}

/**
 * Clear trace runs.
 */
export function clearTraceRuns() {
    agentTraceRuns = [];
    renderTraceTab();
}

/**
 * Render the trace tab content.
 */
export function renderTraceTab() {
    const container = $('agents-trace-content');
    if (!container) return;

    if (agentTraceRuns.length === 0) {
        container.innerHTML = '<div class="trace-empty">No agent runs yet. Send a message in the Agents tab.</div>';
        return;
    }

    let html = '';

    // Show runs in reverse order (newest first)
    for (let i = agentTraceRuns.length - 1; i >= 0; i--) {
        const run = agentTraceRuns[i];
        const time = new Date(run.ts).toLocaleTimeString();

        html += '<div class="trace-run">';
        html += '<div class="trace-run-header">';
        html += `<span class="trace-time">${escapeHtml(time)}</span>`;
        html += `<span class="trace-message">${escapeHtml(run.user_message.substring(0, 50))}${run.user_message.length > 50 ? '...' : ''}</span>`;
        if (run.mission_id) {
            html += `<span class="trace-mission-id">ID: ${escapeHtml(run.mission_id.substring(0, 8))}...</span>`;
        }
        html += '</div>';

        html += '<div class="trace-events">';
        for (const evt of run.trace_events) {
            const levelClass = evt.t === 'error' ? 'trace-error' : evt.t.includes('warn') ? 'trace-warn' : 'trace-info';
            html += `<div class="trace-event ${levelClass}">`;
            html += `<span class="trace-type">${escapeHtml(evt.t)}</span>`;
            html += `<span class="trace-msg">${escapeHtml(evt.msg)}</span>`;
            if (evt.data) {
                html += `<span class="trace-data">${escapeHtml(JSON.stringify(evt.data).substring(0, 100))}</span>`;
            }
            html += '</div>';
        }
        html += '</div>';
        html += '</div>';
    }

    container.innerHTML = html;
}

// =============================================================================
// Event Listeners
// =============================================================================

/**
 * Attach event listeners to agent chat UI elements.
 */
export function attachEventListeners() {
    const btnSend = $('btn-send-agent');
    const inputEl = $('agent-input');

    if (btnSend) {
        btnSend.addEventListener('click', () => handleSendMessage());
    }

    if (inputEl) {
        inputEl.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                handleSendMessage();
            }
        });
    }

    // Clear button if exists
    const btnClear = $('btn-clear-agent');
    if (btnClear) {
        btnClear.addEventListener('click', () => clearHistory());
    }
}

/**
 * Update agent UI state based on permissions.
 * @param {Object} perms - Permission object
 */
export function updateUiState(perms) {
    const btnSend = $('btn-send-agent');
    const inputEl = $('agent-input');

    if (btnSend) {
        btnSend.disabled = perms?.isAnimating || perms?.isEditing || !perms?.canSolve;
    }

    if (inputEl) {
        inputEl.disabled = perms?.isAnimating || perms?.isEditing || !perms?.canSolve;
    }
}

// =============================================================================
// Module Exports
// =============================================================================

export default {
    // Configuration
    setOnRoutesApplied,
    setGetPermissions,

    // Chat UI
    appendSystemMessage,
    removeMessage,
    clearHistory,
    handleSendMessage,

    // API
    sendMessage,

    // Trace
    getTraceRuns,
    clearTraceRuns,
    renderTraceTab,

    // Event handling
    attachEventListeners,
    updateUiState
};
