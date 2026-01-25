/**
 * ISR Planner Modules - Entry Point
 *
 * This file exports all modules for the modular frontend architecture.
 *
 * Usage:
 *   import { state, renderer, animation, missionControl, editor } from './modules/index.js';
 *
 * Or import individual modules:
 *   import state from './modules/state.js';
 */

export { default as state } from './state.js';
export { default as renderer } from './renderer.js';
export { default as animation } from './animation.js';
export { default as missionControl } from './mission_control.js';
export { default as editor } from './editor.js';
export { default as segmentedMission } from './segmented_mission.js';
export { default as segmentManagerUI } from './segment_manager_ui.js';
export { default as droneConfig } from './drone_config.js';
export { default as agentChat } from './agent_chat.js';

// Re-export commonly used functions for convenience
export {
    getState,
    setState,
    getEnvironment,
    setEnvironment,
    getSolution,
    setSolution,
    subscribe
} from './state.js';

export {
    initRenderer,
    render,
    requestRender
} from './renderer.js';

export {
    play,
    pause,
    stop,
    setSpeed,
    seek,
    getProgress
} from './animation.js';

export {
    solveWithAllocation,
    solve,
    insertMissed,
    swapCloser,
    crossingRemoval,
    getCoverageStats
} from './mission_control.js';

export {
    toggleEditMode,
    enterEditMode,
    exitEditMode,
    selectTool,
    addAirport,
    addTarget,
    addSAM,
    deleteSelected
} from './editor.js';

export {
    Segment,
    SegmentedMission,
    isSegmentedMission,
    createSegmentedMission
} from './segmented_mission.js';

export {
    getAllConfigs,
    getConfig,
    setConfig,
    isEnabled,
    getEnabledDroneIds,
    initFromEnvironment as initDroneConfigs,
    syncUiWithConfigs as syncDroneConfigsUi,
    attachEventListeners as attachDroneConfigListeners
} from './drone_config.js';

export {
    sendMessage as sendAgentMessage,
    handleSendMessage as handleAgentSend,
    clearHistory as clearAgentHistory,
    attachEventListeners as attachAgentChatListeners
} from './agent_chat.js';

export {
    getMission,
    setMission,
    clearMission,
    isActive as isSegmentedMissionActive,
    previousSegment,
    nextSegment,
    goToSegment,
    resetToStart as resetSegmentedMission,
    requestCut,
    updateUI as updateSegmentUI,
    renderTimeline as renderSegmentTimeline,
    attachEventListeners as attachSegmentManagerListeners,
    importFromJson as importSegmentedMission,
    exportToJson as exportSegmentedMission
} from './segment_manager_ui.js';
