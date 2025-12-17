// isr.js  — single-file version with:
// - Edit mode ON/OFF
// - Canvas drawing + editing (airports/targets/SAMs)
// - Drone config + sequences
// - Stats
// - Import/Export
// - Planner integration

"use strict";

console.log("isr.js loaded", new Date().toISOString());

// ----------------------------------------------------
// DOM helper
// ----------------------------------------------------
function $(id) {
  return document.getElementById(id);
}

// Add this:
function setText(id, text) {
  const el = $(id);
  if (!el) return;
  el.textContent = text == null ? "" : String(text);
}

// ----------------------------------------------------
// Global state
// ----------------------------------------------------
const state = {
  env: null,                 // environment JSON (airports, targets, sams)
  envFilename: null,         // name of JSON file

  sequences: {},             // { "1": "A1,T1,A1", ... }
  routes: {},                // { "1": {route, points, distance, fuel_budget}, ... }
  wrappedPolygons: [],       // wrapped SAM polygons from solver
  currentDroneForSeq: "1",

  droneConfigs: {},          // { "1": {enabled, fuel_budget, start_airport, end_airport, target_access}, ... }

  missionId: null,           // current v4 mission_id from the backend

  solveAbortController: null, // AbortController for canceling solver requests

  // Editing-related
  editMode: true,            // Edit: ON/OFF
  addMode: null,             // "airport" | "target" | "sam" | null
  selectedObject: null,      // { kind: "airport"|"target"|"sam", index: number } or null
  dragging: null,            // { kind, index, offsetX, offsetY } during drag

  // SAM wrapping debounce
  samWrappingTimeout: null,  // timeout ID for debounced wrapping updates
  wrappingAbortController: null,  // AbortController for canceling pending wrapping requests

  // Trajectory visibility
  trajectoryVisible: {
    "1": true,
    "2": true,
    "3": true,
    "4": true,
    "5": true,
  },

  // Animation state
  animation: {
    active: false,
    drones: {},              // { "1": { progress: 0, animating: false }, ... }
    animationId: null,       // requestAnimationFrame ID
  },

  // Checkpoint / segmented solve state
  checkpoint: {
    active: false,
    pct: 0.5,
    segments: {},   // per drone: { prefix, suffix, splitPoint, checkpointDist }
  },

  // Mission history for segmented replanning
  missionHistory: [],          // Array of segment snapshots
  initialEnvSnapshot: null,    // Immutable copy of original env for reset
  visitedTargets: [],          // Target IDs completed across all segments

  // Checkpoint replan prefix distances (used for animation start position after replan)
  checkpointReplanPrefixDistances: null,
};

// ----------------------------------------------------
// Mission Mode State Machine
// ----------------------------------------------------
/**
 * MissionMode - Single source of truth for UI state
 *
 * IDLE:                 No committed plan exists yet. Can edit env or solve.
 * EDITING_ENV:          User is editing draftEnv. Must Accept Edits to proceed.
 * DRAFT_READY:          A draft solution exists. Must Accept Solution or Discard.
 * READY_TO_ANIMATE:     Committed plan exists. Can start/resume animation.
 * ANIMATING:            Drones are moving. Can Pause or Cut (C).
 * PAUSED_MID_ANIMATION: Paused but resumable (no truncation). Can resume or edit.
 * CHECKPOINT:           Cut occurred. Past locked, future removed. Must replan.
 */
const MissionMode = {
  IDLE: "IDLE",
  EDITING_ENV: "EDITING_ENV",
  DRAFT_READY: "DRAFT_READY",
  READY_TO_ANIMATE: "READY_TO_ANIMATE",
  ANIMATING: "ANIMATING",
  PAUSED_MID_ANIMATION: "PAUSED_MID_ANIMATION",
  CHECKPOINT: "CHECKPOINT",
};

// Mission state machine state
const missionState = {
  mode: MissionMode.IDLE,

  // Environment state (two-phase commit)
  draftEnv: null,              // Editable environment shown in editor
  acceptedEnv: null,           // Committed environment used for solving

  // Solution state (two-phase commit)
  draftSolution: null,         // Last solver output (not yet accepted)
  committedSegments: [],       // Array of accepted solutions (mission history)
  currentSegmentIndex: -1,     // Index of current segment being animated (-1 = none)

  // Runtime state for animation resume
  pauseContext: null,          // { droneStates: {...} } - saved state for resume
};

/**
 * Get UI permissions based on current mission mode
 * Returns which actions are allowed in the current state
 */
function getUiPermissions() {
  const mode = missionState.mode;
  const hasCommittedPlan = missionState.committedSegments.length > 0;
  const hasDraftSolution = missionState.draftSolution !== null;

  return {
    // Animation controls
    canAnimate: mode === MissionMode.READY_TO_ANIMATE,
    canResume: mode === MissionMode.PAUSED_MID_ANIMATION,
    canPause: mode === MissionMode.ANIMATING,
    canCut: mode === MissionMode.ANIMATING,

    // Editing controls
    canEnterEdit: mode === MissionMode.IDLE ||
                  mode === MissionMode.CHECKPOINT ||
                  mode === MissionMode.PAUSED_MID_ANIMATION,
    canAcceptEdits: mode === MissionMode.EDITING_ENV,

    // Planning controls
    canSolve: mode === MissionMode.IDLE || mode === MissionMode.CHECKPOINT,
    canOptimize: mode === MissionMode.DRAFT_READY,
    canAcceptSolution: mode === MissionMode.DRAFT_READY,
    canDiscardDraft: mode === MissionMode.DRAFT_READY,

    // Reset
    canReset: mode !== MissionMode.ANIMATING &&
              mode !== MissionMode.EDITING_ENV &&
              hasCommittedPlan,

    // Derived states for UI
    isAnimating: mode === MissionMode.ANIMATING,
    isEditing: mode === MissionMode.EDITING_ENV,
    hasDraftSolution: hasDraftSolution,
    hasCommittedPlan: hasCommittedPlan,
  };
}

/**
 * Set mission mode and update UI accordingly
 */
function setMissionMode(newMode, reason = "") {
  const oldMode = missionState.mode;
  if (oldMode === newMode) return;

  missionState.mode = newMode;
  console.log(`[MissionMode] ${oldMode} → ${newMode}${reason ? ` (${reason})` : ""}`);
  appendDebugLine(`Mode: ${newMode}${reason ? ` - ${reason}` : ""}`);

  // Update UI elements based on new mode
  updateStatusBanner();
  updateButtonStates();
}

/**
 * Get status banner text for current mode
 */
function getStatusBannerText() {
  const mode = missionState.mode;

  switch (mode) {
    case MissionMode.IDLE:
      return "Ready. Load environment, edit, or Run Planner.";
    case MissionMode.EDITING_ENV:
      return "Editing environment. Click Accept Edits when done.";
    case MissionMode.DRAFT_READY:
      return "Draft solution ready. Optimize or Accept Solution to proceed.";
    case MissionMode.READY_TO_ANIMATE:
      return "Solution accepted. Click Animate to start mission.";
    case MissionMode.ANIMATING:
      return "Animating. Press Pause to stop, or C to cut/checkpoint.";
    case MissionMode.PAUSED_MID_ANIMATION:
      return "Paused. Click Animate to resume, or Edit to modify environment.";
    case MissionMode.CHECKPOINT:
      return "Checkpoint. Edit environment and Run Planner to replan remainder.";
    default:
      return `Mode: ${mode}`;
  }
}

/**
 * Update the status banner display
 */
function updateStatusBanner() {
  const banner = $("status-banner");
  const bannerText = $("status-banner-text");
  if (!banner) return;

  const text = getStatusBannerText();
  const mode = missionState.mode;
  const perms = getUiPermissions();

  // Update text
  if (bannerText) {
    bannerText.textContent = text;
  }

  // Set banner color based on mode
  banner.className = "status-banner";
  switch (mode) {
    case MissionMode.ANIMATING:
      banner.classList.add("status-animating");
      break;
    case MissionMode.EDITING_ENV:
      banner.classList.add("status-editing");
      break;
    case MissionMode.DRAFT_READY:
      banner.classList.add("status-draft-ready");
      break;
    case MissionMode.READY_TO_ANIMATE:
      banner.classList.add("status-ready-to-animate");
      break;
    case MissionMode.CHECKPOINT:
      banner.classList.add("status-checkpoint");
      break;
    case MissionMode.PAUSED_MID_ANIMATION:
      banner.classList.add("status-paused");
      break;
    default:
      banner.classList.add("status-idle");
  }

  // Show/hide action buttons based on permissions
  const acceptEditsBtn = $("btn-accept-edits");
  const acceptSolutionBtn = $("btn-accept-solution");
  const discardBtn = $("btn-discard-draft");

  if (acceptEditsBtn) {
    acceptEditsBtn.style.display = perms.canAcceptEdits ? "inline-block" : "none";
  }
  if (acceptSolutionBtn) {
    acceptSolutionBtn.style.display = perms.canAcceptSolution ? "inline-block" : "none";
  }
  if (discardBtn) {
    discardBtn.style.display = perms.canDiscardDraft ? "inline-block" : "none";
  }
}

/**
 * Update button enabled/disabled states based on current mode
 */
function updateButtonStates() {
  const perms = getUiPermissions();

  // Animation buttons
  const animateBtn = $("anim-all");
  if (animateBtn) {
    animateBtn.disabled = !(perms.canAnimate || perms.canResume);
    animateBtn.textContent = perms.canResume ? "Resume" : "Animate";
  }

  // Pause button (if exists)
  const pauseBtn = $("btn-pause");
  if (pauseBtn) {
    pauseBtn.disabled = !perms.canPause;
  }

  // Edit button
  const editBtn = $("btn-edit-mode");
  if (editBtn) {
    editBtn.disabled = !perms.canEnterEdit && !perms.isEditing;
  }

  // Accept Edits button
  const acceptEditsBtn = $("btn-accept-edits");
  if (acceptEditsBtn) {
    acceptEditsBtn.disabled = !perms.canAcceptEdits;
    acceptEditsBtn.style.display = perms.isEditing ? "inline-block" : "none";
  }

  // Run Planner / Solve button
  const solveBtn = $("btn-run-planner");
  if (solveBtn) {
    solveBtn.disabled = !perms.canSolve;
  }

  // Accept Solution button
  const acceptSolutionBtn = $("btn-accept-solution");
  if (acceptSolutionBtn) {
    acceptSolutionBtn.disabled = !perms.canAcceptSolution;
    acceptSolutionBtn.style.display = perms.hasDraftSolution ? "inline-block" : "none";
  }

  // Discard Draft button
  const discardBtn = $("btn-discard-draft");
  if (discardBtn) {
    discardBtn.disabled = !perms.canDiscardDraft;
    discardBtn.style.display = perms.hasDraftSolution ? "inline-block" : "none";
  }
}

// ----------------------------------------------------
// Mission Mode Action Handlers
// ----------------------------------------------------

/**
 * Enter editing mode - start tracking changes to environment
 */
function enterEditMode() {
  const perms = getUiPermissions();
  if (!perms.canEnterEdit) {
    appendDebugLine("Cannot enter edit mode in current state");
    return;
  }

  // Save a copy of current env as draft
  if (state.env) {
    missionState.draftEnv = JSON.parse(JSON.stringify(state.env));
  }

  setMissionMode(MissionMode.EDITING_ENV, "entered edit mode");
  state.editMode = true;

  const btn = $("btn-toggle-edit");
  if (btn) {
    btn.textContent = "Editing...";
    btn.classList.remove("off");
  }
}

/**
 * Accept edits - commit the draft environment
 */
function acceptEdits() {
  const perms = getUiPermissions();
  if (!perms.canAcceptEdits) {
    appendDebugLine("Cannot accept edits in current state");
    return;
  }

  // Commit the draft environment
  missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));
  missionState.draftEnv = null;

  // Determine next state
  const hasCommittedPlan = missionState.committedSegments.length > 0;

  if (hasCommittedPlan) {
    // If we have a committed plan, we're in checkpoint mode - need to replan
    setMissionMode(MissionMode.CHECKPOINT, "edits accepted, ready to replan");
  } else {
    // No committed plan yet - go back to IDLE
    setMissionMode(MissionMode.IDLE, "edits accepted");
  }

  state.editMode = false;
  const btn = $("btn-toggle-edit");
  if (btn) {
    btn.textContent = "Edit";
    btn.classList.add("off");
  }

  appendDebugLine("✅ Edits accepted");
  drawEnvironment();
}

/**
 * Cancel editing - discard draft changes
 */
function cancelEdits() {
  if (missionState.mode !== MissionMode.EDITING_ENV) return;

  // Restore from draft or accepted env
  if (missionState.draftEnv) {
    state.env = JSON.parse(JSON.stringify(missionState.draftEnv));
  } else if (missionState.acceptedEnv) {
    state.env = JSON.parse(JSON.stringify(missionState.acceptedEnv));
  }
  missionState.draftEnv = null;

  // Determine which state to return to
  const hasCommittedPlan = missionState.committedSegments.length > 0;
  if (hasCommittedPlan) {
    setMissionMode(MissionMode.CHECKPOINT, "edits cancelled");
  } else {
    setMissionMode(MissionMode.IDLE, "edits cancelled");
  }

  state.editMode = false;
  const btn = $("btn-toggle-edit");
  if (btn) {
    btn.textContent = "Edit";
    btn.classList.add("off");
  }

  appendDebugLine("❌ Edits cancelled, reverted to previous state");
  drawEnvironment();
}

/**
 * Accept solution - commit the draft solution as the next segment
 */
function acceptSolution() {
  const perms = getUiPermissions();
  if (!perms.canAcceptSolution) {
    appendDebugLine("Cannot accept solution in current state");
    return;
  }

  if (!missionState.draftSolution) {
    appendDebugLine("No draft solution to accept");
    return;
  }

  // Add the draft solution to committed segments
  const segment = {
    index: missionState.committedSegments.length,
    solution: JSON.parse(JSON.stringify(missionState.draftSolution)),
    env: JSON.parse(JSON.stringify(missionState.acceptedEnv || state.env)),
    timestamp: new Date().toISOString(),
    // Store prefix distances for mission replay (where the C point is)
    prefixDistances: state.checkpointReplanPrefixDistances
      ? JSON.parse(JSON.stringify(state.checkpointReplanPrefixDistances))
      : null,
    isCheckpointReplan: missionState.draftSolution.isCheckpointReplan || false,
  };
  missionState.committedSegments.push(segment);
  missionState.currentSegmentIndex = segment.index;

  // Clear the draft
  missionState.draftSolution = null;

  // Transition to READY_TO_ANIMATE
  setMissionMode(MissionMode.READY_TO_ANIMATE, "solution accepted");

  appendDebugLine(`✅ Solution accepted as segment ${segment.index + 1}`);
  appendDebugLine(`   Visited targets retained: [${state.visitedTargets.join(", ")}]`);
  drawEnvironment();
}

/**
 * Discard draft solution - go back to previous state
 */
function discardDraftSolution() {
  const perms = getUiPermissions();
  if (!perms.canDiscardDraft) {
    appendDebugLine("Cannot discard draft in current state");
    return;
  }

  // Clear the draft solution
  missionState.draftSolution = null;

  // Clear visual routes
  state.routes = {};

  // Determine which state to return to
  const hasCommittedPlan = missionState.committedSegments.length > 0;
  if (hasCommittedPlan) {
    // Restore previous segment's routes for display
    const lastSegment = missionState.committedSegments[missionState.committedSegments.length - 1];
    if (lastSegment && lastSegment.solution && lastSegment.solution.drone_routes) {
      lastSegment.solution.drone_routes.forEach(r => {
        state.routes[String(r.drone_id)] = {
          route: r.route,
          trajectory: r.trajectory,
          distance: r.distance,
          fuel_budget: r.fuel_budget,
          points: r.points || 0,
        };
      });
    }
    setMissionMode(MissionMode.CHECKPOINT, "draft discarded, back to checkpoint");
  } else {
    setMissionMode(MissionMode.IDLE, "draft discarded");
  }

  appendDebugLine("❌ Draft solution discarded");
  drawEnvironment();
}

/**
 * Reset mission - return to initial state with first solution visible
 * Goes to READY_TO_ANIMATE so user can replay from the beginning
 */
function resetMission() {
  const perms = getUiPermissions();
  if (!perms.canReset) {
    appendDebugLine("Cannot reset in current state");
    return;
  }

  // Must have at least one committed segment to reset to
  if (missionState.committedSegments.length === 0) {
    appendDebugLine("No committed segments to reset to");
    return;
  }

  // Stop any animation first (without triggering state change)
  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }
  state.animation.active = false;
  state.animation.drones = {};

  // Restore initial environment if we have it
  if (state.initialEnvSnapshot) {
    state.env = JSON.parse(JSON.stringify(state.initialEnvSnapshot));
    missionState.acceptedEnv = JSON.parse(JSON.stringify(state.initialEnvSnapshot));
  }

  // KEEP all committed segments for mission replay (don't discard!)
  const firstSegment = missionState.committedSegments[0];
  const totalSegments = missionState.committedSegments.length;
  appendDebugLine(`Reset: Found ${totalSegments} committed segments (keeping all for replay)`);
  appendDebugLine(`Reset: firstSegment exists: ${!!firstSegment}, has solution: ${!!(firstSegment && firstSegment.solution)}`);
  if (firstSegment && firstSegment.solution) {
    appendDebugLine(`Reset: firstSegment.solution.routes keys: ${Object.keys(firstSegment.solution.routes || {}).join(", ") || "EMPTY"}`);
  }
  // Log all segments for debugging
  missionState.committedSegments.forEach((seg, idx) => {
    const hasPrefixDist = seg.prefixDistances ? Object.keys(seg.prefixDistances).length : 0;
    appendDebugLine(`  Segment ${idx}: isCheckpointReplan=${seg.isCheckpointReplan}, prefixDistances=${hasPrefixDist} drones`);
  });
  // Don't discard segments - keep them all: missionState.committedSegments = [firstSegment];
  missionState.currentSegmentIndex = 0;  // Start from first segment

  // Clear draft state
  missionState.draftEnv = null;
  missionState.draftSolution = null;
  missionState.pauseContext = null;

  // Restore the first segment's solution to the UI
  if (firstSegment && firstSegment.solution) {
    appendDebugLine("Restoring segment solution routes: " + Object.keys(firstSegment.solution.routes || {}).join(", "));
    // Debug: Log trajectory lengths from the stored segment
    Object.entries(firstSegment.solution.routes || {}).forEach(([did, routeData]) => {
      const trajLen = routeData.trajectory ? routeData.trajectory.length : 0;
      const firstPt = trajLen > 0 ? `(${routeData.trajectory[0][0].toFixed(1)},${routeData.trajectory[0][1].toFixed(1)})` : "N/A";
      const lastPt = trajLen > 0 ? `(${routeData.trajectory[trajLen-1][0].toFixed(1)},${routeData.trajectory[trajLen-1][1].toFixed(1)})` : "N/A";
      appendDebugLine(`  D${did}: ${trajLen} pts, route: ${(routeData.route || []).join("-")}, from ${firstPt} to ${lastPt}`);
    });
    applyDraftSolutionToUI(firstSegment.solution);
    appendDebugLine("After restore, state.routes keys: " + Object.keys(state.routes).join(", "));
    // Debug: Verify routes were applied
    Object.entries(state.routes || {}).forEach(([did, routeData]) => {
      const trajLen = routeData.trajectory ? routeData.trajectory.length : 0;
      const firstPt = trajLen > 0 ? `(${routeData.trajectory[0][0].toFixed(1)},${routeData.trajectory[0][1].toFixed(1)})` : "N/A";
      const lastPt = trajLen > 0 ? `(${routeData.trajectory[trajLen-1][0].toFixed(1)},${routeData.trajectory[trajLen-1][1].toFixed(1)})` : "N/A";
      appendDebugLine(`  state.routes[${did}]: ${trajLen} pts, from ${firstPt} to ${lastPt}`);
    });
  }

  // Clear any _fullTrajectory overrides that may have been set during checkpoint
  Object.values(state.routes).forEach(routeData => {
    if (routeData._fullTrajectory) {
      delete routeData._fullTrajectory;
    }
  });

  // Clear checkpoint state
  state.checkpoint = { active: false, pct: 0.5, segments: {} };
  state.checkpointReplanPrefixDistances = null;
  state.missionHistory = [];
  state.visitedTargets = [];

  // Clear segment switch flags for replay
  state._segmentSwitchPending = false;
  state._nextSegmentIndex = null;

  // Reset to READY_TO_ANIMATE so user can replay from beginning
  setMissionMode(MissionMode.READY_TO_ANIMATE, "mission reset to start");

  appendDebugLine("🔄 Mission reset - ready to replay from beginning");
  updateAirportDropdowns();
  updateAnimationButtonStates();
  drawEnvironment();
}

// ----------------------------------------------------
// Utility helpers
// ----------------------------------------------------
function appendDebugLine(msg) {
  const pre = $("debug-output");
  if (!pre) return;
  const now = new Date().toLocaleTimeString();
  pre.textContent = `[${now}] ${msg}\n` + pre.textContent;
}

function invalidateMission(reason) {
  // Drop current mission so backend will treat next v4 call as a new mission
  state.missionId = null;
  state.routes = {};
  state.trajectoryVisible = {};
  if (reason) {
    appendDebugLine("Mission invalidated: " + reason);
  }
}

// ----------------------------------------------------
// Client-side SAM wrapping (no server round-trip!)
// ----------------------------------------------------
function _sampleCircle(cx, cy, r, minSeg = 2.0) {

  if (r <= 0) return [[cx, cy]];
  let dtheta = minSeg / Math.max(r, 0.001);
  dtheta = Math.max(dtheta, Math.PI / 36); // 5 degrees
  dtheta = Math.min(dtheta, Math.PI / 6);  // 30 degrees
  const nSteps = Math.max(8, Math.ceil(2 * Math.PI / dtheta));
  const thetaStep = 2 * Math.PI / nSteps;
  const pts = [];
  for (let i = 0; i < nSteps; i++) {
    const theta = i * thetaStep;
    pts.push([cx + r * Math.cos(theta), cy + r * Math.sin(theta)]);
  }
  return pts;
}

function _cross(o, a, b) {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

function _convexHull(points) {
  const pts = [...new Set(points.map(p => JSON.stringify(p)))].map(s => JSON.parse(s));
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (pts.length <= 1) return pts;

  const lower = [];
  for (const p of pts) {
    while (lower.length >= 2 && _cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
      lower.pop();
    }
    lower.push(p);
  }

  const upper = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && _cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
      upper.pop();
    }
    upper.push(p);
  }

  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

function _distance(a, b) {
  return Math.hypot(b[0] - a[0], b[1] - a[1]);
}

function _samsOverlap(sam1, sam2) {
  const [x1, y1] = sam1.pos || [sam1.x || 0, sam1.y || 0];
  const [x2, y2] = sam2.pos || [sam2.x || 0, sam2.y || 0];
  const r1 = sam1.range || sam1.radius || 0;
  const r2 = sam2.range || sam2.radius || 0;
  return _distance([x1, y1], [x2, y2]) <= (r1 + r2);
}

// ----------------------------------------------------
// Checkpoint freeze key handler (C) - REMOVED, handled in main keydown listener
// ----------------------------------------------------
// Old duplicate handler removed - C key is now handled in the main window.addEventListener("keydown")
// which properly uses cutAtCheckpoint() with state machine permissions

// Reset mission key handler (R)
// ----------------------------------------------------
if (!window.__resetMissionKeyInstalled) {
  window.__resetMissionKeyInstalled = true;

  document.addEventListener(
    "keydown",
    (e) => {
      if (String(e.key).toLowerCase() === "r") {
        // Skip if user is typing in an input field
        const activeTag = document.activeElement?.tagName?.toLowerCase();
        if (activeTag === "input" || activeTag === "textarea" || activeTag === "select") {
          console.log("R key ignored - focus is on input element:", activeTag);
          return;
        }

        console.log("R key pressed, mode =", missionState.mode);
        appendDebugLine("R key pressed - attempting reset...");

        // Use state machine permissions
        const perms = getUiPermissions();
        if (perms.canReset) {
          console.log("Calling resetMission()");
          resetMission();
        } else {
          appendDebugLine("Cannot reset in current state: " + missionState.mode);
        }
      }
    },
    true
  );
}


function _clusterOverlappingSams(sams) {
  if (!sams || sams.length === 0) return [];
  const n = sams.length;
  const parent = Array.from({ length: n }, (_, i) => i);

  function find(x) {
    if (parent[x] !== x) parent[x] = find(parent[x]);
    return parent[x];
  }

  function union(x, y) {
    const px = find(x), py = find(y);
    if (px !== py) parent[px] = py;
  }

  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      if (_samsOverlap(sams[i], sams[j])) {
        union(i, j);
      }
    }
  }

  const clustersMap = {};
  for (let i = 0; i < n; i++) {
    const root = find(i);
    if (!clustersMap[root]) clustersMap[root] = [];
    clustersMap[root].push(sams[i]);
  }

  return Object.values(clustersMap);
}

function computeWrappedPolygonsClientSide(sams) {
  if (!sams || sams.length === 0) return [];

  const clusters = _clusterOverlappingSams(sams);
  const polygons = [];

  for (const cluster of clusters) {
    const allPoints = [];
    for (const sam of cluster) {
      const [cx, cy] = sam.pos || [sam.x || 0, sam.y || 0];
      const r = sam.range || sam.radius || 0;
      allPoints.push(..._sampleCircle(cx, cy, r, 2.0));
    }
    if (allPoints.length === 0) continue;

    const hull = _convexHull(allPoints);
    if (hull.length >= 3) {
      polygons.push(hull);
    }
  }

  return polygons;
}

// Update SAM wrapping instantly (client-side) during drag
function updateSamWrappingClientSide() {
  if (state.env && state.env.sams && state.env.sams.length > 0) {
    state.wrappedPolygons = computeWrappedPolygonsClientSide(state.env.sams);
  } else {
    state.wrappedPolygons = [];
  }
}

// ----------------------------------------------------
// Polyline utilities
// points: array of [x,y]
// dist: distance along polyline in same units as coords
// ----------------------------------------------------
function split_polyline_at_distance(points, dist) {
  // Defensive defaults
  const n = Array.isArray(points) ? points.length : 0;

  if (n === 0) {
    return {
      prefixPoints: [],
      suffixPoints: [],
      splitPoint: null,
      totalDistance: 0,
      splitIndex: -1,
      t: 0,
    };
  }

  if (n === 1) {
    // Only one point: prefix/suffix are identical
    return {
      prefixPoints: [points[0]],
      suffixPoints: [points[0]],
      splitPoint: points[0],
      totalDistance: 0,
      splitIndex: 0,
      t: 0,
    };
  }

  // Build cumulative distances
  const cum = [0];
  let total = 0;

  for (let i = 1; i < n; i++) {
    const dx = points[i][0] - points[i - 1][0];
    const dy = points[i][1] - points[i - 1][1];
    total += Math.sqrt(dx * dx + dy * dy);
    cum.push(total);
  }

  // Clamp dist into [0, total]
  let d = dist;
  if (!Number.isFinite(d)) d = 0;
  d = Math.max(0, Math.min(d, total));

  // Edge: split at start
  if (d === 0) {
    const sp = points[0];
    return {
      prefixPoints: [sp],
      suffixPoints: [sp, ...points.slice(1)],
      splitPoint: sp,
      totalDistance: total,
      splitIndex: 0,
      t: 0,
    };
  }

  // Edge: split at end
  if (d === total) {
    const sp = points[n - 1];
    return {
      prefixPoints: [...points],
      suffixPoints: [sp],
      splitPoint: sp,
      totalDistance: total,
      splitIndex: n - 2, // last segment
      t: 1,
    };
  }

  // Find segment containing distance d:
  // cum[i] <= d < cum[i+1]
  let seg = 0;
  for (let i = 0; i < n - 1; i++) {
    if (cum[i] <= d && d < cum[i + 1]) {
      seg = i;
      break;
    }
  }

  const segStart = points[seg];
  const segEnd = points[seg + 1];
  const segLen = cum[seg + 1] - cum[seg];

  // Interpolation parameter along the segment
  const t = segLen > 0 ? (d - cum[seg]) / segLen : 0;

  const splitPoint = [
    segStart[0] + t * (segEnd[0] - segStart[0]),
    segStart[1] + t * (segEnd[1] - segStart[1]),
  ];

  // Construct prefix:
  // include points[0..seg] plus splitPoint (unless it equals points[seg] exactly)
  const prefixPoints = points.slice(0, seg + 1);
  const lastPrefix = prefixPoints[prefixPoints.length - 1];
  if (lastPrefix[0] !== splitPoint[0] || lastPrefix[1] !== splitPoint[1]) {
    prefixPoints.push(splitPoint);
  }

  // Construct suffix:
  // start with splitPoint, then include points[seg+1..end]
  const suffixPoints = [splitPoint, ...points.slice(seg + 1)];

  return {
    prefixPoints,
    suffixPoints,
    splitPoint,
    totalDistance: total,
    splitIndex: seg,
    t,
  };
}


// ----------------------------------------------------
// Canvas sizing
// ----------------------------------------------------
function resizeCanvasToContainer() {
  const canvas = $("env-canvas");
  if (!canvas) return;

  // Keep fixed 800x800 internal resolution for quality
  // The CSS will handle visual scaling
  canvas.width = 800;
  canvas.height = 800;

  drawEnvironment();
}

// ----------------------------------------------------
// Environment drawing
// ----------------------------------------------------
let _drawEnvFrameCount = 0;
function drawEnvironment() {
  const canvas = $("env-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  // Debug: Log visited targets every 30 frames (about once per second at 30fps)
  _drawEnvFrameCount++;
  if (_drawEnvFrameCount % 60 === 1 && state.visitedTargets.length > 0) {
    console.log(`[drawEnv] visitedTargets (${state.visitedTargets.length}):`, state.visitedTargets);
  }

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // If no environment yet, draw placeholder grid
  if (!state.env) {
    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    ctx.strokeStyle = "#1f2937";
    ctx.lineWidth = 0.5;
    for (let x = 0; x < canvas.width; x += 30) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, canvas.height);
      ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += 30) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width, y);
      ctx.stroke();
    }
    return;
  }

  const airports = state.env.airports || [];
  const targets = state.env.targets || [];
  const sams = state.env.sams || [];
  const gridSize = 100.0; // world coordinates 0..100

  // world -> canvas coordinates
  function w2c(x, y) {
    const pad = 20;
    const w = canvas.width - 2 * pad;
    const h = canvas.height - 2 * pad;
    const sx = pad + (x / gridSize) * w;
    const sy = canvas.height - (pad + (y / gridSize) * h);
    return [sx, sy];
  }

  // Background
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Grid
  ctx.strokeStyle = "#1f2937";
  ctx.lineWidth = 0.5;
  for (let i = 0; i <= 10; i++) {
    const t = (i / 10) * gridSize;

    const [x1, y1] = w2c(t, 0);
    const [x2, y2] = w2c(t, gridSize);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();

    const [xa, ya] = w2c(0, t);
    const [xb, yb] = w2c(gridSize, t);
    ctx.beginPath();
    ctx.moveTo(xa, ya);
    ctx.lineTo(xb, yb);
    ctx.stroke();
  }

  // Helper to find world position by label
  function findWaypointPosition(label) {
    const id = String(label);
    const a = airports.find((x) => String(x.id) === id);
    if (a) return { x: a.x, y: a.y };
    const t = targets.find((x) => String(x.id) === id);
    if (t) return { x: t.x, y: t.y };
    return null;
  }

  // SAMs
  sams.forEach((sam, idx) => {
    const pos = sam.pos || sam.position;
    if (!pos || pos.length !== 2) return;
    const [sx, sy] = w2c(pos[0], pos[1]);
    const r = Number(sam.range || 0);
    const rpx = (r / gridSize) * (canvas.width - 40);

    ctx.beginPath();
    ctx.arc(sx, sy, rpx, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(234,179,8,0.18)";
    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = 1;
    ctx.fill();
    ctx.stroke();

    const size = 7;
    ctx.beginPath();
    ctx.moveTo(sx, sy - size);
    ctx.lineTo(sx - size, sy + size);
    ctx.lineTo(sx + size, sy + size);
    ctx.closePath();
    ctx.fillStyle = "#f97316";
    ctx.strokeStyle = "#c2410c";
    ctx.fill();
    ctx.stroke();

    // Draw SAM number label (S0, S1, S2, ...)
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 10px system-ui";
    ctx.fillText(`S${idx}`, sx + 10, sy + 4);

    // Highlight if selected - show dashed boundary
    if (
      state.selectedObject &&
      state.selectedObject.kind === "sam" &&
      state.selectedObject.index === idx
    ) {
      ctx.beginPath();
      ctx.arc(sx, sy, rpx, 0, Math.PI * 2);
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  });

  // Wrapped SAM Polygons - draw dashed yellow line around wrapped SAMs
  console.log("drawEnvironment: wrappedPolygons count =", state.wrappedPolygons ? state.wrappedPolygons.length : 0);
  if (state.wrappedPolygons && state.wrappedPolygons.length > 0) {
    state.wrappedPolygons.forEach((polygon) => {
      if (!polygon || polygon.length < 3) return;

      ctx.beginPath();
      const firstPoint = w2c(polygon[0][0], polygon[0][1]);
      ctx.moveTo(firstPoint[0], firstPoint[1]);

      for (let i = 1; i < polygon.length; i++) {
        const point = w2c(polygon[i][0], polygon[i][1]);
        ctx.lineTo(point[0], point[1]);
      }
      ctx.closePath();

      // Dashed yellow line
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 1;
      ctx.setLineDash([5, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }

  // Airports
  airports.forEach((a, idx) => {
    const [x, y] = w2c(a.x, a.y);
    const size = 6;
    ctx.fillStyle = "#3b82f6";
    ctx.strokeStyle = "#1d4ed8";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(x - size, y - size, size * 2, size * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#e5e7eb";
    ctx.font = "10px system-ui";
    ctx.fillText(String(a.id), x + 8, y - 8);

    // Highlight if selected
    if (
      state.selectedObject &&
      state.selectedObject.kind === "airport" &&
      state.selectedObject.index === idx
    ) {
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2;
      ctx.strokeRect(x - size - 3, y - size - 3, (size * 2) + 6, (size * 2) + 6);
    }
  });

  // Helper function to check if a point is inside any SAM polygon
  function isInsideAnySAM(px, py) {
    for (const sam of sams) {
      const pos = sam.pos || sam.position;
      if (!pos || pos.length !== 2) continue;

      const samX = pos[0];
      const samY = pos[1];
      const range = Number(sam.range || 0);

      // Calculate distance from point to SAM center
      const dx = px - samX;
      const dy = py - samY;
      const distance = Math.sqrt(dx * dx + dy * dy);

      // If inside this SAM's range, return true
      if (distance < range) {
        return true;
      }
    }
    return false;
  }

  // Targets
  targets.forEach((t, idx) => {
    const [x, y] = w2c(t.x, t.y);
    const r = 4;
    const type = (t.type || "a").toLowerCase();
    let color = "#93c5fd";
    if (type === "b") color = "#facc15";
    if (type === "c") color = "#fb923c";
    if (type === "d") color = "#ef4444";
    if (type === "e") color = "#b91c1c";

    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.strokeStyle = "#111827";
    ctx.lineWidth = 1;
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#e5e7eb";
    ctx.font = "9px system-ui";
    const priority = t.priority || 5;
    ctx.fillText(`${t.id}-${priority}`, x + 6, y + 3);

    // Draw green X if target is visited (completed in previous checkpoint segment)
    const isVisited = state.visitedTargets && state.visitedTargets.includes(t.id);
    if (isVisited) {
      ctx.strokeStyle = "#22c55e";  // green
      ctx.lineWidth = 3;
      const xSize = 10;
      ctx.beginPath();
      ctx.moveTo(x - xSize, y - xSize);
      ctx.lineTo(x + xSize, y + xSize);
      ctx.moveTo(x + xSize, y - xSize);
      ctx.lineTo(x - xSize, y + xSize);
      ctx.stroke();
    }
    // Draw red X if target is inside a SAM polygon (no-fly zone)
    else if (isInsideAnySAM(t.x, t.y)) {
      ctx.strokeStyle = "#ef4444";
      ctx.lineWidth = 2;
      const xSize = 8;
      ctx.beginPath();
      ctx.moveTo(x - xSize, y - xSize);
      ctx.lineTo(x + xSize, y + xSize);
      ctx.moveTo(x + xSize, y - xSize);
      ctx.lineTo(x - xSize, y + xSize);
      ctx.stroke();
    }

    if (
      state.selectedObject &&
      state.selectedObject.kind === "target" &&
      state.selectedObject.index === idx
    ) {
      ctx.beginPath();
      ctx.arc(x, y, r + 5, 0, Math.PI * 2);
      ctx.strokeStyle = "#facc15";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  });

  // Routes (per drone) - only draw if trajectory is visible
  const colors = {
    "1": "#3b82f6",
    "2": "#f97316",
    "3": "#22c55e",
    "4": "#ef4444",
    "5": "#a855f7",
  };

  // Debug: Log trajectory info once per draw cycle (throttled)
  if (_drawEnvFrameCount % 120 === 1) {
    Object.entries(state.routes || {}).forEach(([did, info]) => {
      const trajLen = info.trajectory ? info.trajectory.length : 0;
      const hasFullTraj = info._fullTrajectory ? info._fullTrajectory.length : 0;
      if (trajLen > 0) {
        const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
        const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
        console.log(`[drawEnv] D${did}: trajectory=${trajLen}pts ${firstPt}→${lastPt}, _fullTrajectory=${hasFullTraj}pts`);
      }
    });
  }

  Object.entries(state.routes || {}).forEach(([did, info]) => {
    // Check if this trajectory should be visible
    if (!state.trajectoryVisible[did]) return;

    // Use trajectory if available (contains SAM-avoiding paths), otherwise use route waypoints
    const trajectory = info.trajectory || [];
    const route = info.route || [];

    ctx.beginPath();

    if (trajectory.length > 0) {
      // Draw the full SAM-avoiding trajectory
      trajectory.forEach((point, idx) => {
        const [cx, cy] = w2c(point[0], point[1]);
        if (idx === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      });
    } else if (route.length > 0) {
      // Fallback: draw straight lines between waypoints
      route.forEach((label, idx) => {
        const pos = findWaypointPosition(label);
        if (!pos) return;
        const [cx, cy] = w2c(pos.x, pos.y);
        if (idx === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      });
    }

    ctx.strokeStyle = colors[did] || "#22c55e";
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 3]);
    ctx.stroke();
    ctx.setLineDash([]);
  });

  // Draw animated drones (active animation OR frozen at checkpoint OR drones positioned after replan)
  const shouldDrawDrones = state.animation.active ||
                          state.checkpoint?.active ||
                          (state.animation.drones && Object.keys(state.animation.drones).length > 0);
  if (shouldDrawDrones && state.animation.drones) {
    Object.entries(state.animation.drones).forEach(([did, droneState]) => {
      // Draw if animating OR frozen at checkpoint OR drone has a valid position
      if (!droneState.animating && !state.checkpoint?.active && droneState.distanceTraveled === undefined) return;
      if (!state.trajectoryVisible[did]) return;

      const routeInfo = state.routes[did];
      if (!routeInfo) return;

      // Use trajectory if available (SAM-avoiding path), otherwise fall back to route waypoints
      const trajectory = routeInfo.trajectory || [];
      const route = routeInfo.route || [];
      const progress = droneState.progress;

      let droneX, droneY;

      // If frozen at checkpoint, draw drone at the splitPoint (end of prefix)
      if (state.checkpoint?.active && state.checkpoint.segments[did]?.splitPoint) {
        const sp = state.checkpoint.segments[did].splitPoint;
        droneX = sp[0];
        droneY = sp[1];
      } else if (trajectory.length >= 2) {
        // Animate along the full SAM-avoiding trajectory using distance-based interpolation
        // This ensures uniform speed regardless of segment count (SAM circles have many short segments)
        const cumulativeDistances = droneState.cumulativeDistances || [];
        const totalDistance = droneState.totalDistance || 0;

        if (totalDistance > 0 && cumulativeDistances.length === trajectory.length) {
          // Find position based on distance traveled (uniform speed)
          const targetDistance = droneState.distanceTraveled;

          // Binary search for the segment containing this distance
          let segmentIdx = 0;
          for (let i = 1; i < cumulativeDistances.length; i++) {
            if (cumulativeDistances[i] >= targetDistance) {
              segmentIdx = i - 1;
              break;
            }
            segmentIdx = i - 1;
          }

          const fromPoint = trajectory[segmentIdx];
          const toPoint = trajectory[segmentIdx + 1] || fromPoint;
          const segmentStartDist = cumulativeDistances[segmentIdx];
          const segmentEndDist = cumulativeDistances[segmentIdx + 1] || segmentStartDist;
          const segmentLength = segmentEndDist - segmentStartDist;

          const segmentProgress = segmentLength > 0
            ? (targetDistance - segmentStartDist) / segmentLength
            : 0;

          droneX = fromPoint[0] + (toPoint[0] - fromPoint[0]) * segmentProgress;
          droneY = fromPoint[1] + (toPoint[1] - fromPoint[1]) * segmentProgress;
        } else {
          // Fallback to segment-based (old behavior)
          const totalSegments = trajectory.length - 1;
          const currentSegmentFloat = progress * totalSegments;
          const currentSegment = Math.min(Math.floor(currentSegmentFloat), totalSegments - 1);
          const segmentProgress = currentSegmentFloat - currentSegment;

          const fromPoint = trajectory[currentSegment];
          const toPoint = trajectory[currentSegment + 1] || fromPoint;

          droneX = fromPoint[0] + (toPoint[0] - fromPoint[0]) * segmentProgress;
          droneY = fromPoint[1] + (toPoint[1] - fromPoint[1]) * segmentProgress;
        }
      } else if (route.length >= 2) {
        // Fallback: animate along waypoint route (straight lines)
        const totalSegments = route.length - 1;
        const currentSegmentFloat = progress * totalSegments;
        const currentSegment = Math.floor(currentSegmentFloat);
        const segmentProgress = currentSegmentFloat - currentSegment;

        if (currentSegment >= totalSegments) return;

        const fromLabel = route[currentSegment];
        const toLabel = route[currentSegment + 1];
        const fromPos = findWaypointPosition(fromLabel);
        const toPos = findWaypointPosition(toLabel);

        if (!fromPos || !toPos) return;

        droneX = fromPos.x + (toPos.x - fromPos.x) * segmentProgress;
        droneY = fromPos.y + (toPos.y - fromPos.y) * segmentProgress;
      } else {
        return;
      }

      const [cx, cy] = w2c(droneX, droneY);

      // Draw drone as a filled circle
      ctx.beginPath();
      ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      ctx.fillStyle = colors[did] || "#22c55e";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.fill();
      ctx.stroke();

      // Draw drone label
      ctx.fillStyle = "#ffffff";
      ctx.font = "bold 8px system-ui";
      ctx.fillText(`D${did}`, cx + 8, cy - 8);
    });
  }

  // Target Type Legend (top-right corner)
  const legendTypes = [
    { label: "A", color: "#93c5fd" },
    { label: "B", color: "#facc15" },
    { label: "C", color: "#fb923c" },
    { label: "D", color: "#ef4444" },
    { label: "E", color: "#b91c1c" },
  ];
  const legendX = canvas.width - 22;
  const legendY = 15;
  const dotRadius = 4;
  const lineHeight = 14;

  // Semi-transparent background box
  ctx.fillStyle = "rgba(2, 6, 23, 0.8)";
  ctx.strokeStyle = "#374151";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(legendX - 8, legendY - 8, 28, legendTypes.length * lineHeight + 10, 4);
  ctx.fill();
  ctx.stroke();

  // Draw each legend item
  legendTypes.forEach((item, i) => {
    const y = legendY + i * lineHeight;
    // Colored dot
    ctx.beginPath();
    ctx.arc(legendX, y + 4, dotRadius, 0, Math.PI * 2);
    ctx.fillStyle = item.color;
    ctx.fill();
    // Label
    ctx.fillStyle = "#e5e7eb";
    ctx.font = "bold 10px system-ui";
    ctx.fillText(item.label, legendX + 7, y + 8);
  });
}

// ----------------------------------------------------
// Edit mode toggle
// ----------------------------------------------------
function setEditMode(on) {
  const perms = getUiPermissions();

  if (on) {
    // Entering edit mode
    if (!perms.canEnterEdit) {
      appendDebugLine("Cannot enter edit mode in current state");
      return;
    }
    enterEditMode();
  } else {
    // Exiting edit mode - if currently editing, this cancels edits
    if (missionState.mode === MissionMode.EDITING_ENV) {
      cancelEdits();
    } else {
      // Not in EDITING_ENV, just update the low-level edit flag
      state.editMode = false;
      const btn = $("btn-toggle-edit");
      if (btn) {
        btn.textContent = "View";
        btn.classList.add("off");
      }
      state.addMode = null;
      state.selectedObject = null;
      state.dragging = null;
      drawEnvironment();
    }
  }
}

function attachEditToggle() {
  const btn = $("btn-toggle-edit");
  if (!btn) return;

  // Start with edit mode OFF for the state machine
  state.editMode = false;

  btn.addEventListener("click", () => {
    const perms = getUiPermissions();

    if (missionState.mode === MissionMode.EDITING_ENV) {
      // Currently editing - toggle OFF means cancel
      cancelEdits();
    } else if (perms.canEnterEdit) {
      // Can enter edit mode
      enterEditMode();
    } else {
      appendDebugLine("Cannot toggle edit mode in current state");
    }
  });
}

// ----------------------------------------------------
// Canvas editing: hit-test, drag, add, delete
// ----------------------------------------------------
function setAddMode(kind) {
  state.addMode = kind; // "airport", "target", "sam"
  if (kind) {
    appendDebugLine(`Add mode: ${kind.toUpperCase()} (click on canvas to place)`);
  } else {
    appendDebugLine("Add mode: off");
  }
}

function deleteSelected() {
  if (!state.env || !state.selectedObject) return;

  const { kind, index } = state.selectedObject;
  if (kind === "airport") {
    state.env.airports.splice(index, 1);
    updateAirportDropdowns();
  } else if (kind === "target") {
    state.env.targets.splice(index, 1);
  } else if (kind === "sam") {
    state.env.sams.splice(index, 1);
    // Recalculate wrapping after SAM deletion
    updateSamWrappingClientSide();
  }

  state.selectedObject = null;
  appendDebugLine(`Deleted ${kind}.`);
  drawEnvironment();
}

/**
 * Export environment to JSON file with naming convention: isr_envYYMMDDHH_n.json
 * Includes drone configs for persistence
 */
// Export counter for unique filenames
let _exportCounter = 1;

async function exportEnvironment() {
  try {
    if (!state.env) {
      alert("No environment loaded.");
      return;
    }

    appendDebugLine("📤 Exporting environment...");

    // Include drone configs in the environment
    state.env.drone_configs = state.droneConfigs;

    // Generate filename: isr_envYYMMDDHHMM_n.json
    const now = new Date();
    const yy = String(now.getFullYear()).slice(-2);
    const mo = String(now.getMonth() + 1).padStart(2, '0');
    const dd = String(now.getDate()).padStart(2, '0');
    const hh = String(now.getHours()).padStart(2, '0');
    const mm = String(now.getMinutes()).padStart(2, '0');
    const filename = `isr_env${yy}${mo}${dd}${hh}${mm}_${_exportCounter}.json`;
    _exportCounter++;

    // Create blob and download
    const blob = new Blob([JSON.stringify(state.env, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    appendDebugLine(`✅ Exported as ${filename}`);

  } catch (err) {
    appendDebugLine(`❌ Export error: ${err.message}`);
  }
}

// world <-> canvas helpers for editing
function canvasToWorld(canvas, clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const cx = (clientX - rect.left) * scaleX;
  const cy = (clientY - rect.top) * scaleY;

  const pad = 20;
  const w = canvas.width - 2 * pad;
  const h = canvas.height - 2 * pad;
  const gridSize = 100.0;

  const xWorld = ((cx - pad) / w) * gridSize;
  const yWorld = ((canvas.height - cy - pad) / h) * gridSize;

  return [xWorld, yWorld];
}

// hit-test on env (airport/target/sam)
function hitTestWorld(x, y) {
  if (!state.env) return null;
  const airports = state.env.airports || [];
  const targets = state.env.targets || [];
  const sams = state.env.sams || [];
  const R = 4.0; // threshold in world units (approx)

  function dist2(ax, ay, bx, by) {
    const dx = ax - bx;
    const dy = ay - by;
    return dx * dx + dy * dy;
  }

  let best = null;
  let bestD2 = Infinity;

  airports.forEach((a, idx) => {
    const d2 = dist2(x, y, a.x, a.y);
    if (d2 < bestD2 && d2 < R * R) {
      bestD2 = d2;
      best = { kind: "airport", index: idx };
    }
  });

  targets.forEach((t, idx) => {
    const d2 = dist2(x, y, t.x, t.y);
    if (d2 < bestD2 && d2 < R * R) {
      bestD2 = d2;
      best = { kind: "target", index: idx };
    }
  });

  sams.forEach((s, idx) => {
    const pos = s.pos || s.position;
    if (!pos || pos.length !== 2) return;
    const d2 = dist2(x, y, pos[0], pos[1]);
    const thr = 3.0;
    if (d2 < bestD2 && d2 < thr * thr) {
      bestD2 = d2;
      best = { kind: "sam", index: idx };
    }
  });

  return best;
}

function attachCanvasEditing() {
  const canvas = $("env-canvas");
  if (!canvas) return;

  canvas.addEventListener("mousedown", (evt) => {
    if (!state.env || !state.editMode) return;

    const [wx, wy] = canvasToWorld(canvas, evt.clientX, evt.clientY);

    // If we are in "add mode", create an object here
    if (state.addMode) {
      if (state.addMode === "airport") {
        const airports = state.env.airports || (state.env.airports = []);
        const nextNum =
          airports
            .map((a) => {
              const m = String(a.id).match(/^A(\d+)$/i);
              return m ? parseInt(m[1], 10) : 0;
            })
            .reduce((a, b) => Math.max(a, b), 0) + 1;
        airports.push({ id: `A${nextNum}`, x: wx, y: wy });
        appendDebugLine(`Added Airport A${nextNum} at (${wx.toFixed(1)}, ${wy.toFixed(1)})`);
        updateAirportDropdowns();
      } else if (state.addMode === "target") {
        const targets = state.env.targets || (state.env.targets = []);
        const nextNum =
          targets
            .map((t) => {
              const m = String(t.id).match(/^T(\d+)$/i);
              return m ? parseInt(m[1], 10) : 0;
            })
            .reduce((a, b) => Math.max(a, b), 0) + 1;

        // Read type and priority from dropdowns
        const typeSelect = $("new-target-type");
        const prioritySelect = $("new-target-priority");
        const targetType = typeSelect ? typeSelect.value : "a";
        const targetPriority = prioritySelect ? parseInt(prioritySelect.value, 10) : 5;

        targets.push({
          id: `T${nextNum}`,
          x: wx,
          y: wy,
          priority: targetPriority,
          type: targetType,
        });
        appendDebugLine(`Added Target T${nextNum} (type=${targetType.toUpperCase()}, priority=${targetPriority}) at (${wx.toFixed(1)}, ${wy.toFixed(1)})`);
      } else if (state.addMode === "sam") {
        const sams = state.env.sams || (state.env.sams = []);

        // Read range from dropdown
        const rangeSelect = $("new-sam-range");
        const samRange = rangeSelect ? parseInt(rangeSelect.value, 10) : 15;

        sams.push({
          pos: [wx, wy],
          range: samRange,
        });
        appendDebugLine(`Added SAM (range=${samRange}) at (${wx.toFixed(1)}, ${wy.toFixed(1)})`);

        // Calculate wrapping immediately (client-side) and draw
        updateSamWrappingClientSide();
        state.addMode = null;
        drawEnvironment();
        return;
      }
      // After a single click, clear addMode (or keep it; your choice)
      state.addMode = null;
      drawEnvironment();
      return;
    }

    // Otherwise: hit-test and start dragging if something is hit
    const hit = hitTestWorld(wx, wy);
    state.selectedObject = hit;
    state.dragging = null;

    if (hit) {
      appendDebugLine(`Selected ${hit.kind.toUpperCase()} index ${hit.index}`);
      // Start dragging
      state.dragging = {
        kind: hit.kind,
        index: hit.index,
        offsetX: 0,
        offsetY: 0,
      };
    }

    drawEnvironment();
  });

  canvas.addEventListener("mousemove", (evt) => {
    if (!state.env || !state.editMode) return;
    if (!state.dragging) return;

    const [wx, wy] = canvasToWorld(canvas, evt.clientX, evt.clientY);
    const { kind, index } = state.dragging;

    if (kind === "airport") {
      const airports = state.env.airports || [];
      if (airports[index]) {
        airports[index].x = wx;
        airports[index].y = wy;
      }
    } else if (kind === "target") {
      const targets = state.env.targets || [];
      if (targets[index]) {
        targets[index].x = wx;
        targets[index].y = wy;
      }
    } else if (kind === "sam") {
      const sams = state.env.sams || [];
      if (sams[index]) {
        sams[index].pos = [wx, wy];
        // Update wrapping instantly using client-side calculation
        updateSamWrappingClientSide();
      }
    }

    drawEnvironment();
  });

  // Use window-level mouseup so we ALWAYS end drag, even if the mouse
  // leaves the canvas or some other UI element grabs focus.
  window.addEventListener("mouseup", () => {
    // Always end drag
    const wasDragging = state.dragging !== null;
    state.dragging = null;

    // Redraw if we were dragging something (wrapping already updated client-side during drag)
    if (wasDragging) {
      drawEnvironment();
    }
  });

  // If the window loses focus (e.g. you alt-tab), make sure we are not dragging.
  window.addEventListener("blur", () => {
    state.dragging = null;
  });

  // Note: We intentionally do NOT cancel dragging on mouseleave.
  // The window-level mouseup handler will properly end the drag
  // even if the mouse leaves the canvas, allowing wrapping to update.
}

// ----------------------------------------------------
// Drone config + sequence bar
// ----------------------------------------------------

// Update airport dropdowns when airports are added/deleted
function updateAirportDropdowns() {
  const airports = (state.env && state.env.airports) || [];
  const airportIds = airports.map((a) => String(a.id));

  for (let did = 1; did <= 5; did++) {
    const startSel = $(`cfg-d${did}-start`);
    const endSel = $(`cfg-d${did}-end`);

    // Store current selections
    const currentStart = startSel ? startSel.value : null;
    const currentEnd = endSel ? endSel.value : null;

    // Repopulate start dropdown (no flexible option for start)
    if (startSel) {
      startSel.innerHTML = "";
      airportIds.forEach((aid) => {
        const opt = document.createElement("option");
        opt.value = aid;
        opt.textContent = aid;
        startSel.appendChild(opt);
      });
    }

    // Repopulate end dropdown WITH flexible "-" option
    if (endSel) {
      endSel.innerHTML = "";
      // Add flexible endpoint option first
      const flexOpt = document.createElement("option");
      flexOpt.value = "-";
      flexOpt.textContent = "Any";
      endSel.appendChild(flexOpt);
      // Add airport options
      airportIds.forEach((aid) => {
        const opt = document.createElement("option");
        opt.value = aid;
        opt.textContent = aid;
        endSel.appendChild(opt);
      });
    }

    // Restore selections if they still exist, otherwise use first available
    if (startSel) {
      if (airportIds.includes(currentStart)) {
        startSel.value = currentStart;
      } else if (airportIds.length > 0) {
        startSel.value = airportIds[0];
        state.droneConfigs[String(did)].start_airport = airportIds[0];
      }
    }

    if (endSel) {
      // Check if current selection is valid (flexible "-" or a known airport)
      if (currentEnd === "-" || airportIds.includes(currentEnd)) {
        endSel.value = currentEnd;
      } else if (airportIds.length > 0) {
        endSel.value = airportIds[0];
        state.droneConfigs[String(did)].end_airport = airportIds[0];
      }
    }
  }
}

function initDroneConfigsFromEnv() {
  const airports = (state.env && state.env.airports) || [];
  const airportIds = airports.map((a) => String(a.id));
  const defaultAirport = airportIds[0] || "A1";

  // Check if drone_configs are saved in the environment
  const savedConfigs = (state.env && state.env.drone_configs) || {};

  function populateStartSelect(selectId) {
    const sel = $(selectId);
    if (!sel) return;
    sel.innerHTML = "";
    airportIds.forEach((aid) => {
      const opt = document.createElement("option");
      opt.value = aid;
      opt.textContent = aid;
      sel.appendChild(opt);
    });
  }

  function populateEndSelect(selectId) {
    const sel = $(selectId);
    if (!sel) return;
    sel.innerHTML = "";
    // Add flexible endpoint option first
    const flexOpt = document.createElement("option");
    flexOpt.value = "-";
    flexOpt.textContent = "Any";
    sel.appendChild(flexOpt);
    // Add airport options
    airportIds.forEach((aid) => {
      const opt = document.createElement("option");
      opt.value = aid;
      opt.textContent = aid;
      sel.appendChild(opt);
    });
  }

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const saved = savedConfigs[idStr] || {};

    // Use saved config if available, otherwise use defaults
    const enabledDefault = saved.enabled !== undefined ? saved.enabled : true;
    const fuelDefault = saved.fuel_budget !== undefined ? saved.fuel_budget : 150;
    const startDefault = saved.start_airport || (airportIds.includes(`A${did}`) ? `A${did}` : defaultAirport);
    const endDefault = saved.end_airport || (airportIds.includes(`A${did}`) ? `A${did}` : defaultAirport);
    const accessDefault = saved.target_access || { a: true, b: true, c: true, d: true, e: true };

    state.droneConfigs[idStr] = {
      enabled: enabledDefault,
      fuel_budget: fuelDefault,
      start_airport: startDefault,
      end_airport: endDefault,
      target_access: accessDefault,
    };

    populateStartSelect(`cfg-d${did}-start`);
    populateEndSelect(`cfg-d${did}-end`);

    const cfg = state.droneConfigs[idStr];
    const cbEnabled = $(`cfg-d${did}-enabled`);
    const fuelInput = $(`cfg-d${did}-fuel`);
    const startSel = $(`cfg-d${did}-start`);
    const endSel = $(`cfg-d${did}-end`);

    if (cbEnabled) cbEnabled.checked = cfg.enabled;
    if (fuelInput) fuelInput.value = cfg.fuel_budget;
    if (startSel && cfg.start_airport) startSel.value = cfg.start_airport;
    if (endSel && cfg.end_airport) endSel.value = cfg.end_airport;

    // Update target access checkboxes
    ["a", "b", "c", "d", "e"].forEach((t) => {
      const cb = $(`cfg-d${did}-type-${t}`);
      if (cb && cfg.target_access[t] !== undefined) {
        cb.checked = cfg.target_access[t];
      }
    });
  }
}

function attachConfigListeners() {
  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);

    const cbEnabled = $(`cfg-d${did}-enabled`);
    const fuelInput = $(`cfg-d${did}-fuel`);
    const startSel = $(`cfg-d${did}-start`);
    const endSel = $(`cfg-d${did}-end`);

    if (cbEnabled) {
      cbEnabled.addEventListener("change", () => {
        state.droneConfigs[idStr].enabled = cbEnabled.checked;
        invalidateMission(`Drone ${idStr} enabled changed`);
      });
    }

    if (fuelInput) {
      fuelInput.addEventListener("change", () => {
        const v = parseFloat(fuelInput.value || "0");
        state.droneConfigs[idStr].fuel_budget = isNaN(v) ? 0 : v;
        // Update stats to reflect new fuel budget
        updateStatsFromRoutes();
        invalidateMission(`Drone ${idStr} fuel_budget changed`);
      });
    }

    if (startSel) {
      startSel.addEventListener("change", () => {
        state.droneConfigs[idStr].start_airport = startSel.value;
        invalidateMission(`Drone ${idStr} start_airport changed`);
      });
    }

    if (endSel) {
      endSel.addEventListener("change", () => {
        state.droneConfigs[idStr].end_airport = endSel.value;
        invalidateMission(`Drone ${idStr} end_airport changed`);
      });
    }

    ["a", "b", "c", "d", "e"].forEach((t) => {
      const cb = $(`cfg-d${did}-type-${t}`);
      if (!cb) return;
      cb.addEventListener("change", () => {
        state.droneConfigs[idStr].target_access[t] = cb.checked;
        invalidateMission(`Drone ${idStr} target_access changed`);
      });
    });
  }
}


function attachSequenceBar() {
  const droneSel = $("sequence-drone-select");
  const seqInput = $("sequence-input");
  const applyBtn = $("sequence-apply-btn");

  if (!droneSel || !seqInput || !applyBtn) return;

  droneSel.addEventListener("change", () => {
    state.currentDroneForSeq = droneSel.value;
    const seq = state.sequences[state.currentDroneForSeq] || "";
    seqInput.value = seq;
  });

  applyBtn.addEventListener("click", async () => {
    const seqText = seqInput.value.trim().toUpperCase();
    const droneId = state.currentDroneForSeq;

    if (!seqText) {
      appendDebugLine(`Empty sequence for D${droneId}`);
      return;
    }

    if (!state.env) {
      appendDebugLine(`No environment loaded - cannot apply sequence`);
      return;
    }

    // Get fuel budget from drone config
    const droneConfig = state.droneConfigs[droneId] || {};
    const fuelBudget = droneConfig.fuel_budget || 300;

    appendDebugLine(`Applying sequence for D${droneId}: ${seqText}`);

    try {
      const res = await fetch("/api/apply_sequence", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          drone_id: droneId,
          sequence: seqText,
          env: state.env,
          fuel_budget: fuelBudget
        })
      });

      const data = await res.json();

      if (data.success) {
        // Update state with new route data
        state.sequences[droneId] = seqText;
        state.routes[droneId] = {
          route: data.route,
          sequence: seqText,
          points: data.points,
          distance: data.distance,
          fuel_budget: fuelBudget,
          trajectory: data.trajectory
        };

        appendDebugLine(
          `Applied D${droneId}: ${data.route.length} waypoints, ${Math.round(data.distance)} fuel, ${data.points} pts`
        );

        // Redraw canvas with new trajectory
        drawEnvironment();

        // Update stats display
        updateStatsFromRoutes();
      } else {
        appendDebugLine(`Error applying sequence: ${data.error || "Unknown error"}`);
      }
    } catch (err) {
      appendDebugLine(`Failed to apply sequence: ${err.message}`);
    }
  });
}

// ----------------------------------------------------
// Stats update
// ----------------------------------------------------
function updateStatsFromRoutes() {
  console.log("📊 updateStatsFromRoutes called");
  console.log("📊 state.routes:", state.routes);

  const envTargets = (state.env && state.env.targets) || [];
  const totalTargets = envTargets.length; // reserved if you need it

  let missionPoints = 0;
  let missionFuel = 0;
  let missionBudget = 0;
  let missionVisited = 0;

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const routeInfo = state.routes[idStr] || {};
    const droneConfig = state.droneConfigs[idStr] || {};
    const route = routeInfo.route || [];
    const points = Number(routeInfo.points || 0);
    const distance = Number(routeInfo.distance || 0);
    console.log(`📊 Drone ${did}: points=${points}, distance=${distance}, route=`, route);

    // Use fuel budget from droneConfigs (Config tab) as the source of truth
    // Fall back to route info if config not available
    const budget = Number(droneConfig.fuel_budget || routeInfo.fuel_budget || 0);

    const targetCount = route.filter((label) =>
      String(label).toUpperCase().startsWith("T")
    ).length;

    const fuelUsed = distance; // for now, assume distance == fuel
    const pf = fuelUsed > 0 ? points / fuelUsed : 0;

    setText(`stat-d${did}-tp`, `${targetCount} / ${points}`);
    setText(`stat-d${did}-fuel`, `${Math.round(fuelUsed)} / ${Math.round(budget)}`);
    setText(`stat-d${did}-pf`, pf.toFixed(2));

    if (route && route.length > 0) {
      missionPoints += points;
      missionFuel += fuelUsed;
      missionBudget += budget;
      missionVisited += targetCount;
    }
  }

  const missionPf = missionFuel > 0 ? missionPoints / missionFuel : 0;
  setText("stat-mission-tp", `${missionVisited} / ${missionPoints}`);
  setText(
    "stat-mission-fuel",
    `${Math.round(missionFuel)} / ${Math.round(missionBudget)}`
  );
  setText("stat-mission-pf", missionPf.toFixed(2));
}

function updateAllocationDisplay(allocations, strategy) {
  const container = $("allocation-display");
  if (!container) return;

  // Handle null/undefined allocations
  const allocs = allocations || {};
  const totalTargets = (state.env && state.env.targets) ? state.env.targets.length : 0;

  // Build allocation from routes if allocations is empty but routes exist
  let effectiveAllocs = allocs;
  if (Object.keys(allocs).length === 0 && state.routes && Object.keys(state.routes).length > 0) {
    effectiveAllocs = {};
    for (const [did, routeData] of Object.entries(state.routes)) {
      const route = routeData.route || [];
      effectiveAllocs[did] = route.filter(wp => String(wp).startsWith('T'));
    }
  }

  let html = '';
  let totalAllocated = 0;

  for (let did = 1; did <= 5; did++) {
    const idStr = String(did);
    const targets = effectiveAllocs[idStr] || [];
    const cfg = state.droneConfigs[idStr] || {};

    if (cfg.enabled === false) continue;

    totalAllocated += targets.length;
    const color = targets.length > 0 ? '#4ade80' : '#f87171';
    html += `<div style="color: ${color};">D${did}: ${targets.length > 0 ? targets.join(', ') : '(none)'}</div>`;
  }

  if (totalAllocated === 0 && totalTargets === 0) {
    container.innerHTML = '<div style="color: #888;">No targets in environment</div>';
    return;
  }

  // Add strategy in parentheses if available
  const strategyText = strategy ? ` (${strategy})` : '';
  html += `<div style="color: #60a5fa; margin-top: 4px;">Total: ${totalAllocated}/${totalTargets} targets${strategyText}</div>`;
  container.innerHTML = html;
}

// ----------------------------------------------------
// Import / Export
// ----------------------------------------------------
// ----------------------------------------------------
// Import / Export
// ----------------------------------------------------
function attachIOHandlers() {
  const fileInput = $("file-input");
  const btnImport = $("btn-import");
  const btnExport = $("btn-export");

  // If there's no file input, we can't import anything
  if (!fileInput) {
    appendDebugLine("attachIOHandlers: no file-input element found; import disabled.");
    return;
  }

  // Wire up the Import button if present
  if (btnImport) {
    btnImport.addEventListener("click", () => {
      fileInput.value = "";
      fileInput.click();
    });
  } else {
    appendDebugLine("attachIOHandlers: no btn-import element found; import button disabled.");
  }

  // File selection / JSON import
  fileInput.addEventListener("change", (evt) => {
    const file = evt.target.files && evt.target.files[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const text = e.target.result;
        const data = JSON.parse(text);

        // Handle different formats:
        // 1. Wrapped format: { environment: {...}, ... }
        // 2. Raw format with drone_configs: { airports: [...], drone_configs: {...}, ... }
        // 3. Old format: { airports: [...], ... } (no drone_configs)
        if (data.environment) {
          // Wrapped format from old API export
          state.env = data.environment;
        } else {
          // Raw format
          state.env = data;
        }

        state.envFilename = file.name;

        // Reset mission when importing a new environment
        state.missionId = null;
        state.routes = {};
        state.sequences = {};
        state.trajectoryVisible = {};

        // Stop any running animation (guard in case animation object not yet initialized)
        if (state.animation) {
          if (state.animation.animationId) {
            cancelAnimationFrame(state.animation.animationId);
            state.animation.animationId = null;
          }
          state.animation.active = false;
          state.animation.drones = {};
        }

        // Update filename label
        const envNameEl = $("env-filename");
        if (envNameEl) {
          envNameEl.textContent = file.name;
        }

        appendDebugLine(`Imported environment from ${file.name}`);

        // Initialize drone configs (will use saved configs from env if present)
        initDroneConfigsFromEnv();

        // Recompute SAM wrapping client-side and redraw
        updateSamWrappingClientSide();
        drawEnvironment();
      } catch (err) {
        appendDebugLine("Error parsing JSON: " + err);
        console.error("Error parsing JSON:", err);
        alert("Error parsing JSON: " + err);
      }
    };

    reader.readAsText(file);
  });

  // Wire up Export button if present
  if (btnExport) {
    btnExport.addEventListener("click", exportEnvironment);
  } else {
    appendDebugLine("attachIOHandlers: no btn-export element found; export disabled.");
  }
}

// ----------------------------------------------------
// Optimization buttons
// ----------------------------------------------------
function attachOptimizationHandlers() {
  const btnInsert = $("btn-optimize-insert");
  const btnSwap = $("btn-optimize-swap");
  const btnCrossRemove = $("btn-optimize-cross");

  if (btnInsert) {
    btnInsert.addEventListener("click", async () => {
      if (!state.routes || Object.keys(state.routes).length === 0) {
        appendDebugLine("No routes to optimize. Run planner first.");
        return;
      }
      appendDebugLine("Running Insert Missed optimization...");
      btnInsert.disabled = true;
      btnInsert.textContent = "Working...";

      try {
        const droneConfigs = state.droneConfigs;
        const resp = await fetch("/api/insert_missed_optimize", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            solution: {
              routes: state.routes,
              distance_matrix: state.distanceMatrix
            },
            env: state.env,
            drone_configs: droneConfigs
          })
        });
        const data = await resp.json();

        if (data.success) {
          const insertions = data.insertions || [];
          if (insertions.length > 0) {
            // Update routes with optimized solution
            state.routes = data.routes;
            state.sequences = data.sequences;

            // Update sequence display
            for (const [did, seq] of Object.entries(data.sequences)) {
              state.sequences[did] = seq;
            }

            appendDebugLine(`Insert optimization: ${insertions.length} targets inserted.`);
            insertions.forEach(ins => {
              appendDebugLine(`  ${ins.target} -> D${ins.drone}`);
            });

            // Recalculate trajectories for modified routes
            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();
          } else {
            appendDebugLine("Insert optimization: No insertions possible (all visited or fuel exhausted).");
          }
        } else {
          appendDebugLine("Insert optimization error: " + (data.error || "Unknown error"));
        }
      } catch (err) {
        appendDebugLine("Insert optimization error: " + err.message);
      } finally {
        btnInsert.disabled = false;
        btnInsert.textContent = "Insert Missed";
      }
    });
  }

  if (btnSwap) {
    console.log("Swap button handler attached");
    btnSwap.addEventListener("click", async () => {
      console.log("Swap button clicked!", state.routes);
      if (!state.routes || Object.keys(state.routes).length === 0) {
        appendDebugLine("No routes to optimize. Run planner first.");
        return;
      }
      appendDebugLine("Running Trajectory Swap optimization (auto-iterate mode)...");
      btnSwap.disabled = true;
      btnSwap.textContent = "Optimizing...";

      try {
        const droneConfigs = state.droneConfigs;
        // Debug: Log what we're sending to the backend
        console.log("=== SWAP CLOSER REQUEST ===");
        console.log("Current routes:", state.routes);
        console.log("Distance matrix available:", !!state.distanceMatrix);
        if (state.distanceMatrix) {
          console.log("Distance matrix labels:", state.distanceMatrix.labels);
        }

        const resp = await fetch("/api/trajectory_swap_optimize", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            solution: {
              routes: state.routes,
              distance_matrix: state.distanceMatrix
            },
            env: state.env,
            drone_configs: droneConfigs
          })
        });
        const data = await resp.json();

        // Debug: Log the complete response
        console.log("=== SWAP CLOSER RESPONSE ===");
        console.log("Success:", data.success);
        console.log("Swaps made:", data.swaps_made);
        console.log("Number of swaps:", (data.swaps_made || []).length);
        if (data.swaps_made && data.swaps_made.length > 0) {
          console.log("Swap details:");
          data.swaps_made.forEach(swap => {
            console.log(`  ${swap.target}: D${swap.from_drone} -> D${swap.to_drone}`);
          });
        }
        console.log("Routes in response:", Object.keys(data.routes || {}));
        console.log("Full response data:", data);

        if (data.success) {
          const swaps = data.swaps_made || [];
          const iterations = data.iterations || 1;
          const bestIteration = data.best_iteration || iterations;
          const bestDistance = data.best_distance || 0;
          const cycleDetected = data.cycle_detected || false;
          const converged = data.converged || false;

          if (swaps.length > 0) {
            // Update routes with optimized solution
            state.routes = data.routes;
            state.sequences = data.sequences;

            // Update sequence display
            for (const [did, seq] of Object.entries(data.sequences)) {
              state.sequences[did] = seq;
            }

            // Show summary of auto-iteration
            if (cycleDetected) {
              appendDebugLine(`Swap optimization: Cycle detected after ${iterations} iterations.`);
              appendDebugLine(`  Using best solution from iteration ${bestIteration} (distance: ${bestDistance.toFixed(1)})`);
            } else if (converged) {
              appendDebugLine(`Swap optimization: Converged after ${iterations} iterations.`);
            } else {
              appendDebugLine(`Swap optimization: Completed ${iterations} iterations.`);
            }

            appendDebugLine(`  Total swaps: ${swaps.length}, Best distance: ${bestDistance.toFixed(1)}`);

            // Show individual swaps (limit to first 10 if many)
            const swapsToShow = swaps.slice(0, 10);
            swapsToShow.forEach(swap => {
              const iter = swap.iteration ? ` (iter ${swap.iteration})` : '';
              appendDebugLine(`  ${swap.target}: D${swap.from_drone} -> D${swap.to_drone}${iter}`);
            });
            if (swaps.length > 10) {
              appendDebugLine(`  ... and ${swaps.length - 10} more swaps`);
            }

            // Recalculate trajectories for modified routes
            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();
          } else {
            appendDebugLine(`Swap optimization: No beneficial swaps found (${iterations} iteration${iterations > 1 ? 's' : ''}).`);
          }
        } else {
          appendDebugLine("Swap optimization error: " + (data.error || "Unknown error"));
        }
      } catch (err) {
        appendDebugLine("Swap optimization error: " + err.message);
      } finally {
        btnSwap.disabled = false;
        btnSwap.textContent = "Swap Closer";
      }
    });
  }

  if (btnCrossRemove) {
    console.log("Cross Remove button handler attached");
    btnCrossRemove.addEventListener("click", async () => {
      console.log("Cross Remove button clicked!", state.routes);
      if (!state.routes || Object.keys(state.routes).length === 0) {
        appendDebugLine("No routes to optimize. Run planner first.");
        return;
      }
      appendDebugLine("Running Crossing Removal optimization...");
      btnCrossRemove.disabled = true;
      btnCrossRemove.textContent = "Working...";

      try {
        const droneConfigs = state.droneConfigs;
        const resp = await fetch("/api/crossing_removal_optimize", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            solution: { routes: state.routes },
            env: state.env,
            drone_configs: droneConfigs
          })
        });
        const data = await resp.json();

        if (data.success) {
          const fixes = data.fixes_made || [];
          if (fixes.length > 0) {
            // Update routes with optimized solution
            state.routes = data.routes;
            state.sequences = data.sequences;

            // Update sequence display
            for (const [did, seq] of Object.entries(data.sequences)) {
              state.sequences[did] = seq;
            }

            appendDebugLine(`Crossing removal: ${fixes.length} crossings fixed.`);
            fixes.forEach(fix => {
              appendDebugLine(`  Drone ${fix.drone}: reversed segment ${fix.segment_i}-${fix.segment_j}`);
            });

            // Recalculate trajectories for modified routes
            await regenerateTrajectories();
            updateStatsFromRoutes();
            drawEnvironment();
          } else {
            appendDebugLine("Crossing removal: No crossings found.");
          }
        } else {
          appendDebugLine("Crossing removal error: " + (data.error || "Unknown error"));
        }
      } catch (err) {
        appendDebugLine("Crossing removal error: " + err.message);
      } finally {
        btnCrossRemove.disabled = false;
        btnCrossRemove.textContent = "Cross Remove";
      }
    });
  }
}

async function regenerateTrajectories() {
  // Regenerate SAM-avoiding trajectories for all routes after swap optimization
  const droneConfigs = state.droneConfigs;

  for (const [did, routeData] of Object.entries(state.routes)) {
    if (!routeData.route || routeData.route.length < 2) continue;

    try {
      const resp = await fetch("/api/apply_sequence", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          drone_id: did,
          sequence: routeData.route.join(","),
          env: state.env,
          fuel_budget: droneConfigs[did]?.fuel_budget || 300
        })
      });
      const data = await resp.json();

      if (data.success) {
        state.routes[did].trajectory = data.trajectory;
        state.routes[did].distance = data.distance;
        state.routes[did].points = data.points;
      }
    } catch (err) {
      console.error(`Failed to regenerate trajectory for D${did}:`, err);
    }
  }
}

// ----------------------------------------------------
// Planner integration
// ----------------------------------------------------
async function fetchWrappedPolygons(recalculate = false) {
  console.log("fetchWrappedPolygons called, recalculate =", recalculate, "sams =", state.env?.sams?.length || 0);

  // Abort any pending wrapping request
  if (state.wrappingAbortController) {
    console.log("fetchWrappedPolygons: aborting previous request");
    state.wrappingAbortController.abort();
  }

  // Create new abort controller for this request
  state.wrappingAbortController = new AbortController();
  const signal = state.wrappingAbortController.signal;

  try {
    let resp, data;

    if (recalculate && state.env && state.env.sams && state.env.sams.length > 0) {
      // Recalculate wrapping with current SAM positions
      console.log("fetchWrappedPolygons: calling POST /api/calculate_wrapping with", state.env.sams.length, "SAMs");
      resp = await fetch("/api/calculate_wrapping", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ sams: state.env.sams }),
        signal: signal
      });
      data = await resp.json();
      console.log("fetchWrappedPolygons: response received", data);
    } else {
      // Get cached wrapping from last solve
      console.log("fetchWrappedPolygons: calling GET /api/wrapped_polygons (cached)");
      resp = await fetch("/api/wrapped_polygons", { signal: signal });
      data = await resp.json();
    }

    if (data && data.success && data.polygons) {
      console.log("fetchWrappedPolygons: received", data.polygons.length, "polygons");
      state.wrappedPolygons = data.polygons;
    } else {
      console.log("fetchWrappedPolygons: no polygons in response");
      state.wrappedPolygons = [];
    }
  } catch (err) {
    if (err.name === 'AbortError') {
      console.log("fetchWrappedPolygons: request was aborted (this is ok)");
      return; // Don't clear polygons on abort
    }
    console.error("fetchWrappedPolygons error:", err);
    state.wrappedPolygons = [];
  }
}

// ----------------------------------------------------
// Mission History & Segmented Replanning Helpers
// ----------------------------------------------------

/**
 * Save the initial environment snapshot (called on first load or reset)
 */
function saveInitialEnvSnapshot() {
  if (state.env) {
    state.initialEnvSnapshot = JSON.parse(JSON.stringify(state.env));
    state.missionHistory = [];
    state.visitedTargets = [];
    appendDebugLine("Initial environment snapshot saved");
  }
}

/**
 * Push current segment to mission history
 */
function pushMissionSegment(note = "") {
  const segment = {
    envSnapshot: JSON.parse(JSON.stringify(state.env)),
    droneConfigsSnapshot: JSON.parse(JSON.stringify(state.droneConfigs)),
    startStates: {},  // { did: { x, y } } - where each drone started this segment
    trajectoriesByDrone: {},  // { did: [[x,y], ...] }
    routesByDrone: {},  // { did: ["A1", "T1", ...] }
    visitedTargetsThisSegment: [],
    meta: {
      createdAt: new Date().toISOString(),
      note: note,
      segmentIndex: state.missionHistory.length,
    }
  };

  // Capture start states and trajectories from current routes
  Object.entries(state.routes || {}).forEach(([did, routeData]) => {
    const traj = routeData.trajectory || [];
    const route = routeData.route || [];

    // Start state is first point of trajectory (or start airport position)
    if (traj.length > 0) {
      segment.startStates[did] = { x: traj[0][0], y: traj[0][1] };
      segment.trajectoriesByDrone[did] = JSON.parse(JSON.stringify(traj));
    }
    segment.routesByDrone[did] = [...route];

    // Extract visited targets (anything starting with T)
    route.forEach(wp => {
      if (String(wp).startsWith("T") && !state.visitedTargets.includes(wp)) {
        segment.visitedTargetsThisSegment.push(wp);
        state.visitedTargets.push(wp);
      }
    });
  });

  state.missionHistory.push(segment);
  appendDebugLine(`Segment ${segment.meta.segmentIndex} saved (${segment.visitedTargetsThisSegment.length} targets visited)`);
  return segment;
}

/**
 * Build env2 for replanning after checkpoint freeze
 * - Creates synthetic start nodes at frozen positions
 * - Marks visited targets
 * - Returns modified env and droneConfigs
 */
function buildCheckpointEnv() {
  appendDebugLine(`buildCheckpointEnv: checkpoint.active=${state.checkpoint?.active}, segments=${Object.keys(state.checkpoint?.segments || {}).join(",") || "NONE"}`);
  appendDebugLine(`buildCheckpointEnv: visitedTargets=[${state.visitedTargets.join(", ")}]`);

  if (!state.checkpoint?.active || !state.checkpoint.segments) {
    appendDebugLine("buildCheckpointEnv: NOT a checkpoint replan (checkpoint.active is false or no segments)");
    return { env: state.env, droneConfigs: state.droneConfigs, isCheckpointReplan: false };
  }

  // Deep copy current env
  const env2 = JSON.parse(JSON.stringify(state.env));

  // Add synthetic start nodes for each frozen drone
  env2.synthetic_starts = env2.synthetic_starts || {};

  // Deep copy drone configs
  const newDroneConfigs = JSON.parse(JSON.stringify(state.droneConfigs));

  // Calculate remaining fuel for each drone
  Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
    if (!seg?.splitPoint) return;

    const nodeId = `D${did}_START`;
    env2.synthetic_starts[nodeId] = {
      id: nodeId,
      x: seg.splitPoint[0],
      y: seg.splitPoint[1],
    };

    // Update drone config to start from synthetic node
    if (newDroneConfigs[did]) {
      newDroneConfigs[did].start_airport = nodeId;

      // Calculate remaining fuel (original budget minus distance traveled)
      const originalBudget = newDroneConfigs[did].fuel_budget || 150;
      const distanceTraveled = seg.checkpointDist || 0;
      newDroneConfigs[did].fuel_budget = Math.max(0, originalBudget - distanceTraveled);
    }
  });

  // Mark visited targets (exclude them from planning or mark as completed)
  env2.visited_targets = [...state.visitedTargets];

  // Optionally filter out visited targets from the targets array
  // (depending on how backend handles this)
  // env2.targets = env2.targets.filter(t => !state.visitedTargets.includes(t.id));

  appendDebugLine(`buildCheckpointEnv: IS checkpoint replan. synthetic_starts=${Object.keys(env2.synthetic_starts).join(",")}, visited=${env2.visited_targets.length}`);

  return {
    env: env2,
    droneConfigs: newDroneConfigs,
    isCheckpointReplan: true,
  };
}

/**
 * Get joined trajectories for full mission replay
 * Concatenates all segment trajectories per drone
 */
function getJoinedTrajectories() {
  const joined = {};

  state.missionHistory.forEach((segment) => {
    Object.entries(segment.trajectoriesByDrone || {}).forEach(([did, traj]) => {
      if (!joined[did]) {
        joined[did] = [];
      }

      if (joined[did].length > 0 && traj.length > 0) {
        // Check if we need to bridge (avoid duplicate points)
        const lastPoint = joined[did][joined[did].length - 1];
        const firstPoint = traj[0];
        if (Math.abs(lastPoint[0] - firstPoint[0]) < 0.001 &&
            Math.abs(lastPoint[1] - firstPoint[1]) < 0.001) {
          // Skip duplicate point
          joined[did] = joined[did].concat(traj.slice(1));
        } else {
          joined[did] = joined[did].concat(traj);
        }
      } else {
        joined[did] = joined[did].concat(traj);
      }
    });
  });

  return joined;
}

// resetMission() is now defined in the Mission Mode Action Handlers section

async function runPlanner() {
  if (!state.env) {
    alert("No environment loaded yet.");
    return;
  }

  // Check if we can solve in current state
  const perms = getUiPermissions();
  if (!perms.canSolve) {
    appendDebugLine("Cannot run planner in current state");
    return;
  }

  // Cancel any existing solve request
  if (state.solveAbortController) {
    state.solveAbortController.abort();
  }

  // Create new abort controller
  state.solveAbortController = new AbortController();

  // Grey out the Run Planner button while solving
  const btnRun = $("btn-run-planner");
  if (btnRun) {
    btnRun.disabled = true;
    btnRun.textContent = "Solving...";
    btnRun.style.opacity = "0.5";
    btnRun.style.cursor = "not-allowed";
  }

  // Get the selected allocation strategy
  const strategySelect = $("allocation-strategy");
  const strategy = strategySelect ? strategySelect.value : "efficient";

  // Check if this is a checkpoint replan (solve from frozen positions)
  const isCheckpointMode = missionState.mode === MissionMode.CHECKPOINT;
  const { env: envToSolve, droneConfigs: configsToSolve, isCheckpointReplan } = buildCheckpointEnv();

  // If this is the first solve (no history), save the initial snapshot
  if (missionState.committedSegments.length === 0 && !isCheckpointMode) {
    saveInitialEnvSnapshot();
    // Also set the accepted env if not already set
    if (!missionState.acceptedEnv) {
      missionState.acceptedEnv = JSON.parse(JSON.stringify(state.env));
    }
  }

  const payload = {
    env: envToSolve,
    drone_configs: configsToSolve,
    allocation_strategy: strategy,
    is_checkpoint_replan: isCheckpointReplan,
    visited_targets: state.visitedTargets,
  };

  const modeLabel = isCheckpointReplan ? "CHECKPOINT REPLAN" : "FRESH SOLVE";
  appendDebugLine(`➡ [${modeLabel}] Sending /api/solve_with_allocation (strategy: ${strategy})...`);

  let bodyStr;
  try {
    bodyStr = JSON.stringify(payload);
  } catch (jsonErr) {
    appendDebugLine(`❌ Failed to serialize payload: ${jsonErr}`);
    $("debug-output").textContent = "Error serializing payload: " + jsonErr;
    return;
  }

  try {
    const resp = await fetch("/api/solve_with_allocation", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: bodyStr,
      signal: state.solveAbortController.signal,
    });
    const data = await resp.json();

    $("debug-output").textContent = JSON.stringify(data, null, 2);

    if (!data || data.success === false) {
      appendDebugLine("❌ Planner reported an error.");
      return;
    }

    // If this was a checkpoint replan, splice new trajectories with frozen prefixes
    if (isCheckpointReplan && state.checkpoint?.segments) {
      const newRoutes = data.routes || {};

      Object.entries(newRoutes).forEach(([did, routeData]) => {
        const seg = state.checkpoint.segments[did];
        if (!seg?.prefix || !routeData?.trajectory) return;

        const prefix = seg.prefix;
        const newSuffix = routeData.trajectory;

        // Splice: prefix + newSuffix (avoiding duplicate join point)
        let joined;
        if (prefix.length > 0 && newSuffix.length > 0) {
          const lastPrefixPt = prefix[prefix.length - 1];
          const firstSuffixPt = newSuffix[0];
          if (Math.abs(lastPrefixPt[0] - firstSuffixPt[0]) < 0.001 &&
              Math.abs(lastPrefixPt[1] - firstSuffixPt[1]) < 0.001) {
            joined = prefix.concat(newSuffix.slice(1));
          } else {
            joined = prefix.concat(newSuffix);
          }
        } else {
          joined = prefix.concat(newSuffix);
        }

        routeData.trajectory = joined;
        const firstPt = joined.length > 0 ? `(${joined[0][0].toFixed(1)},${joined[0][1].toFixed(1)})` : "N/A";
        const lastPt = joined.length > 0 ? `(${joined[joined.length-1][0].toFixed(1)},${joined[joined.length-1][1].toFixed(1)})` : "N/A";
        const prefixFirst = prefix.length > 0 ? `(${prefix[0][0].toFixed(1)},${prefix[0][1].toFixed(1)})` : "N/A";
        const prefixLast = prefix.length > 0 ? `(${prefix[prefix.length-1][0].toFixed(1)},${prefix[prefix.length-1][1].toFixed(1)})` : "N/A";
        appendDebugLine(`📎 D${did}: prefix=${prefix.length}pts [${prefixFirst}→${prefixLast}] + suffix=${newSuffix.length}pts = joined=${joined.length}pts [${firstPt}→${lastPt}]`);
      });

      data.routes = newRoutes;
      appendDebugLine("📎 Spliced new trajectories with frozen prefixes");

      // Store prefix distances so animation can start from the right position
      // We calculate the distance of each prefix so startAnimation knows where to begin
      state.checkpointReplanPrefixDistances = {};
      Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
        if (seg?.prefix && seg.prefix.length >= 2) {
          let prefixDist = 0;
          for (let i = 1; i < seg.prefix.length; i++) {
            const dx = seg.prefix[i][0] - seg.prefix[i - 1][0];
            const dy = seg.prefix[i][1] - seg.prefix[i - 1][1];
            prefixDist += Math.sqrt(dx * dx + dy * dy);
          }
          state.checkpointReplanPrefixDistances[did] = prefixDist;
          appendDebugLine(`📏 Drone ${did} prefix distance: ${prefixDist.toFixed(1)}`);
        }
      });
      console.log("checkpointReplanPrefixDistances:", state.checkpointReplanPrefixDistances);
    }

    // Store as draft solution (two-phase commit)
    // IMPORTANT: Deep copy routes to prevent modifications to state.routes from affecting the draft
    missionState.draftSolution = {
      sequences: JSON.parse(JSON.stringify(data.sequences || {})),
      routes: JSON.parse(JSON.stringify(data.routes || {})),
      wrappedPolygons: JSON.parse(JSON.stringify(data.wrapped_polygons || [])),
      allocations: JSON.parse(JSON.stringify(data.allocations || {})),
      allocationStrategy: data.allocation_strategy || null,
      distanceMatrix: data.distance_matrix ? JSON.parse(JSON.stringify(data.distance_matrix)) : null,
      isCheckpointReplan: isCheckpointReplan,
      checkpointSegments: isCheckpointReplan ? JSON.parse(JSON.stringify(state.checkpoint.segments)) : null,
    };

    // Apply the draft solution visually (but not committed yet)
    applyDraftSolutionToUI(missionState.draftSolution);

    // Clear checkpoint state after successful replan (now in draft)
    state.checkpoint = {
      active: false,
      pct: 0.5,
      segments: {},
    };

    // Transition to DRAFT_READY
    setMissionMode(MissionMode.DRAFT_READY, isCheckpointReplan ? "checkpoint replan ready" : "solution ready");

    const resultLabel = isCheckpointReplan ? "✅ Checkpoint replan ready for review." : "✅ Solution ready for review.";
    appendDebugLine(resultLabel);
  } catch (err) {
    if (err.name === 'AbortError') {
      appendDebugLine("⚠️ Solve request canceled.");
      $("debug-output").textContent = "Solve request was canceled.";
    } else {
      $("debug-output").textContent = "Error calling /api/solve_with_allocation: " + err;
      appendDebugLine("❌ Error calling /api/solve_with_allocation: " + err);
    }
  } finally {
    state.solveAbortController = null;
    // Re-enable the Run Planner button
    const btnRun = $("btn-run-planner");
    if (btnRun) {
      btnRun.disabled = false;
      btnRun.textContent = "Run Planner";
      btnRun.style.opacity = "1";
      btnRun.style.cursor = "pointer";
    }
    updateButtonStates();
  }
}

/**
 * Apply a draft solution to the UI for visualization
 * (without committing it to missionState.committedSegments)
 */
function applyDraftSolutionToUI(draft) {
  if (!draft) return;

  // Debug: Log what's in the draft routes before applying
  console.log("📥 applyDraftSolutionToUI called, isCheckpointReplan:", draft.isCheckpointReplan);
  Object.entries(draft.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
      console.log(`📥 draft.routes[${did}]: ${trajLen}pts ${firstPt}→${lastPt}`);
    }
  });

  // Deep copy to allow UI modifications without affecting the stored draft
  state.sequences = JSON.parse(JSON.stringify(draft.sequences || {}));
  state.routes = JSON.parse(JSON.stringify(draft.routes || {}));
  state.wrappedPolygons = JSON.parse(JSON.stringify(draft.wrappedPolygons || []));
  state.allocations = JSON.parse(JSON.stringify(draft.allocations || {}));
  state.allocationStrategy = draft.allocationStrategy || null;
  state.distanceMatrix = draft.distanceMatrix ? JSON.parse(JSON.stringify(draft.distanceMatrix)) : null;

  // Debug: Log what state.routes looks like after copy
  console.log("📥 state.routes after copy:");
  Object.entries(state.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
      console.log(`📥 state.routes[${did}]: ${trajLen}pts ${firstPt}→${lastPt}`);
    }
  });

  // After checkpoint replan, update animation drones to show at their new starting positions
  if (draft.isCheckpointReplan && state.animation.drones) {
    Object.keys(state.animation.drones).forEach((did) => {
      if (state.routes[did] && state.checkpointReplanPrefixDistances && state.checkpointReplanPrefixDistances[did]) {
        const routeInfo = state.routes[did];
        const trajectory = routeInfo.trajectory || [];

        // Recalculate cumulative distances for the new spliced trajectory
        let cumulativeDistances = [0];
        let totalDistance = 0;
        if (trajectory.length >= 2) {
          for (let i = 1; i < trajectory.length; i++) {
            const dx = trajectory[i][0] - trajectory[i - 1][0];
            const dy = trajectory[i][1] - trajectory[i - 1][1];
            totalDistance += Math.sqrt(dx * dx + dy * dy);
            cumulativeDistances.push(totalDistance);
          }
        }

        const prefixDist = state.checkpointReplanPrefixDistances[did];
        state.animation.drones[did] = {
          progress: totalDistance > 0 ? prefixDist / totalDistance : 0,
          distanceTraveled: prefixDist,
          animating: false,  // Not animating yet, just showing position
          cumulativeDistances,
          totalDistance,
        };
      }
    });
  }

  // Debug: Log what allocations we received
  console.log("🎯 draft.allocations:", draft.allocations);
  appendDebugLine("🎯 Allocations received: " + JSON.stringify(draft.allocations));

  // Display allocations in Env tab
  updateAllocationDisplay(state.allocations, state.allocationStrategy);

  const cur = state.currentDroneForSeq;
  const curSeq = state.sequences[cur] || "";
  const seqInput = $("sequence-input");
  if (seqInput) seqInput.value = curSeq;

  updateStatsFromRoutes();
  drawEnvironment();
}

// Cancel any running solve request
function cancelSolve() {
  if (state.solveAbortController) {
    state.solveAbortController.abort();
    appendDebugLine("⚠️ Canceling solve request...");
  } else {
    appendDebugLine("No solve request in progress.");
  }
}

// ----------------------------------------------------
// Initial environment load from backend (optional)
// ----------------------------------------------------
async function initialLoadEnv() {
  try {
    const resp = await fetch("/api/environment");
    if (!resp.ok) {
      appendDebugLine("No /api/environment available, start by importing JSON.");
      drawEnvironment();
      return;
    }
    const data = await resp.json();
    if (!data || data.success === false) {
      appendDebugLine("Backend /api/environment responded but not success.");
      drawEnvironment();
      return;
    }
    state.env = data.environment || null;
    state.envFilename = data.filename || null;
    if (state.envFilename) {
      $("env-filename").textContent = state.envFilename;
    }

    // Reset mission when loading a new environment from backend
    state.missionId = null;
    state.routes = {};
    state.sequences = {};
    state.trajectoryVisible = {};

    appendDebugLine("Loaded environment from backend.");
    initDroneConfigsFromEnv();

    // Fetch SAM wrapping if there are SAMs in the environment
    if (state.env && state.env.sams && state.env.sams.length > 0) {
      fetchWrappedPolygons(true).then(() => drawEnvironment());
    } else {
      drawEnvironment();
    }
  } catch (err) {
    appendDebugLine("Failed to fetch /api/environment: " + err);
    drawEnvironment();
  }
}

// ----------------------------------------------------
// Tab switching
// ----------------------------------------------------
function attachTabSwitching() {
  const tabBtns = document.querySelectorAll(".tab-btn");
  const tabPanes = document.querySelectorAll(".tab-pane");

  tabBtns.forEach((btn) => {
    btn.addEventListener("click", () => {
      const tabId = btn.getAttribute("data-tab");

      // Remove active from all buttons and panes
      tabBtns.forEach((b) => b.classList.remove("active"));
      tabPanes.forEach((p) => p.classList.remove("active"));

      // Add active to clicked button and corresponding pane
      btn.classList.add("active");
      const pane = document.getElementById(`tab-${tabId}`);
      if (pane) pane.classList.add("active");
    });
  });
}

// ----------------------------------------------------
// Trajectory visibility controls
// ----------------------------------------------------
function updateTrajectoryButtonStates() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`traj-d${did}`);
    if (btn) {
      if (state.trajectoryVisible[String(did)]) {
        btn.classList.add("active");
      } else {
        btn.classList.remove("active");
      }
    }
  }

  // Update "All" button - active if all are visible
  const allBtn = $("traj-all");
  if (allBtn) {
    const allVisible = Object.values(state.trajectoryVisible).every((v) => v);
    if (allVisible) {
      allBtn.classList.add("active");
    } else {
      allBtn.classList.remove("active");
    }
  }
}

function toggleTrajectory(did) {
  state.trajectoryVisible[did] = !state.trajectoryVisible[did];
  updateTrajectoryButtonStates();
  drawEnvironment();
  appendDebugLine(`Trajectory D${did}: ${state.trajectoryVisible[did] ? "shown" : "hidden"}`);
}

function toggleAllTrajectories() {
  const allVisible = Object.values(state.trajectoryVisible).every((v) => v);
  const newState = !allVisible;

  for (let did = 1; did <= 5; did++) {
    state.trajectoryVisible[String(did)] = newState;
  }

  updateTrajectoryButtonStates();
  drawEnvironment();
  appendDebugLine(`All trajectories: ${newState ? "shown" : "hidden"}`);
}

function attachTrajectoryControls() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`traj-d${did}`);
    if (btn) {
      btn.addEventListener("click", () => toggleTrajectory(String(did)));
    }
  }

  const allBtn = $("traj-all");
  if (allBtn) {
    allBtn.addEventListener("click", toggleAllTrajectories);
  }

  // Initialize button states
  updateTrajectoryButtonStates();
}

function freezeAtCheckpoint() {
  // Freeze drones at their CURRENT position (distanceTraveled), not at a percentage

  if (!state.animation || !state.animation.drones || Object.keys(state.animation.drones).length === 0) {
    appendDebugLine("No animation data to freeze (drones: " + JSON.stringify(state.animation?.drones) + ")");
    return;
  }

  // Ensure checkpoint container exists
  state.checkpoint = state.checkpoint || { active: false, pct: 0, segments: {} };
  state.checkpoint.active = true;
  state.checkpoint.segments = {};

  // Track which targets are visited in the current (frozen) routes
  const targetsVisitedThisSegment = [];

  Object.entries(state.animation.drones).forEach(([did, droneState]) => {
    const traj = state.routes[did]?.trajectory || [];
    if (traj.length < 2 || droneState.totalDistance <= 0) return;

    // Use the drone's CURRENT distanceTraveled (actual position during animation)
    const currentDist = droneState.distanceTraveled || 0;

    const { prefixPoints, suffixPoints, splitPoint } =
      split_polyline_at_distance(traj, currentDist);

    // Store the current position as splitPoint for drawing
    state.checkpoint.segments[did] = {
      prefix: prefixPoints,
      suffix: suffixPoints,
      splitPoint,
      checkpointDist: currentDist,
    };

    // Mark ONLY targets that have been passed (are in the prefix trajectory)
    // Targets ahead of the frozen position should remain available for replanning
    const route = state.routes[did]?.route || [];
    const env = state.env;

    route.forEach(wp => {
      if (!String(wp).startsWith("T")) return;
      if (state.visitedTargets.includes(wp)) return;

      // Find target position
      const target = env.targets?.find(t => t.id === wp);
      if (!target) return;

      // Check if this target position appears in the prefix trajectory
      // A target is visited if we've passed through or near its position
      const targetPos = [target.x, target.y];
      const isPassed = prefixPoints.some(pt => {
        const dist = Math.sqrt(
          Math.pow(pt[0] - targetPos[0], 2) +
          Math.pow(pt[1] - targetPos[1], 2)
        );
        return dist < 0.5; // Within 0.5 units means we visited it
      });

      if (isPassed) {
        targetsVisitedThisSegment.push(wp);
        state.visitedTargets.push(wp);
        console.log(`✓ D${did}: Marked ${wp} as visited (found in prefix trajectory)`);
      } else {
        console.log(`○ D${did}: ${wp} NOT visited (ahead of frozen position)`);
      }
    });

    // Stop animating but keep current position
    droneState.animating = false;

    // Store full trajectory and replace with prefix (traveled portion only)
    state.routes[did]._fullTrajectory = state.routes[did]._fullTrajectory || traj;
    state.routes[did].trajectory = prefixPoints;
  });

  if (targetsVisitedThisSegment.length > 0) {
    appendDebugLine(`🎯 Marked ${targetsVisitedThisSegment.length} targets as visited (in frozen trajectory): ${targetsVisitedThisSegment.join(', ')}`);
    appendDebugLine(`   These targets will show GREEN X and be excluded from replanning`);
  } else {
    appendDebugLine(`   No targets in frozen trajectories - all remain available for replanning`);
  }

  // Stop the RAF loop cleanly
  state.animation.active = false;
  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }

  const avgPct = Object.values(state.animation.drones)
    .filter(d => d.totalDistance > 0)
    .map(d => (d.distanceTraveled || 0) / d.totalDistance)
    .reduce((a, b, _, arr) => a + b / arr.length, 0) * 100;

  updateAnimationButtonStates();
  drawEnvironment();
  appendDebugLine(`Frozen at ${avgPct.toFixed(0)}% progress`);
}
console.log("freezeAtCheckpoint defined?", typeof freezeAtCheckpoint, "line marker A");

// ----------------------------------------------------
// Animation controls
// ----------------------------------------------------
function startAnimation(droneIds) {
  const perms = getUiPermissions();

  // Check if we can animate or resume
  if (!perms.canAnimate && !perms.canResume) {
    appendDebugLine("⚠️ Cannot animate in current state. Accept a solution first.");
    return;
  }

  // If we're resuming from pause, restore context
  if (perms.canResume && missionState.pauseContext) {
    resumeAnimation();
    return;
  }

  // Stop any existing animation
  stopAnimation();

  // Check if we're in a frozen checkpoint state WITHOUT a replan
  // If checkpoint is active (frozen) but no prefix distances exist, block animation
  const hasFrozenCheckpoint = state.checkpoint?.active ||
    Object.keys(state.checkpoint?.segments || {}).length > 0;
  const hasReplanData = state.checkpointReplanPrefixDistances &&
    Object.keys(state.checkpointReplanPrefixDistances).length > 0;

  if (hasFrozenCheckpoint && !hasReplanData) {
    appendDebugLine("⚠️ Cannot animate: checkpoint is frozen. Run Planner to replan from checkpoint, or press R to reset.");
    return;
  }

  // Clear checkpoint state so frozen drones don't persist
  if (state.checkpoint) {
    state.checkpoint.active = false;
    state.checkpoint.segments = {};
  }

  // Restore full trajectories if they were truncated by a previous freeze
  // BUT NOT if this is a checkpoint replan (we want to keep the spliced trajectory)
  Object.keys(state.routes || {}).forEach((did) => {
    if (state.routes[did]._fullTrajectory) {
      console.log(`⚠️ startAnimation D${did}: _fullTrajectory exists (${state.routes[did]._fullTrajectory.length} pts), restoring...`);
      state.routes[did].trajectory = state.routes[did]._fullTrajectory;
      delete state.routes[did]._fullTrajectory;
    }
  });

  // Debug: Log trajectories at animation start
  console.log("🎬 startAnimation: trajectories before animation:");
  Object.entries(state.routes || {}).forEach(([did, info]) => {
    const trajLen = info.trajectory ? info.trajectory.length : 0;
    if (trajLen > 0) {
      const firstPt = `(${info.trajectory[0][0].toFixed(1)},${info.trajectory[0][1].toFixed(1)})`;
      const lastPt = `(${info.trajectory[trajLen-1][0].toFixed(1)},${info.trajectory[trajLen-1][1].toFixed(1)})`;
      console.log(`🎬 state.routes[${did}]: ${trajLen}pts ${firstPt}→${lastPt}`);
    }
  });

  // Initialize animation state for selected drones
  state.animation.active = true;
  state.animation.drones = {};

  // Enable debug stop at splice point if this is a checkpoint replan
  const isCheckpointReplan = state.checkpointReplanPrefixDistances &&
    Object.keys(state.checkpointReplanPrefixDistances).length > 0;
  state.debugStopAtSplice = isCheckpointReplan;  // Set true to stop at splice point

  // Clear segment switch flags at animation start
  state._segmentSwitchPending = false;
  state._nextSegmentIndex = null;

  // Debug: Log visited targets at animation start
  appendDebugLine(`🎬 Animation starting. Visited targets: [${state.visitedTargets.join(", ")}]`);
  appendDebugLine(`🎬 checkpointReplanPrefixDistances: ${JSON.stringify(state.checkpointReplanPrefixDistances)}`);
  appendDebugLine(`🎬 Segments: ${missionState.committedSegments.length}, currentSegmentIndex: ${missionState.currentSegmentIndex}`);
  if (isCheckpointReplan) {
    appendDebugLine(`🛑 DEBUG: Will stop at splice point (prefix distance)`);
  }

  droneIds.forEach((did) => {
    const routeInfo = state.routes[did];

    // Debug: Log why a drone might be skipped
    if (!routeInfo) {
      appendDebugLine(`⚠️ D${did} skipped: no routeInfo`);
      return;
    }

    // Debug: Log trajectory info at animation start
    const trajLen = routeInfo.trajectory ? routeInfo.trajectory.length : 0;
    const routeStr = (routeInfo.route || []).join("-");
    const firstPt = trajLen > 0 ? `(${routeInfo.trajectory[0][0].toFixed(1)},${routeInfo.trajectory[0][1].toFixed(1)})` : "N/A";
    const lastPt = trajLen > 0 ? `(${routeInfo.trajectory[trajLen-1][0].toFixed(1)},${routeInfo.trajectory[trajLen-1][1].toFixed(1)})` : "N/A";
    appendDebugLine(`🎬 D${did}: ${trajLen} pts, route: ${routeStr}, from ${firstPt} to ${lastPt}`);
    if (!routeInfo.route) {
      appendDebugLine(`⚠️ D${did} skipped: no route array`);
      return;
    }
    if (routeInfo.route.length < 2) {
      appendDebugLine(`⚠️ D${did} skipped: route too short (${routeInfo.route.length} waypoints)`);
      return;
    }

    // Pre-calculate cumulative distances for uniform speed animation
    const trajectory = routeInfo.trajectory || [];
    let cumulativeDistances = [0];
    let totalDistance = 0;

    if (trajectory.length >= 2) {
      for (let i = 1; i < trajectory.length; i++) {
        const dx = trajectory[i][0] - trajectory[i - 1][0];
        const dy = trajectory[i][1] - trajectory[i - 1][1];
        totalDistance += Math.sqrt(dx * dx + dy * dy);
        cumulativeDistances.push(totalDistance);
      }
    }

    if (totalDistance <= 0) {
      appendDebugLine(`⚠️ D${did} skipped: totalDistance is ${totalDistance} (trajectory has ${trajectory.length} points)`);
      return;
    }

    // Check if this is a checkpoint replan - start from prefix distance if so
    let initialDistance = 0;
    if (state.checkpointReplanPrefixDistances && state.checkpointReplanPrefixDistances[did]) {
      initialDistance = state.checkpointReplanPrefixDistances[did];
      appendDebugLine(`🎬 Drone ${did} starting animation from distance ${initialDistance.toFixed(1)} (checkpoint replan)`);
    }

    state.animation.drones[did] = {
      progress: totalDistance > 0 ? initialDistance / totalDistance : 0,
      distanceTraveled: initialDistance,
      animating: true,
      cumulativeDistances,
      totalDistance,
    };

    // TEMP TEST: verify split utility
    const traj = (state.routes[did] && state.routes[did].trajectory) ? state.routes[did].trajectory : [];
    if (traj.length >= 2) {
      const total = state.animation.drones[did].totalDistance;
      const checkpointDist = 0.5 * total;

      const res = split_polyline_at_distance(traj, checkpointDist);
      console.log("SPLIT TEST", did, {
        total,
        checkpointDist,
        splitPoint: res.splitPoint,
        prefixLen: res.prefixPoints.length,
        suffixLen: res.suffixPoints.length,
        splitIndex: res.splitIndex,
        t: res.t,
      });
    }

    // Make sure trajectory is visible for animated drones
    state.trajectoryVisible[did] = true;
  });

  // Clear checkpoint replan prefix distances after using them
  // (so next animation without replan starts from 0)
  state.checkpointReplanPrefixDistances = null;

  updateTrajectoryButtonStates();
  updateAnimationButtonStates();

  // Check if any drones were added for animation
  const activeDrones = Object.keys(state.animation.drones);
  if (activeDrones.length === 0) {
    appendDebugLine("No valid drones to animate. Check route data.");
    state.animation.active = false;
    return;
  }

  // Start animation loop
  const speedUnitsPerSec = 20; // tune this (map units per second)
  let lastTime = null;

  function animate(currentTime) {
    // Check if animation was stopped/paused externally
    if (!state.animation.active) {
      return;
    }

    if (lastTime === null) lastTime = currentTime;

    const deltaTime = Math.min(currentTime - lastTime, 100);
    lastTime = currentTime;

    const dt = deltaTime / 1000; // seconds
    let anyAnimating = false;

    Object.entries(state.animation.drones).forEach(([did, droneState]) => {
      if (!droneState.animating) return;

      // 1) advance by distance (single source of truth)
      droneState.distanceTraveled += speedUnitsPerSec * dt;

      // 2) clamp and stop if finished
      if (droneState.distanceTraveled >= droneState.totalDistance) {
        droneState.distanceTraveled = droneState.totalDistance;
        droneState.animating = false;
      } else {
        anyAnimating = true;
      }

      // MISSION REPLAY: Check if we've reached the C point (segment boundary)
      // and need to switch to the next segment
      if (!state._segmentSwitchPending) {
        const currentSegIdx = missionState.currentSegmentIndex;
        const nextSegIdx = currentSegIdx + 1;
        const nextSegment = missionState.committedSegments[nextSegIdx];

        // If there's a next segment with prefix distances, check if we've reached the C point
        if (nextSegment && nextSegment.prefixDistances && nextSegment.prefixDistances[did]) {
          const cPointDist = nextSegment.prefixDistances[did];

          if (droneState.distanceTraveled >= cPointDist && !droneState._passedCPoint) {
            droneState._passedCPoint = true;
            console.log(`🔄 D${did} reached C point at distance ${cPointDist.toFixed(1)} - triggering segment switch`);
            appendDebugLine(`🔄 D${did} reached C point - switching to segment ${nextSegIdx}`);

            // Mark that we need to switch segments (do it once, not per drone)
            state._segmentSwitchPending = true;
            state._nextSegmentIndex = nextSegIdx;
          }
        }
      }

      // DEBUG: For checkpoint replan, stop at the splice point (prefix distance)
      // This allows verification that the splice is correct
      if (state.debugStopAtSplice && state.checkpointReplanPrefixDistances) {
        const prefixDist = state.checkpointReplanPrefixDistances[did];
        if (prefixDist !== undefined && droneState.distanceTraveled >= prefixDist && !droneState._passedSplicePoint) {
          droneState._passedSplicePoint = true;
          droneState.distanceTraveled = prefixDist;
          droneState.animating = false;
          console.log(`🛑 D${did} stopped at splice point (prefixDist=${prefixDist.toFixed(1)})`);
          appendDebugLine(`🛑 D${did} stopped at splice point`);
        }
      }

      // 3) derive progress (for existing drawing/UI code)
      droneState.progress = (droneState.totalDistance > 0)
        ? (droneState.distanceTraveled / droneState.totalDistance)
        : 1;

      // 4) Check if drone has passed any target waypoints and mark them as visited
      const routeInfo = state.routes[did];
      if (routeInfo && routeInfo.route && routeInfo.trajectory) {
        const trajectory = routeInfo.trajectory;
        const route = routeInfo.route;
        const cumulativeDistances = droneState.cumulativeDistances || [];
        const currentDistance = droneState.distanceTraveled;

        // Debug: Log once per drone at start of animation
        if (droneState.distanceTraveled < 1 && !droneState._debugLogged) {
          droneState._debugLogged = true;
          const targets = route.filter(wp => String(wp).startsWith("T"));
          const envTargetIds = (state.env.targets || []).map(t => t.id).join(", ");
          console.log(`🔍 D${did} route targets: ${targets.join(", ")}, trajectory points: ${trajectory.length}`);
          console.log(`🔍 D${did} env targets: [${envTargetIds}]`);
          appendDebugLine(`🔍 D${did} route targets: [${targets.join(", ")}]`);
        }

        // For each target in the route, check if we've passed its position
        route.forEach((wp, wpIdx) => {
          if (!String(wp).startsWith("T")) return;  // Only targets
          if (state.visitedTargets.includes(wp)) return;  // Already visited

          // Find the target's position in the environment
          const target = state.env.targets?.find(t => t.id === wp);
          if (!target) {
            // Debug: target not found in env
            if (!droneState[`_debugNoTarget_${wp}`]) {
              droneState[`_debugNoTarget_${wp}`] = true;
              console.log(`⚠️ D${did} target ${wp} not found in state.env.targets`);
            }
            return;
          }

          // Check if we've passed close to this target's position
          // Find the trajectory point closest to the target
          let minDist = Infinity;
          let closestIdx = -1;
          trajectory.forEach((pt, idx) => {
            const dist = Math.sqrt(Math.pow(pt[0] - target.x, 2) + Math.pow(pt[1] - target.y, 2));
            if (dist < minDist) {
              minDist = dist;
              closestIdx = idx;
            }
          });

          // Debug: Log closest distance once per target
          if (!droneState[`_debugDist_${wp}`]) {
            droneState[`_debugDist_${wp}`] = true;
            console.log(`📏 D${did} target ${wp} at (${target.x.toFixed(1)}, ${target.y.toFixed(1)}), closest traj point idx=${closestIdx}, minDist=${minDist.toFixed(2)}`);
          }

          // If we found a close point and we've traveled past it, mark target as visited
          // Increased threshold from 1.0 to 20.0 to account for trajectory smoothing
          if (closestIdx >= 0 && minDist < 20.0 && cumulativeDistances[closestIdx] !== undefined) {
            const targetDistance = cumulativeDistances[closestIdx];
            if (currentDistance >= targetDistance) {
              state.visitedTargets.push(wp);
              console.log(`🎯 D${did} passed target ${wp} at distance ${targetDistance.toFixed(1)} (minDist=${minDist.toFixed(2)})`);
              appendDebugLine(`🎯 D${did} visited ${wp}`);
            }
          } else if (!droneState[`_debugSkip_${wp}`] && closestIdx >= 0) {
            // Debug: Log why target was skipped (only once)
            droneState[`_debugSkip_${wp}`] = true;
            if (minDist >= 20.0) {
              console.log(`⚠️ D${did} target ${wp} skipped: minDist=${minDist.toFixed(2)} >= 20.0`);
            }
          }
        });
      }
    });

    // MISSION REPLAY: Perform segment switch if triggered
    if (state._segmentSwitchPending) {
      state._segmentSwitchPending = false;
      const nextSegIdx = state._nextSegmentIndex;
      const nextSegment = missionState.committedSegments[nextSegIdx];

      if (nextSegment) {
        console.log(`🔄 Switching to segment ${nextSegIdx}`);
        appendDebugLine(`🔄 === SEGMENT SWITCH to ${nextSegIdx} ===`);

        // Update current segment index
        missionState.currentSegmentIndex = nextSegIdx;

        // Switch environment (shows new targets)
        if (nextSegment.env) {
          state.env = JSON.parse(JSON.stringify(nextSegment.env));
          appendDebugLine(`🔄 Environment updated with ${state.env.targets?.length || 0} targets`);
        }

        // Switch trajectories to the spliced versions from this segment
        if (nextSegment.solution && nextSegment.solution.routes) {
          Object.entries(nextSegment.solution.routes).forEach(([did, routeData]) => {
            if (state.routes[did] && routeData.trajectory) {
              const oldLen = state.routes[did].trajectory?.length || 0;
              state.routes[did].trajectory = JSON.parse(JSON.stringify(routeData.trajectory));
              state.routes[did].route = JSON.parse(JSON.stringify(routeData.route || []));
              const newLen = state.routes[did].trajectory.length;
              appendDebugLine(`🔄 D${did}: trajectory ${oldLen}→${newLen} pts`);

              // Update drone's cumulative distances and total distance for the new trajectory
              const droneState = state.animation.drones[did];
              if (droneState) {
                const trajectory = state.routes[did].trajectory;
                let cumulativeDistances = [0];
                let totalDistance = 0;
                for (let i = 1; i < trajectory.length; i++) {
                  const dx = trajectory[i][0] - trajectory[i - 1][0];
                  const dy = trajectory[i][1] - trajectory[i - 1][1];
                  totalDistance += Math.sqrt(dx * dx + dy * dy);
                  cumulativeDistances.push(totalDistance);
                }
                droneState.cumulativeDistances = cumulativeDistances;
                droneState.totalDistance = totalDistance;
                // Keep distanceTraveled as-is (we're continuing from C point)
                appendDebugLine(`🔄 D${did}: totalDistance now ${totalDistance.toFixed(1)}`);
              }
            }
          });
        }

        appendDebugLine(`🔄 === SEGMENT SWITCH COMPLETE ===`);
      }
    }

    drawEnvironment();

    if (anyAnimating && state.animation.active) {
      state.animation.animationId = requestAnimationFrame(animate);
    } else if (!state.animation.active) {
      // Stopped externally (pause/cut) - don't change mode
      return;
    } else {
      state.animation.active = false;
      // Animation completed naturally - go to READY_TO_ANIMATE
      setMissionMode(MissionMode.READY_TO_ANIMATE, "animation complete");
      updateAnimationButtonStates();
    }
  }

  state.animation.animationId = requestAnimationFrame(animate);

  // Transition to ANIMATING state
  setMissionMode(MissionMode.ANIMATING, "animation started");

  appendDebugLine(`Started animation for: ${droneIds.map((d) => `D${d}`).join(", ")}`);
}

function stopAnimation() {
  const wasAnimating = state.animation.active;

  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }

  state.animation.active = false;
  state.animation.drones = {};
  missionState.pauseContext = null;

  // If we were animating, transition back to READY_TO_ANIMATE
  if (wasAnimating && missionState.mode === MissionMode.ANIMATING) {
    setMissionMode(MissionMode.READY_TO_ANIMATE, "animation stopped");
  }

  updateAnimationButtonStates();
  drawEnvironment();
  appendDebugLine("Animation stopped.");
}

/**
 * Pause animation - save state for resuming
 */
function pauseAnimation() {
  const perms = getUiPermissions();
  if (!perms.canPause) {
    appendDebugLine("Cannot pause in current state");
    return;
  }

  // Save the current animation state for resume
  missionState.pauseContext = {
    droneStates: JSON.parse(JSON.stringify(state.animation.drones)),
    animatingDrones: Object.keys(state.animation.drones).filter(
      did => state.animation.drones[did].animating
    ),
  };

  // Stop the animation frame loop but keep drone positions
  if (state.animation.animationId) {
    cancelAnimationFrame(state.animation.animationId);
    state.animation.animationId = null;
  }
  state.animation.active = false;

  // Transition to PAUSED state
  setMissionMode(MissionMode.PAUSED_MID_ANIMATION, "animation paused");

  updateAnimationButtonStates();
  drawEnvironment();
  appendDebugLine("Animation paused. Click Animate to resume.");
}

/**
 * Resume animation from pause state
 */
function resumeAnimation() {
  if (!missionState.pauseContext) {
    appendDebugLine("No pause context to resume from");
    return;
  }

  // Restore animation state
  state.animation.drones = missionState.pauseContext.droneStates;
  state.animation.active = true;

  // Clear pause context
  missionState.pauseContext = null;

  // Restart animation loop
  const speedUnitsPerSec = 20;
  let lastTime = null;

  function animate(currentTime) {
    if (lastTime === null) lastTime = currentTime;

    const deltaTime = Math.min(currentTime - lastTime, 100);
    lastTime = currentTime;

    const dt = deltaTime / 1000;
    let anyAnimating = false;

    Object.entries(state.animation.drones).forEach(([did, droneState]) => {
      if (!droneState.animating) return;

      droneState.distanceTraveled += speedUnitsPerSec * dt;

      if (droneState.distanceTraveled >= droneState.totalDistance) {
        droneState.distanceTraveled = droneState.totalDistance;
        droneState.animating = false;
      } else {
        anyAnimating = true;
      }

      droneState.progress = (droneState.totalDistance > 0)
        ? (droneState.distanceTraveled / droneState.totalDistance)
        : 1;
    });

    drawEnvironment();

    if (anyAnimating) {
      state.animation.animationId = requestAnimationFrame(animate);
    } else {
      state.animation.active = false;
      // Animation completed naturally - go to READY_TO_ANIMATE
      setMissionMode(MissionMode.READY_TO_ANIMATE, "animation complete");
      updateAnimationButtonStates();
    }
  }

  state.animation.animationId = requestAnimationFrame(animate);

  // Transition to ANIMATING state
  setMissionMode(MissionMode.ANIMATING, "animation resumed");

  updateAnimationButtonStates();
  appendDebugLine("Animation resumed.");
}

/**
 * Cut/Checkpoint - freeze current position and truncate future
 */
function cutAtCheckpoint() {
  const perms = getUiPermissions();
  if (!perms.canCut) {
    appendDebugLine("Cannot cut in current state");
    return;
  }

  appendDebugLine(`cutAtCheckpoint called. animation.drones keys: ${Object.keys(state.animation.drones || {}).join(", ") || "NONE"}`);
  appendDebugLine(`animation.active: ${state.animation.active}`);

  // Use the existing freezeAtCheckpoint logic
  freezeAtCheckpoint();

  // Verify checkpoint was set up
  if (!state.checkpoint?.active) {
    appendDebugLine("⚠️ WARNING: freezeAtCheckpoint did not set checkpoint.active! Check drone data.");
  } else {
    appendDebugLine(`✅ Checkpoint set with ${Object.keys(state.checkpoint.segments || {}).length} drone segments`);
  }

  // Transition to CHECKPOINT state
  setMissionMode(MissionMode.CHECKPOINT, "cut at checkpoint");

  // Always redraw to show visited targets with green X
  drawEnvironment();

  // Debug: Log visited targets
  appendDebugLine(`📍 Visited targets after cut: [${state.visitedTargets.join(", ")}]`);
  appendDebugLine("Cut at checkpoint. Edit environment and Run Planner to continue.");
}

function updateAnimationButtonStates() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`anim-d${did}`);
    if (btn) {
      const droneState = state.animation.drones[String(did)];
      if (droneState && droneState.animating) {
        btn.classList.add("active");
      } else {
        btn.classList.remove("active");
      }
    }
  }

  const allBtn = $("anim-all");
  if (allBtn) {
    if (state.animation.active && Object.keys(state.animation.drones).length === 5) {
      allBtn.classList.add("active");
    } else {
      allBtn.classList.remove("active");
    }
  }
}

function attachAnimationControls() {
  for (let did = 1; did <= 5; did++) {
    const btn = $(`anim-d${did}`);
    if (btn) {
      btn.addEventListener("click", () => {
        startAnimation([String(did)]);
      });
    }
  }

  const allBtn = $("anim-all");
  if (allBtn) {
    allBtn.addEventListener("click", () => {
      const enabledDrones = [];
      for (let did = 1; did <= 5; did++) {
        const routeInfo = state.routes[String(did)];
        if (routeInfo && routeInfo.route && routeInfo.route.length >= 2) {
          enabledDrones.push(String(did));
        }
      }
      if (enabledDrones.length > 0) {
        startAnimation(enabledDrones);
      } else {
        appendDebugLine("No routes to animate. Run planner first.");
      }
    });
  }

  const stopBtn = $("anim-stop");
  if (stopBtn) {
    stopBtn.addEventListener("click", stopAnimation);
  }
}
// ----------------------------------------------------
// Replan from checkpoint
// ----------------------------------------------------
async function replanFromCheckpoint() {
  if (!state.missionId) {
    appendDebugLine("No missionId. Run planner first.");
    return;
  }
  if (!state.checkpoint?.active) {
    appendDebugLine("No active checkpoint. Press C to freeze first.");
    return;
  }

  // Build env2: copy current env, then (for now) only change drone starts to splitPoint.
  // (We’ll exclude visited targets next; this gets the pipeline working end-to-end.)
  const env2 = JSON.parse(JSON.stringify(state.env));

  // Add synthetic start nodes per drone based on checkpoint splitPoint
  // and update drone configs to start from those nodes.
  env2.synthetic_starts = env2.synthetic_starts || {};
  const newDroneConfigs = JSON.parse(JSON.stringify(state.droneConfigs));

  Object.entries(state.checkpoint.segments).forEach(([did, seg]) => {
    if (!seg?.splitPoint) return;

    const nodeId = `D${did}_START`;
    env2.synthetic_starts[nodeId] = { x: seg.splitPoint[0], y: seg.splitPoint[1] };

    // Update drone config to start at synthetic node
    if (newDroneConfigs[did]) {
      newDroneConfigs[did].start_airport = nodeId; // naming consistent with existing schema
    }
  });

  appendDebugLine("Requesting replanned suffix from checkpoint...");

  // Call backend (new endpoint)
  const resp = await fetch("/api/mission/replan_checkpoint", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      mission_id: state.missionId,
      checkpoint_pct: state.checkpoint.pct,
      env2,
      drone_configs: newDroneConfigs,
    }),
  });

  if (!resp.ok) {
    const text = await resp.text();
    console.error("Replan failed:", text);
    appendDebugLine("Replan failed. See console.");
    return;
  }

  const data = await resp.json();

  // Expected: data.routes like your normal solver response:
  // { "1": { trajectory: [[x,y],...], route:[...]} , ... }
  if (!data.routes) {
    console.error("Replan response missing routes:", data);
    appendDebugLine("Replan response invalid (missing routes).");
    return;
  }

  // Splice: finalTrajectory = prefix + newSuffixTrajectory (skipping duplicate split point)
  Object.entries(data.routes).forEach(([did, r]) => {
    const seg = state.checkpoint.segments[did];
    if (!seg?.prefix || !r?.trajectory || r.trajectory.length < 1) return;

    const prefix = seg.prefix;
    const newSuffix = r.trajectory;

    // Avoid duplicating the join point if the suffix starts at the split
    const joined =
      (prefix.length > 0 &&
       newSuffix.length > 0 &&
       prefix[prefix.length - 1][0] === newSuffix[0][0] &&
       prefix[prefix.length - 1][1] === newSuffix[0][1])
        ? prefix.concat(newSuffix.slice(1))
        : prefix.concat(newSuffix);

    // Install joined route back into UI state
    state.routes[did].trajectory = joined;

    // If you also display route labels, keep the new route (or splice labels later)
    if (r.route) state.routes[did].route = r.route;
  });

  // After replanning, clear checkpoint active (optional) and redraw
  state.checkpoint.active = false;

  drawEnvironment();
  appendDebugLine("Replan complete: suffix replaced and spliced.");
}

// ----------------------------------------------------
// Boot
// ----------------------------------------------------
window.addEventListener("load", () => {
  resizeCanvasToContainer();
  window.addEventListener("resize", resizeCanvasToContainer);

  attachCanvasEditing();
  attachConfigListeners();
  attachSequenceBar();
  attachIOHandlers();
  attachOptimizationHandlers();
  attachEditToggle();
  attachTabSwitching();
  attachTrajectoryControls();
  attachAnimationControls();

  const btnRun = $("btn-run-planner");
  if (btnRun) btnRun.addEventListener("click", runPlanner);

  const btnCancel = $("btn-cancel-solve");
  if (btnCancel) btnCancel.addEventListener("click", cancelSolve);

  // Add buttons (respect editMode)
  const btnAddA = $("btn-add-airport");
  const btnAddT = $("btn-add-target");
  const btnAddS = $("btn-add-sam");
  const btnDel  = $("btn-delete-selected");

  if (btnAddA) {
    btnAddA.addEventListener("click", () => {
      if (!state.editMode) return;
      setAddMode("airport");
    });
  }
  if (btnAddT) {
    btnAddT.addEventListener("click", () => {
      if (!state.editMode) return;
      setAddMode("target");
    });
  }
  if (btnAddS) {
    btnAddS.addEventListener("click", () => {
      if (!state.editMode) return;
      setAddMode("sam");
    });
  }
  if (btnDel) {
    btnDel.addEventListener("click", () => {
      if (!state.editMode) return;
      deleteSelected();
    });
  }

  // Status banner action buttons
  const btnAcceptEdits = $("btn-accept-edits");
  if (btnAcceptEdits) {
    btnAcceptEdits.addEventListener("click", acceptEdits);
  }

  const btnAcceptSolution = $("btn-accept-solution");
  if (btnAcceptSolution) {
    btnAcceptSolution.addEventListener("click", acceptSolution);
  }

  const btnDiscardDraft = $("btn-discard-draft");
  if (btnDiscardDraft) {
    btnDiscardDraft.addEventListener("click", discardDraftSolution);
  }

  // Agent Send button
  attachAgentHandlers();

  // Initialize status banner
  updateStatusBanner();
  updateButtonStates();

  initialLoadEnv();
});

// ----------------------------------------------------
// Agent Communication
// ----------------------------------------------------
function attachAgentHandlers() {
  const btnSend = $("btn-send-agent");
  const inputEl = $("agent-input");

  if (!btnSend || !inputEl) return;

  // Send on button click
  btnSend.addEventListener("click", () => sendAgentMessage());

  // Send on Enter (Shift+Enter for new line)
  inputEl.addEventListener("keydown", (e) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendAgentMessage();
    }
  });

  // Memory management handlers
  attachMemoryHandlers();

  // Load memories on init
  loadAgentMemories();
}

// ----------------------------------------------------
// Agent Memory Management
// ----------------------------------------------------
function attachMemoryHandlers() {
  const btnToggle = $("btn-toggle-memory");
  const btnClear = $("btn-clear-memory");
  const btnAdd = $("btn-add-memory");
  const memoryInput = $("memory-input");

  if (btnToggle) {
    btnToggle.addEventListener("click", () => {
      const panel = $("memory-panel");
      if (panel) {
        const isHidden = panel.style.display === "none";
        panel.style.display = isHidden ? "block" : "none";
        btnToggle.textContent = isHidden ? "Hide" : "Show";
      }
    });
  }

  if (btnClear) {
    btnClear.addEventListener("click", async () => {
      if (!confirm("Clear all agent memories?")) return;
      try {
        const resp = await fetch("/api/agent/memory", { method: "DELETE" });
        const data = await resp.json();
        if (data.success) {
          loadAgentMemories();
          appendDebugLine("Cleared all agent memories");
        }
      } catch (e) {
        console.error("Error clearing memories:", e);
      }
    });
  }

  if (btnAdd && memoryInput) {
    btnAdd.addEventListener("click", () => addAgentMemory());
    memoryInput.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        addAgentMemory();
      }
    });
  }
}

async function loadAgentMemories() {
  try {
    const resp = await fetch("/api/agent/memory");
    const data = await resp.json();
    if (data.success) {
      renderMemoryList(data.memories);
      const countEl = $("memory-count");
      if (countEl) countEl.textContent = data.count;
    }
  } catch (e) {
    console.error("Error loading memories:", e);
  }
}

function renderMemoryList(memories) {
  const listEl = $("memory-list");
  if (!listEl) return;

  if (!memories || memories.length === 0) {
    listEl.innerHTML = '<div style="color: #6b7280; font-size: 0.7rem; text-align: center; padding: 0.5rem;">No memories yet. Add instructions the agent should remember.</div>';
    return;
  }

  listEl.innerHTML = memories.map(m => `
    <div class="memory-item" data-id="${m.id}">
      <span class="memory-category">${m.category}</span>
      <span class="memory-content">${escapeHtml(m.content)}</span>
      <button class="btn-delete-memory" onclick="deleteAgentMemory(${m.id})">×</button>
    </div>
  `).join("");
}

async function addAgentMemory() {
  const inputEl = $("memory-input");
  const categoryEl = $("memory-category");
  if (!inputEl || !categoryEl) return;

  const content = inputEl.value.trim();
  if (!content) return;

  const category = categoryEl.value;

  try {
    const resp = await fetch("/api/agent/memory", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ content, category })
    });
    const data = await resp.json();
    if (data.success) {
      inputEl.value = "";
      loadAgentMemories();
      appendDebugLine(`Added memory: [${category}] ${content}`);
    }
  } catch (e) {
    console.error("Error adding memory:", e);
  }
}

async function deleteAgentMemory(memoryId) {
  try {
    const resp = await fetch(`/api/agent/memory/${memoryId}`, { method: "DELETE" });
    const data = await resp.json();
    if (data.success) {
      loadAgentMemories();
      appendDebugLine(`Deleted memory #${memoryId}`);
    }
  } catch (e) {
    console.error("Error deleting memory:", e);
  }
}

async function sendAgentMessage() {
  const inputEl = $("agent-input");
  const chatHistory = $("agent-chat-history");
  const btnSend = $("btn-send-agent");

  if (!inputEl || !chatHistory) return;

  const message = inputEl.value.trim();
  if (!message) return;

  // Create a Q&A block container
  const qaBlock = createQABlock(message);
  chatHistory.appendChild(qaBlock);
  inputEl.value = "";

  // Scroll to show the new block
  chatHistory.scrollTop = chatHistory.scrollHeight;

  // Disable button while processing
  if (btnSend) btnSend.disabled = true;

  try {
    const response = await fetch("/api/agents/chat-v4", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        message: message,
        env: state.env,
        sequences: state.sequences,
        drone_configs: state.droneConfigs,
        mission_id: state.missionId,   // <-- NEW
      }),
    });

    const data = await response.json();

    // NEW: store mission_id returned by backend, if any
    if (data.mission_id) {
      state.missionId = data.mission_id;
      appendDebugLine("Current mission_id: " + state.missionId);
    }

    // Debug: log the response to see what we're getting
    console.log("Agent response:", data);
    appendDebugLine("Agent response keys: " + Object.keys(data).join(", "));
    if (data.routes) {
      appendDebugLine("Agent routes: " + JSON.stringify(data.routes));
    }
    
    // Update the Q&A block with the response
    const responseArea = qaBlock.querySelector(".qa-response");
    if (data.reply) {
      let replyContent = data.reply;

      // Show route badges for multi-drone routes
      if (data.routes && Object.keys(data.routes).length > 0) {
        replyContent += `\n\n`;
        for (const [did, route] of Object.entries(data.routes)) {
          if (route && route.length > 0) {
            replyContent += `<span class="agent-route-badge">D${did}: ${route.join(" → ")}</span> `;
          }
        }
      } else if (data.route && data.route.length > 0) {
        // Legacy single route
        replyContent += `\n\n<span class="agent-route-badge">Route: ${data.route.join(" → ")}</span>`;
      }
      responseArea.innerHTML = replyContent.replace(/\n/g, "<br>");
      responseArea.classList.remove("thinking");

      // If agent returned routes, apply them to the planner
      if (data.routes && Object.keys(data.routes).length > 0) {
        applyAgentRoutes(data.routes, data.trajectories, data.points, data.fuel);
      } else if (data.route && data.route.length > 0) {
        // Legacy single route (D1 only)
        applyAgentRoute(data.route, data.points, data.fuel);
      }
    } else {
      responseArea.innerHTML = "No response from agent.";
      responseArea.classList.add("error");
    }
  } catch (err) {
    const responseArea = qaBlock.querySelector(".qa-response");
    responseArea.innerHTML = `Error: ${escapeHtml(err.message)}`;
    responseArea.classList.add("error");
    console.error("Agent error:", err);
  } finally {
    if (btnSend) btnSend.disabled = false;
    chatHistory.scrollTop = chatHistory.scrollHeight;
  }
}

function createQABlock(question) {
  const timestamp = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  const block = document.createElement("div");
  block.className = "qa-block";
  block.innerHTML = `
    <div class="qa-question">
      <span class="qa-label">YOU [${timestamp}]</span>
      <span class="qa-text">${escapeHtml(question)}</span>
    </div>
    <div class="qa-response-wrapper">
      <div class="qa-response-label">AGENT RESPONSE</div>
      <div class="qa-response thinking">
        🤔 Thinking...
      </div>
    </div>
  `;
  return block;
}

function appendChatMessage(content, type) {
  // Legacy function - kept for system messages
  const chatHistory = $("agent-chat-history");
  if (!chatHistory) return null;

  const msgId = "msg-" + Date.now() + "-" + Math.random().toString(36).substring(2, 7);
  const msgDiv = document.createElement("div");
  msgDiv.id = msgId;
  msgDiv.className = "agent-message system-message";
  msgDiv.innerHTML = content;

  chatHistory.appendChild(msgDiv);
  chatHistory.scrollTop = chatHistory.scrollHeight;
  return msgId;
}

function removeChatMessage(msgId) {
  if (!msgId) return;
  const el = document.getElementById(msgId);
  if (el) el.remove();
}

function escapeHtml(text) {
  const div = document.createElement("div");
  div.textContent = text;
  return div.innerHTML;
}

function applyAgentRoutes(routes, trajectories, totalPoints, totalFuel) {
  // Apply multi-drone routes from the agent
  // routes is like {"1": ["A1", "T3", "A1"], "2": ["A2", "T5", "A2"], ...}
  // trajectories is like {"1": [[x1,y1], [x2,y2], ...], ...} - SAM-avoiding paths

  console.log("🎯 applyAgentRoutes called with:", { routes, trajectories, totalPoints, totalFuel });
  appendDebugLine(`🎯 applyAgentRoutes: ${Object.keys(routes || {}).length} routes`);

  if (!routes || Object.keys(routes).length === 0) {
    console.log("❌ No routes to apply");
    appendDebugLine("❌ applyAgentRoutes: No routes to apply");
    return;
  }

  // Build waypoint lookup for calculating distances and points
  const waypoints = {};
  const targetPriorities = {};

  if (state.env) {
    for (const a of (state.env.airports || [])) {
      const id = a.id || a.label || "A?";
      waypoints[id] = [a.x, a.y];
    }
    for (const t of (state.env.targets || [])) {
      const id = t.id || t.label || "T?";
      waypoints[id] = [t.x, t.y];
      targetPriorities[id] = t.priority || t.value || 5;
    }
  }

  // Build allocations for the allocation display
  const allocations = {};

  for (const [droneId, route] of Object.entries(routes)) {
    if (!route || route.length < 2) continue;

    const routeStr = route.join(",");

    // Store sequence
    state.sequences[droneId] = routeStr;

    // Extract targets for allocation display
    const routeTargets = route.filter(wp => String(wp).startsWith('T'));
    allocations[droneId] = routeTargets;

    // Calculate distance (sum of segment lengths)
    let distance = 0;
    for (let i = 0; i < route.length - 1; i++) {
      const from = waypoints[route[i]];
      const to = waypoints[route[i + 1]];
      if (from && to) {
        distance += _distance(from, to);
      }
    }

    // Calculate points (sum of target priorities)
    let points = 0;
    for (const tid of routeTargets) {
      points += targetPriorities[tid] || 0;
    }

    // Use server-provided SAM-avoiding trajectory if available, otherwise build locally
    let trajectory;
    if (trajectories && trajectories[droneId] && trajectories[droneId].length > 0) {
      trajectory = trajectories[droneId];
      appendDebugLine(`Agent route D${droneId}: using SAM-avoiding trajectory (${trajectory.length} points)`);
    } else {
      trajectory = buildTrajectoryFromRoute(route);
    }

    // Build route info for visualization
    state.routes[droneId] = {
      route: route,
      points: points,
      distance: distance,
      trajectory: trajectory,
    };

    // Ensure trajectory is visible for this drone
    state.trajectoryVisible[droneId] = true;

    appendDebugLine(`Agent route D${droneId}: ${routeStr} (${points} pts, ${distance.toFixed(1)} fuel)`);
  }

  // Store allocations
  state.allocations = allocations;

  // Update the sequence input to show the currently selected drone's sequence
  const curDrone = state.currentDroneForSeq || "1";
  const seqInput = $("sequence-input");
  if (seqInput && state.sequences[curDrone]) {
    seqInput.value = state.sequences[curDrone];
  }

  // Update allocation display in Env tab
  updateAllocationDisplay(allocations);

  // Update debug output
  appendDebugLine(`Agent applied ${Object.keys(routes).length} drone routes (${totalPoints || "?"} pts, ${totalFuel?.toFixed(1) || "?"} fuel)`);

  // Redraw and update stats
  drawEnvironment();
  updateStatsFromRoutes();
}

function applyAgentRoute(route, points, fuel) {
  // Apply the agent's route to drone D1 (legacy single-drone)
  // Route is like ["A1", "T3", "T7", "A1"]

  if (!route || route.length < 2) return;

  const routeStr = route.join(",");

  // Store as D1's sequence
  state.sequences["1"] = routeStr;

  // Update the sequence input if it exists
  const seqInput = $("sequence-input");
  if (seqInput) {
    seqInput.value = routeStr;
  }

  // Calculate points and distance if not provided
  let calculatedPoints = points || 0;
  let calculatedFuel = fuel || 0;

  if (!points || !fuel) {
    // Build waypoint lookup
    const waypoints = {};
    const targetPriorities = {};

    if (state.env) {
      for (const a of (state.env.airports || [])) {
        const id = a.id || a.label || "A?";
        waypoints[id] = [a.x, a.y];
      }
      for (const t of (state.env.targets || [])) {
        const id = t.id || t.label || "T?";
        waypoints[id] = [t.x, t.y];
        targetPriorities[id] = t.priority || t.value || 5;
      }
    }

    // Calculate distance
    if (!fuel) {
      calculatedFuel = 0;
      for (let i = 0; i < route.length - 1; i++) {
        const from = waypoints[route[i]];
        const to = waypoints[route[i + 1]];
        if (from && to) {
          calculatedFuel += _distance(from, to);
        }
      }
    }

    // Calculate points
    if (!points) {
      calculatedPoints = 0;
      const routeTargets = route.filter(wp => String(wp).startsWith('T'));
      for (const tid of routeTargets) {
        calculatedPoints += targetPriorities[tid] || 0;
      }
    }
  }

  // Build route info for visualization
  state.routes["1"] = {
    route: route,
    points: calculatedPoints,
    distance: calculatedFuel,
    trajectory: buildTrajectoryFromRoute(route),
  };
}

window.addEventListener("keydown", (e) => {
  if (String(e.key).toLowerCase() === "c") {
    // Skip if user is typing in an input field
    const activeTag = document.activeElement?.tagName?.toLowerCase();
    if (activeTag === "input" || activeTag === "textarea" || activeTag === "select") {
      return;
    }

    appendDebugLine("Keypress: C detected (attempting cut/checkpoint)");
    try {
      cutAtCheckpoint();
    } catch (err) {
      console.error("cutAtCheckpoint failed:", err);
      appendDebugLine("Cut failed (see console)");
    }
  }

  // P key for Pause
  if (String(e.key).toLowerCase() === "p") {
    // Skip if user is typing in an input field
    const activeTag = document.activeElement?.tagName?.toLowerCase();
    if (activeTag === "input" || activeTag === "textarea" || activeTag === "select") {
      return;
    }

    const perms = getUiPermissions();
    if (perms.canPause) {
      pauseAnimation();
    }
  }
});

function buildTrajectoryFromRoute(route) {
  // Build trajectory points from waypoint IDs
  if (!state.env || !route) return [];

  const trajectory = [];
  const waypoints = {};

  // Index airports
  for (const a of (state.env.airports || [])) {
    const id = a.id || a.label || "A?";
    waypoints[id] = [a.x, a.y];
  }

  // Index targets
  for (const t of (state.env.targets || [])) {
    const id = t.id || t.label || "T?";
    waypoints[id] = [t.x, t.y];
  }

  // Build trajectory
  for (const wp of route) {
    if (waypoints[wp]) {
      trajectory.push(waypoints[wp]);
    }
  }

  return trajectory;
}
