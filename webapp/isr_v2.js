/**
 * ISR Mission Planner v2 - Clean Rewrite
 * ======================================
 * Single state object, clear event handlers, consistent UI updates.
 */

// =============================================================================
// State - Single source of truth
// =============================================================================

const state = {
  // Environment
  targets: [],
  airports: [],
  sams: [],

  // Drone configs
  droneConfigs: {
    "1": { enabled: true, fuel_budget: 300, start_airport: "A1", end_airport: "A1", target_access: { A: true, B: true, C: true, D: true } },
    "2": { enabled: true, fuel_budget: 300, start_airport: "A2", end_airport: "A2", target_access: { A: true, B: true, C: true, D: true } },
    "3": { enabled: true, fuel_budget: 300, start_airport: "A3", end_airport: "A4", target_access: { A: true, B: true, C: true, D: true } },
    "4": { enabled: true, fuel_budget: 300, start_airport: "A4", end_airport: "A3", target_access: { A: true, B: true, C: true, D: true } },
    "5": { enabled: true, fuel_budget: 300, start_airport: "A1", end_airport: "A1", target_access: { A: true, B: true, C: true, D: true } },
  },

  // Solution
  routes: {},
  allocations: {},
  wrappedPolygons: [],

  // UI state
  selectedDrone: "1",
  visibleTrajectories: new Set(),
  allocationStrategy: "efficient",

  // Canvas
  canvas: null,
  ctx: null,
  scale: 8,
  offsetX: 20,
  offsetY: 20,
};

// Drone colors
const DRONE_COLORS = {
  "1": "#ef4444",
  "2": "#3b82f6",
  "3": "#22c55e",
  "4": "#a855f7",
  "5": "#f97316",
};

// =============================================================================
// Initialization
// =============================================================================

function init() {
  state.canvas = document.getElementById("env-canvas");
  state.ctx = state.canvas.getContext("2d");

  setupEventHandlers();
  loadDefaultEnvironment();
  render();

  console.log("ISR Planner v2 initialized");
}

function setupEventHandlers() {
  // Solve button
  document.getElementById("btn-solve").addEventListener("click", solve);

  // Trajectory buttons
  for (let i = 1; i <= 5; i++) {
    const btn = document.getElementById(`traj-d${i}`);
    if (btn) {
      btn.addEventListener("click", () => toggleTrajectory(String(i)));
    }
  }

  const trajAll = document.getElementById("traj-all");
  if (trajAll) trajAll.addEventListener("click", showAllTrajectories);

  // Import/Export
  document.getElementById("btn-import").addEventListener("click", () => {
    document.getElementById("file-input").click();
  });
  document.getElementById("file-input").addEventListener("change", handleImport);
  document.getElementById("btn-export").addEventListener("click", handleExport);

  // Strategy dropdown
  const strategySelect = document.getElementById("allocation-strategy");
  if (strategySelect) {
    strategySelect.addEventListener("change", (e) => {
      state.allocationStrategy = e.target.value;
    });
  }

  // Drone config inputs
  for (let i = 1; i <= 5; i++) {
    const fuelInput = document.getElementById(`fuel-d${i}`);
    if (fuelInput) {
      fuelInput.addEventListener("change", (e) => {
        state.droneConfigs[String(i)].fuel_budget = parseFloat(e.target.value) || 300;
      });
    }

    const enabledCheckbox = document.getElementById(`enabled-d${i}`);
    if (enabledCheckbox) {
      enabledCheckbox.addEventListener("change", (e) => {
        state.droneConfigs[String(i)].enabled = e.target.checked;
      });
    }
  }
}

function loadDefaultEnvironment() {
  // Default environment - can be overridden by import
  state.airports = [
    { id: "A1", x: 10, y: 10 },
    { id: "A2", x: 90, y: 10 },
    { id: "A3", x: 10, y: 90 },
    { id: "A4", x: 90, y: 90 },
  ];

  state.targets = [];
  state.sams = [];
}

// =============================================================================
// API Calls
// =============================================================================

async function solve() {
  const btn = document.getElementById("btn-solve");
  btn.disabled = true;
  btn.textContent = "Solving...";

  try {
    const response = await fetch("/api/solve", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        env: {
          targets: state.targets,
          airports: state.airports,
          sams: state.sams,
        },
        drone_configs: state.droneConfigs,
        allocation_strategy: state.allocationStrategy,
      }),
    });

    const data = await response.json();
    console.log("Solve response:", data);

    if (data.success) {
      state.routes = data.routes || {};
      state.allocations = data.allocations || {};
      state.wrappedPolygons = data.wrapped_polygons || [];

      // Show all trajectories after solve
      state.visibleTrajectories.clear();
      Object.keys(state.routes).forEach((did) => state.visibleTrajectories.add(did));

      updateUI();
      render();
    } else {
      alert("Solve failed: " + (data.error || "Unknown error"));
    }
  } catch (e) {
    console.error("Solve error:", e);
    alert("Solve error: " + e.message);
  } finally {
    btn.disabled = false;
    btn.textContent = "Solve";
  }
}

// =============================================================================
// UI Updates
// =============================================================================

function updateUI() {
  updateStats();
  updateAllocations();
}

function updateStats() {
  for (let i = 1; i <= 5; i++) {
    const did = String(i);
    const route = state.routes[did];

    const distEl = document.getElementById(`stat-d${i}-dist`);
    const ptsEl = document.getElementById(`stat-d${i}-pts`);
    const pfEl = document.getElementById(`stat-d${i}-pf`);

    if (route) {
      const dist = route.distance || 0;
      const pts = route.points || 0;
      const pf = dist > 0 ? (pts / dist).toFixed(2) : "0.00";

      if (distEl) distEl.textContent = dist.toFixed(1);
      if (ptsEl) ptsEl.textContent = pts;
      if (pfEl) pfEl.textContent = pf;
    } else {
      if (distEl) distEl.textContent = "0";
      if (ptsEl) ptsEl.textContent = "0";
      if (pfEl) pfEl.textContent = "0.00";
    }
  }
}

function updateAllocations() {
  const container = document.getElementById("allocation-display");
  if (!container) return;

  const allocations = state.allocations;
  console.log("Updating allocations display:", allocations);

  if (!allocations || Object.keys(allocations).length === 0) {
    container.innerHTML = '<div style="color: #888;">No allocation data</div>';
    return;
  }

  let html = "";
  for (const did of Object.keys(allocations).sort((a, b) => parseInt(a) - parseInt(b))) {
    const targets = allocations[did];
    const color = DRONE_COLORS[did] || "#888";

    if (targets && targets.length > 0) {
      html += `<div style="color: ${color}; margin-bottom: 2px;">D${did}: ${targets.join(", ")}</div>`;
    }
  }

  container.innerHTML = html || '<div style="color: #888;">No targets allocated</div>';
}

// =============================================================================
// Canvas Rendering
// =============================================================================

function render() {
  const ctx = state.ctx;
  const canvas = state.canvas;

  // Clear
  ctx.fillStyle = "#0f172a";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Draw grid
  drawGrid();

  // Draw wrapped polygons (yellow)
  drawWrappedPolygons();

  // Draw SAMs
  drawSAMs();

  // Draw trajectories
  drawTrajectories();

  // Draw airports
  drawAirports();

  // Draw targets
  drawTargets();
}

function toCanvas(x, y) {
  return [state.offsetX + x * state.scale, state.offsetY + y * state.scale];
}

function drawGrid() {
  const ctx = state.ctx;
  ctx.strokeStyle = "#1e293b";
  ctx.lineWidth = 0.5;

  for (let i = 0; i <= 100; i += 10) {
    const [x1, y1] = toCanvas(i, 0);
    const [x2, y2] = toCanvas(i, 100);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();

    const [x3, y3] = toCanvas(0, i);
    const [x4, y4] = toCanvas(100, i);
    ctx.beginPath();
    ctx.moveTo(x3, y3);
    ctx.lineTo(x4, y4);
    ctx.stroke();
  }
}

function drawWrappedPolygons() {
  const ctx = state.ctx;

  for (const polygon of state.wrappedPolygons) {
    if (polygon.length < 2) continue;

    ctx.strokeStyle = "#facc15";
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 3]);

    ctx.beginPath();
    const [x0, y0] = toCanvas(polygon[0][0], polygon[0][1]);
    ctx.moveTo(x0, y0);

    for (let i = 1; i < polygon.length; i++) {
      const [x, y] = toCanvas(polygon[i][0], polygon[i][1]);
      ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function drawSAMs() {
  const ctx = state.ctx;

  for (const sam of state.sams) {
    const [cx, cy] = toCanvas(sam.x, sam.y);
    const r = sam.range * state.scale;

    // Red fill
    ctx.fillStyle = "rgba(239, 68, 68, 0.3)";
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fill();

    // Red border
    ctx.strokeStyle = "#ef4444";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Label
    ctx.fillStyle = "#fff";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    ctx.fillText(sam.id, cx, cy + 3);
  }
}

function drawTrajectories() {
  const ctx = state.ctx;

  for (const did of state.visibleTrajectories) {
    const route = state.routes[did];
    if (!route || !route.trajectory || route.trajectory.length < 2) continue;

    const color = DRONE_COLORS[did] || "#888";
    const traj = route.trajectory;

    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 2]);

    ctx.beginPath();
    const [x0, y0] = toCanvas(traj[0][0], traj[0][1]);
    ctx.moveTo(x0, y0);

    for (let i = 1; i < traj.length; i++) {
      const [x, y] = toCanvas(traj[i][0], traj[i][1]);
      ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function drawAirports() {
  const ctx = state.ctx;

  for (const airport of state.airports) {
    const [cx, cy] = toCanvas(airport.x, airport.y);

    // Square
    ctx.fillStyle = "#22c55e";
    ctx.fillRect(cx - 6, cy - 6, 12, 12);

    // Label
    ctx.fillStyle = "#fff";
    ctx.font = "bold 10px monospace";
    ctx.textAlign = "center";
    ctx.fillText(airport.id, cx, cy + 3);
  }
}

function drawTargets() {
  const ctx = state.ctx;

  // Build target-to-drone map from allocations
  const targetToDrone = {};
  for (const [did, targets] of Object.entries(state.allocations)) {
    for (const tid of targets) {
      targetToDrone[tid] = did;
    }
  }

  for (const target of state.targets) {
    const [cx, cy] = toCanvas(target.x, target.y);
    const assignedDrone = targetToDrone[target.id];
    const color = assignedDrone ? DRONE_COLORS[assignedDrone] : "#888";

    // Circle
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(cx, cy, 8, 0, Math.PI * 2);
    ctx.fill();

    // Border
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1;
    ctx.stroke();

    // Label
    ctx.fillStyle = "#fff";
    ctx.font = "bold 9px monospace";
    ctx.textAlign = "center";
    ctx.fillText(target.id, cx, cy + 3);
  }
}

// =============================================================================
// Trajectory Controls
// =============================================================================

function toggleTrajectory(droneId) {
  if (state.visibleTrajectories.has(droneId)) {
    state.visibleTrajectories.delete(droneId);
  } else {
    state.visibleTrajectories.add(droneId);
  }
  updateTrajectoryButtons();
  render();
}

function showAllTrajectories() {
  state.visibleTrajectories.clear();
  Object.keys(state.routes).forEach((did) => state.visibleTrajectories.add(did));
  updateTrajectoryButtons();
  render();
}

function updateTrajectoryButtons() {
  for (let i = 1; i <= 5; i++) {
    const btn = document.getElementById(`traj-d${i}`);
    if (btn) {
      btn.classList.toggle("active", state.visibleTrajectories.has(String(i)));
    }
  }
}

// =============================================================================
// Import/Export
// =============================================================================

function handleImport(event) {
  const file = event.target.files[0];
  if (!file) return;

  const reader = new FileReader();
  reader.onload = (e) => {
    try {
      const data = JSON.parse(e.target.result);
      console.log("Imported data:", data);

      // Load environment
      state.targets = data.targets || [];
      state.airports = data.airports || [];
      state.sams = data.sams || [];

      // Load drone configs if present
      if (data.drone_configs) {
        Object.assign(state.droneConfigs, data.drone_configs);
        updateDroneConfigUI();
      }

      // Clear solution
      state.routes = {};
      state.allocations = {};
      state.wrappedPolygons = [];

      updateUI();
      render();

      console.log(`Imported: ${state.targets.length} targets, ${state.airports.length} airports, ${state.sams.length} SAMs`);
    } catch (err) {
      alert("Import error: " + err.message);
    }
  };
  reader.readAsText(file);

  // Reset file input
  event.target.value = "";
}

function handleExport() {
  const data = {
    targets: state.targets,
    airports: state.airports,
    sams: state.sams,
    drone_configs: state.droneConfigs,
    routes: state.routes,
    allocations: state.allocations,
  };

  const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);

  const a = document.createElement("a");
  a.href = url;
  a.download = `isr_mission_${Date.now()}.json`;
  a.click();

  URL.revokeObjectURL(url);
}

function updateDroneConfigUI() {
  for (let i = 1; i <= 5; i++) {
    const did = String(i);
    const cfg = state.droneConfigs[did];
    if (!cfg) continue;

    const fuelInput = document.getElementById(`fuel-d${i}`);
    if (fuelInput) fuelInput.value = cfg.fuel_budget;

    const enabledCheckbox = document.getElementById(`enabled-d${i}`);
    if (enabledCheckbox) enabledCheckbox.checked = cfg.enabled;
  }
}

// =============================================================================
// Start
// =============================================================================

document.addEventListener("DOMContentLoaded", init);
