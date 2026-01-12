# Swap Closer — Cascade & Auto-Regenerate

Summary
-------
This document records the recent changes made to the Swap Closer post-optimizer to enable multi-pass cascade optimization with optional regeneration of SAM-aware trajectories between passes. The goal was to allow the optimizer to discover cascade swaps that reduce total route distance but require trajectories to be recomputed between passes.

Files changed
-------------
- `server/solver/post_optimizer.py`
  - Added support for `auto_regen` in the optimizer and updated the internal `optimize` workflow to optionally regenerate SAM-aware trajectories between iterations.
- `server/main.py`
  - The `POST /api/trajectory_swap_optimize` endpoint now forwards `auto_iterate` and `auto_regen` flags. Default behavior was changed so the endpoint runs the cascade (auto-iterate + auto-regen) by default when no flags are supplied.
- `webapp/index.html`
  - Added a checkbox `#chk-auto-regen` labeled "Auto-regenerate & cascade (one-click)" and a button `#btn-optimize-swap-until` "Swap Until No More" to the Post-Optimization controls.
- `webapp/isr.js`
  - Added client handlers for the new UI elements. The UI supports two modes:
    - one-click server-side cascade: send `auto_iterate=true, auto_regen=true` and the server runs the full cascade and returns the best solution;
    - client-looped single-pass calls: repeatedly call the single-pass endpoint and regenerate trajectories client-side between iterations to preserve per-swap visibility.
- `tools/run_swap_cascade_test.py`
  - Headless test script that builds a simple baseline solution and calls `trajectory_swap_optimize(..., auto_iterate=True, auto_regen=True)` to validate cascade behavior.
- `tools/swap_closer_diag.py`
  - Diagnostic helper for per-target SSD/OSD/insertion/fuel checks used while debugging why certain targets (e.g. T10, T39) were not moved.

Why this change
----------------
- Before: the optimizer performed a single-best-swap-per-pass using the current per-route trajectories. This meant it often missed beneficial cascades that required trajectories to be recomputed after earlier swaps.
- After: enabling regeneration between passes allows the optimizer to discover cascades where moving A enables moving B, etc. This produces substantially better final solutions on tested environments.

Behavioral notes
----------------
- Server default: To make the improvement available without requiring a UI change, the endpoint now defaults to running `auto_iterate` + `auto_regen`. Clients may override by explicitly sending `auto_iterate: false` or `auto_regen: false` in the payload.
- UI: The checkbox and the new button are convenience controls. If the deployed UI is stale or cached, the server default still runs the cascade when the endpoint is called directly.
- Cycle detection: the optimizer detects cycles and returns the best solution found across iterations.

How to test locally
-------------------
1. Create and activate a venv:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

2. Start the server (use the venv's python to ensure packages are visible):

```bash
export PYTHONPATH=.
python -m uvicorn server.main:app --reload --host 127.0.0.1 --port 8080
```

3. Open the UI: http://127.0.0.1:8080/ — Import your JSON, Solve, then click "Swap Until No More".

4. Or run the headless test:

```bash
PYTHONPATH=. python3 tools/run_swap_cascade_test.py /path/to/env.json
```

Deployment notes
----------------
- If deploying to Railway (or any host with caching/CDN), ensure the new server code and web assets are deployed and the browser cache is cleared (hard refresh or disable cache in DevTools). If the UI still looks unchanged, verify the deployed `isr.js` contains the `chk-auto-regen` marker.

Suggested follow-ups
--------------------
- Add a small integration test (pytest) that runs `trajectory_swap_optimize` on a known env and asserts improvement or expected swaps.
- Add a UI progress indicator for long-running server-side cascade runs.
- (Optional) Add a feature flag or environment variable to control the server default behavior (in case always-on cascade is undesired in some contexts).

Contact
-------
If you want me to run additional local tests or produce a PR with CI tests, tell me which envs to use and I'll add them.
