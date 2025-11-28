import numpy as np


def solve_orienteering_with_matrix(env_data, fuel_cap=None, **kwargs):
    """
    Simple placeholder orienteering solver that matches the call:

        solution = solve_orienteering_with_matrix(env_data, fuel_cap=...)

    It returns:
      - path: list of (x, y)
      - visited_goals: list of labels
      - distance: total length of the path
    """
    if not isinstance(env_data, dict):
        return {"success": False, "path": [], "visited_goals": [], "distance": 0.0}

    # Try a few common key names for airports / homes / targets
    airports = env_data.get("airports") or env_data.get("homes") or []
    targets = env_data.get("targets") or []

    # If we have no targets, nothing to do
    if not targets:
        return {"success": True, "path": [], "visited_goals": [], "distance": 0.0}

    # Pick a reasonable start: first airport if available, otherwise first target
    if airports:
        start = (airports[0]["x"], airports[0]["y"])
    else:
        start = (targets[0]["x"], targets[0]["y"])

    current = np.array(start)
    remaining = list(targets)
    path = [start]
    visited = []

    while remaining:
        # Nearest-neighbor heuristic
        distances = [
            np.linalg.norm(current - np.array((g["x"], g["y"])))
            for g in remaining
        ]
        idx = int(np.argmin(distances))
        g = remaining.pop(idx)

        label = g.get("label") or g.get("id") or f"T{len(visited) + 1}"
        visited.append(label)

        pos = (g["x"], g["y"])
        path.append(pos)
        current = np.array(pos)

    # --- NEW: compute total path length ---
    total_distance = 0.0
    if len(path) > 1:
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            total_distance += float((dx**2 + dy**2) ** 0.5)

    return {
        "success": True,
        "path": path,
        "visited_goals": visited,
        "distance": total_distance,
    }
