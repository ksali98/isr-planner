import numpy as np

def solve_orienteering_with_matrix(
    start,
    goals,
    score_matrix=None,
    max_distance=None,
    **kwargs
):
    """
    Simple placeholder orienteering solver.

    Parameters
    ----------
    start : tuple (x, y)
    goals : list of dicts with at least {'x': ..., 'y': ...}
    score_matrix : ignored placeholder
    max_distance : ignored placeholder

    Returns
    -------
    dict with keys:
      'success': True
      'path': list of (x, y)
      'visited_goals': list of goal labels
    """

    # Extract coordinates
    start_pos = (start['x'], start['y']) if isinstance(start, dict) else tuple(start)

    # Sort goals by simple nearest neighbor (distance)
    remaining = goals.copy()
    path = [start_pos]
    visited = []

    current = np.array(start_pos)

    while remaining:
        # pick nearest goal
        distances = [np.linalg.norm(current - np.array((g["x"], g["y"]))) for g in remaining]
        idx = int(np.argmin(distances))
        g = remaining.pop(idx)

        visited.append(g["label"])
        path.append((g["x"], g["y"]))
        current = np.array((g["x"], g["y"]))

    return {
        "success": True,
        "path": path,
        "visited_goals": visited
    }
