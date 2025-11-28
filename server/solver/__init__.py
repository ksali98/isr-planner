"""
ISR Mission Solver Package
Modular solver for ISR mission planning with SAM avoidance
"""

# Import solver_bridge which uses legacy isr_editor
try:
    from .solver_bridge import (
        solve_mission,
        solve_mission_with_allocation,
        prepare_distance_matrix,
        get_current_matrix,
        clear_cached_matrix,
    )
except ImportError:
    solve_mission = None
    solve_mission_with_allocation = None
    prepare_distance_matrix = None
    get_current_matrix = None
    clear_cached_matrix = None

# Import SAM-aware distance matrix calculator
try:
    from .sam_distance_matrix import (
        calculate_sam_aware_matrix,
        get_cached_matrix,
        clear_matrix_cache,
        get_path_between,
        get_distance_between,
        SAM_DISTANCE_MATRIX_TOOL,
    )
except ImportError:
    calculate_sam_aware_matrix = None
    get_cached_matrix = None
    clear_matrix_cache = None
    get_path_between = None
    get_distance_between = None
    SAM_DISTANCE_MATRIX_TOOL = None

# Import target allocator
try:
    from .target_allocator import (
        allocate_targets,
        set_allocator_matrix,
        AllocationStrategy,
        TARGET_ALLOCATOR_TOOL,
    )
except ImportError:
    allocate_targets = None
    set_allocator_matrix = None
    AllocationStrategy = None
    TARGET_ALLOCATOR_TOOL = None

# Import post-optimizer
try:
    from .post_optimizer import (
        post_optimize_solution,
        get_unvisited_targets,
        get_coverage_stats,
        set_optimizer_matrix,
        POST_OPTIMIZER_TOOL,
        COVERAGE_STATS_TOOL,
    )
except ImportError:
    post_optimize_solution = None
    get_unvisited_targets = None
    get_coverage_stats = None
    set_optimizer_matrix = None
    POST_OPTIMIZER_TOOL = None
    COVERAGE_STATS_TOOL = None

# Optional imports for delivery solver components
try:
    from .nfz_distance_calculator import NFZDistanceCalculator
    from .nfz_trajectory_planner import NFZTrajectoryPlanner
    from .orienteering_solver import DeliveryOrienteeringSolver
    from .delivery_solver import DeliverySolver
except ImportError:
    NFZDistanceCalculator = None
    NFZTrajectoryPlanner = None
    DeliveryOrienteeringSolver = None
    DeliverySolver = None

# LangGraph tools collection
SOLVER_TOOLS = [
    tool for tool in [
        SAM_DISTANCE_MATRIX_TOOL,
        TARGET_ALLOCATOR_TOOL,
        POST_OPTIMIZER_TOOL,
        COVERAGE_STATS_TOOL,
    ] if tool is not None
]

__all__ = [
    # Main solver functions
    'solve_mission',
    'solve_mission_with_allocation',
    'prepare_distance_matrix',
    'get_current_matrix',
    'clear_cached_matrix',
    # SAM distance matrix
    'calculate_sam_aware_matrix',
    'get_cached_matrix',
    'clear_matrix_cache',
    'get_path_between',
    'get_distance_between',
    'SAM_DISTANCE_MATRIX_TOOL',
    # Target allocator
    'allocate_targets',
    'set_allocator_matrix',
    'AllocationStrategy',
    'TARGET_ALLOCATOR_TOOL',
    # Post-optimizer
    'post_optimize_solution',
    'get_unvisited_targets',
    'get_coverage_stats',
    'set_optimizer_matrix',
    'POST_OPTIMIZER_TOOL',
    'COVERAGE_STATS_TOOL',
    # Legacy delivery solver
    'NFZDistanceCalculator',
    'NFZTrajectoryPlanner',
    'DeliveryOrienteeringSolver',
    'DeliverySolver',
    # Tool collections
    'SOLVER_TOOLS',
]
