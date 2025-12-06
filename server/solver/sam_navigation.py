"""
SAM Navigation module for ISR Editor (Server)
Re-exports SAMNavigator from path_planning_core for backward compatibility.

The canonical implementation lives in path_planning_core/sam_navigator.py
"""
import sys
import os

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Re-export from single source of truth
from path_planning_core import SAMNavigator

__all__ = ['SAMNavigator']
