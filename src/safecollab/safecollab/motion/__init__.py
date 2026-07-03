"""Motion — fuses the nominal trajectory with the safety scale (the speed knob).

Pure ``retime`` + ``logic`` + ``config`` are separated from the ROS ``node``.
"""

from safecollab.motion.config import load_motion_config
from safecollab.motion.logic import MotionLogic
from safecollab.motion.retime import retime

__all__ = ["MotionLogic", "load_motion_config", "retime"]
