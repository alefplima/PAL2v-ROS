"""lpa2v: Paraconsistent Annotated Logic with two-valued annotation (PAL2v) for robotics and control."""
from .core import (STATES, PANResult, analyze, classify, clamp01, denormalize, normalize, pan)
from .algorithms import (PID, FusionOutput, LearningCell, PAL2vController, PAL2vFilter,
                         PAL2vFusion, ParaconsistentHybridPI, para_extractor)

__version__ = "1.0.0"
__all__ = ["STATES", "PANResult", "analyze", "classify", "clamp01", "denormalize", "normalize",
           "pan", "PID", "FusionOutput", "LearningCell", "PAL2vController", "PAL2vFilter",
           "PAL2vFusion", "ParaconsistentHybridPI", "para_extractor"]
