# ai_detectors/sam_detector.py
import numpy as np, torch
from pathlib import Path
from .base import Detector

try:
    from segment_anything import sam_model_registry, SamAutomaticMaskGenerator
except ImportError:
    sam_model_registry = None      # allow unit tests w/out SAM installed

_ROOT = Path(__file__).resolve().parent.parent
_CKPT = _ROOT / "weights/sam_vit_b_01ec64.pth"

class SamDetector(Detector):
    _model = None
    def __init__(self, device="cuda", model_type="vit_b"):
        super().__init__(device)
        if SamDetector._model is None:
            if sam_model_registry is None:
                raise ImportError("segment-anything not installed")
            if not _CKPT.exists():
                raise FileNotFoundError(f"SAM weight not found: {_CKPT}")
            model = sam_model_registry[model_type](checkpoint=str(_CKPT))
            model.to(device, dtype=torch.float16)  # FP16 to save VRAM
            SamDetector._model = SamAutomaticMaskGenerator(model)

    def detect(self, bgr_img):
        masks = SamDetector._model.generate(bgr_img)  # list of dicts
        # convert to boxes for mapper compatibility
        boxes = np.array([m["bbox"] for m in masks], dtype=np.float32)
        return {"boxes": boxes, "masks": masks}
