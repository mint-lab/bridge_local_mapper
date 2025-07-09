# ai_detectors/dino_detector.py
import numpy as np, cv2 as cv, torch
from pathlib import Path
from groundingdino.util.inference import Model
from .base import Detector

_ROOT  = Path(__file__).resolve().parent.parent
_CFG   = _ROOT / "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
_CKPT  = _ROOT / "weights/groundingdino_swint_ogc.pth"

class DinoDetector(Detector):
    _model = None                 # global cache

    def __init__(self,
                 device="cuda",
                 prompt="ground floor road terrain",
                 box_threshold=0.10,
                 text_threshold=0.10):
        super().__init__(device)
        self.prompt, self.box_thr, self.txt_thr = prompt, box_threshold, text_threshold
        if DinoDetector._model is None:
            if not _CKPT.exists():
                raise FileNotFoundError(f"DINO weight not found: {_CKPT}")
            DinoDetector._model = Model(cfg=str(_CFG), ckpt=str(_CKPT), device=device)

    def detect(self, bgr_img):
        bgr_img = bgr_img.copy()  # groundingdino modifies in-place
        boxes = DinoDetector._model.predict(
            bgr_img,
            self.prompt,
            box_threshold=self.box_thr,
            text_threshold=self.txt_thr
        ).astype(np.float32)
        return {"boxes": boxes}
