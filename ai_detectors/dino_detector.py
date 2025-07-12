# ai_detectors/dino_detector.py
import numpy as np, cv2 as cv, torch
import cv2
from pathlib import Path
from GroundingDINO.groundingdino.util.inference import Model, predict, load_model
from .base import Detector

_ROOT  = Path(__file__).resolve().parent.parent
_CFG   = _ROOT / "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
_CKPT  = _ROOT / "weights/groundingdino_swint_ogc.pth"

class DinoDetector(Detector):
    _model = None                 # global cache

    def __init__(self,
                 device="cuda",
                 prompt="ground floor road terrain person object",
                 box_threshold=0.10,
                 text_threshold=0.10):
        super().__init__(device)
        self.prompt, self.box_thr, self.txt_thr = prompt, box_threshold, text_threshold
        if DinoDetector._model is None:
            if not _CKPT.exists():
                raise FileNotFoundError(f"DINO weight not found: {_CKPT}")
            DinoDetector._model = Model(
    model_config_path=str(_CFG),
    model_checkpoint_path=str(_CKPT),
    device=device
)
    def detect(self, bgr_img):
        import groundingdino.datasets.transforms as T
        from PIL import Image

        rgb_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2RGB)
        image_pil = Image.fromarray(rgb_img)

        transform = T.Compose([
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        image_tensor, _ = transform(image_pil, None)

        boxes, logits, phrases = predict(
            model=DinoDetector._model.model,
            image=image_tensor,
            caption=self.prompt,
            box_threshold=self.box_thr,
            text_threshold=self.txt_thr,
            device=self.device
        )

        # convert boxes to numpy array with shape (N, 4)
        boxes = boxes.cpu().numpy().astype(np.float32)

        return {"boxes": boxes}


    #def detect(self, bgr_img):
    #    bgr_img = bgr_img.copy()  # groundingdino modifies in-place
    #    boxes = DinoDetector._model.predict(
    #        bgr_img,
    #        self.prompt,
    #        box_threshold=self.box_thr,
    #        text_threshold=self.txt_thr
    #    ).astype(np.float32)
    #    return {"boxes": boxes}
