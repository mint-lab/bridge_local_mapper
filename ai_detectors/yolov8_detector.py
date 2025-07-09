# ai_detectors/yolov8_detector.py
import numpy as np, cv2 as cv, onnxruntime as ort
from pathlib import Path
from .base import Detector

_ROOT = Path(__file__).resolve().parent.parent
_CKPT = _ROOT / "weights/yolov8n.onnx"

class YoloV8Detector(Detector):
    _sess = None
    _input_name = None
    def __init__(self, device="cpu", conf=0.25):
        super().__init__(device)
        self.conf = conf
        if YoloV8Detector._sess is None:
            if not _CKPT.exists():
                raise FileNotFoundError(f"YOLOv8 weight not found: {_CKPT}")
            providers = ["CUDAExecutionProvider"] if device=="cuda" else ["CPUExecutionProvider"]
            YoloV8Detector._sess = ort.InferenceSession(str(_CKPT), providers=providers)
            YoloV8Detector._input_name = YoloV8Detector._sess.get_inputs()[0].name

    def detect(self, bgr_img):
        img = cv.resize(bgr_img, (640,640))
        img = img.transpose(2,0,1).astype(np.float32)/255.0
        pred = YoloV8Detector._sess.run(None, {YoloV8Detector._input_name: img[None]})[0][0]
        boxes = []
        for x1,y1,x2,y2,conf,cls in pred:
            if conf < self.conf: break
            boxes.append([x1,y1,x2,y2])
        return {"boxes": np.array(boxes, dtype=np.float32)}
