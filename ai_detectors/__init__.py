# ai_detectors/__init__.py
from importlib import import_module

_REGISTRY = {
    "dino" : "ai_detectors.dino_detector:DinoDetector",
    "sam"  : "ai_detectors.sam_detector:SamDetector",
    "yolo" : "ai_detectors.yolov8_detector:YoloV8Detector",
}

def build(name: str, **kwargs):
    if name not in _REGISTRY:
        raise ValueError(f"Unknown detector '{name}'. Available: {list(_REGISTRY)}")
    module_path, cls_name = _REGISTRY[name].split(":")
    cls = getattr(import_module(module_path), cls_name)
    return cls(**kwargs)
