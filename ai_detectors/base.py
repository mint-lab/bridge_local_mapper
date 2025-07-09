# ai_detectors/base.py
class Detector:
    """
    Base class for all detectors.
    `detect()` must return a dict with at least:
        {"boxes": np.ndarray [N,4] (xyxy, float32)}
    It may also include masks or extra keys.
    """
    def __init__(self, device: str = "cuda"):
        self.device = device

    def detect(self, bgr_img):
        raise NotImplementedError("Sub-class must implement detect()")
