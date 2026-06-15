import torch
from ultralytics import YOLO
from config import *
class PedestrianDetector:
    def __init__(self, model_path=None, conf_threshold=None, device=None):
        model_path = model_path or YOLO_MODEL_PATH
        self.conf_threshold = conf_threshold or YOLO_CONF_THRESHOLD
        self.device = "cpu"
        self.model = YOLO(model_path)
        self.model.to(self.device)

    def detect(self, bgr_frame):
        
        results = self.model.predict(
            source=bgr_frame,
            conf=self.conf_threshold,
            classes=[YOLO_PERSON_CLASS_ID],
            device=self.device,
            verbose=False,
        )

        detections = []
        result = results[0]

        if result.boxes is None:
            return detections

        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = (int(v) for v in xyxy)
            conf = float(box.conf[0].cpu().numpy())

            detections.append({
                "bbox": (x1, y1, x2, y2),
                "conf": conf,
            })

        return detections
