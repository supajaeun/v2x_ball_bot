# detector.py
from ultralytics import YOLO
import os

class BallDetector:
    def __init__(self, package_dir, model_file='ball_detector_v3.pt'):
        model_path = os.path.join(package_dir, model_file)
        self.model = YOLO(model_path)

    def detect(self, image):
        """
        이미지에서 YOLO 탐지 수행
        :return: (boxes, scores, vis_img)
        """
        results = self.model(image)
        boxes = results[0].boxes.xyxy.cpu().numpy()
        scores = results[0].boxes.conf.cpu().numpy()
        vis_img = results[0].plot()
        return boxes, scores, vis_img
