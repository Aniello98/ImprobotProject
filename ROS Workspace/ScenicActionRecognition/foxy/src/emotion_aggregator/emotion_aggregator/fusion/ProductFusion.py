from emotion_aggregator.fusion.EmotionFusionMethod import EmotionFusionMethod
import numpy as np


class ProductFusion(EmotionFusionMethod):

    def fuse(self, face_emotion, body_emotion):
        return np.multiply(face_emotion, body_emotion)
