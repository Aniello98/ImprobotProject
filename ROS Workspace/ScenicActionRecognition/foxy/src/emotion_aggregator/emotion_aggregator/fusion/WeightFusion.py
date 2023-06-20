from emotion_aggregator.fusion.EmotionFusionMethod import EmotionFusionMethod


class WeightFusion(EmotionFusionMethod):

    __face_weight: float
    __body_weight: float

    def __init__(self, face_weight: float = 0.7) -> None:
        super().__init__()
        self.__face_weight = face_weight
        self.__body_weight = 1-face_weight

    def fuse(self, face_emotion, body_emotion, face_weight=None):
        if face_weight == None:
            return self.__face_weight * face_emotion + self.__body_weight * body_emotion
        else:
            return face_weight * face_emotion + (1-face_weight) * body_emotion
