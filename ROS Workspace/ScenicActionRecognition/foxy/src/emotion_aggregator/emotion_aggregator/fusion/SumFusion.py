from emotion_aggregator.fusion.EmotionFusionMethod import EmotionFusionMethod


class SumFusion(EmotionFusionMethod):

    def fuse(self, face_emotion, body_emotion):
        return face_emotion + body_emotion
