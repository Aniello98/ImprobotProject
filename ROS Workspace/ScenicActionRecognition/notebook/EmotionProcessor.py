from paz.applications import MiniXceptionFER
import paz.processors as pr
import numpy as np

class EmotionProcessor(pr.Processor):
    def __init__(self):
        super(EmotionProcessor, self).__init__()
        self.classify = MiniXceptionFER()

    def call(self, image):
        # The incoming image is already a face cropped from the original frame
        # Classifiy emotion
        classification = self.classify(image)
        # And eventually return
        return classification["scores"][0]