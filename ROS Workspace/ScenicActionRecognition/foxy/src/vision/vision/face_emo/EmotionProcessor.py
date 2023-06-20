from paz.applications import MiniXceptionFER
import paz.processors as pr
import numpy as np
from keras.models import load_model
from pathlib import Path
import os




class EmotionProcessor(pr.Processor):
    def __init__(self):
        super(EmotionProcessor, self).__init__()
        emotionModelPath = os.getcwd() + "/src/vision/vision/face_emo/models/emotionModel.hdf5"
        self.emotionClassifier = load_model(emotionModelPath, compile=False)
        self.emotionTargetSize = self.emotionClassifier.input_shape[1:3]
        self.classify = MiniXceptionFER()

    def call(self, image):
        # The incoming image is already a face cropped from the original frame
        # Classifiy emotion
        #classification = self.classify(image)
        classification = self.emotionClassifier.predict(image)
        # And eventually return
        #return classification["scores"][0]        
        return classification[0]
