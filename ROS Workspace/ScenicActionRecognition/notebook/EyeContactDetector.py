from model import model_static
import torch
import torch.nn.functional as F
import numpy as np

from typing import Optional

MODEL_WEIGHTS = "/home/airlab/model_weights.pkl"

class EyeContactDetector:
    """
    A class representing an object that performs estimation of eye-contact presence on a given frame, smoothing results over time
    """

    def __init__(self) -> None:
        # Initialize the model with pre-trained weights
        self.__model = model_static(MODEL_WEIGHTS)
        self.__model_dict = self.__model.state_dict()
        snapshot = torch.load(MODEL_WEIGHTS, map_location=torch.device('cpu'))
        self.__model_dict.update(snapshot)
        self.__model.load_state_dict(self.__model_dict)

        # Set inference mode on the model
        self.__model.train(False)

        self.__accumulated_value = None

    def detect_eye_contact_on_face(self, face: Optional[object]):
        # Initialize score to 0 (no face and hence no eye contact)
        score = 0
        # Then activate network only if a face is present
        if face is not None:
            # Run inference on the face
            output = self.__model(face)
            score = F.sigmoid(output).item()

        # Smooth the score
        if self.__accumulated_value is not None:
            # Add an epsilon, so we don't have 0
            score += 0.0001
            # Smooth, by averaging new and old data, but by giving more importance to the new point
            smoothed_value = np.average([self.__accumulated_value, score], weights=[1, 3])
            self.__accumulated_value = smoothed_value
        else: 
            # If this is the first run, initialize the accumulated value
            self.__accumulated_value = score
        # In the end return the smooth value
        return self.__accumulated_value
