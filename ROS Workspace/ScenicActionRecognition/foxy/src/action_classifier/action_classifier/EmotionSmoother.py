import numpy as np

SCORE_DELTA_THRES = 0.1

class EmotionSmoother:

    def __init__(self):
        self.previous_emotion = None

    def smooth(self, current_emotion):
        # Initialization case
        if self.previous_emotion is None:
            self.previous_emotion = current_emotion
            return current_emotion

        dom_ind_old = np.argmax(self.previous_emotion)
        dom_ind_now = np.argmax(current_emotion)
        # Check if dominant emotion is the same
        if dom_ind_now == dom_ind_old:
            self.previous_emotion = current_emotion
            return current_emotion
        else:
            # In this case get the current score of the old dominant emotion
            current_old_dom_score = current_emotion[dom_ind_old]
            # And check if the difference from current value is higher than the threshold
            delta = abs(self.previous_emotion[dom_ind_old] - current_old_dom_score)
            if delta > SCORE_DELTA_THRES:
                self.previous_emotion = current_emotion
                return current_emotion
            else:
                # TODO: Check wheter here we still have to update the previous emotion
                return self.previous_emotion