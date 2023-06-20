from emotion_aggregator.accumulators.EmotionAccumulator import EmotionAccumulator


class LinearAccumulator(EmotionAccumulator):

    def _update_embedding(self, new_data):
        if self._update_counter == 0:
            self._scores_embedding = new_data
            return
        self._scores_embedding = (self._scores_embedding * self._update_counter + new_data) / (self._update_counter + 1)
