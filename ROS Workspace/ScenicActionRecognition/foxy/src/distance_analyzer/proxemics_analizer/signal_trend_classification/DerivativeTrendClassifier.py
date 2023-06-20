from collections import Counter
from typing import List, Tuple

from proxemics_analizer.signal_trend_classification.SignalTrendClassifier import \
    SignalTrendClassifier, SignalTrend


def incremental_derivative(signal) -> List[float]:
    derivatives = []
    # Move along all the data points
    for i in range(1, len(signal)):
        deriv = (signal[i] - signal[0]) / i
        derivatives.append(deriv)
    return derivatives


def derivative_embedder(derivative) -> SignalTrend:
    if derivative > 0:
        return SignalTrend.UP
    elif derivative < 0:
        return SignalTrend.DOWN
    else:
        return SignalTrend.STILL


def derivatives_voter(derivatives) -> SignalTrend:
    embeddings = [derivative_embedder(d) for d in derivatives]
    c = Counter(embeddings)
    return c.most_common(1)[0][0]


def get_average_derivative(derivatives, ref_trend) -> float:
    embeddings = [derivative_embedder(d) for d in derivatives]
    avg = 0.0
    n = 0
    for i, d in enumerate(derivatives):
        if embeddings[i] == ref_trend:
            avg += d
            n += 1
    return avg/n


class DerivativeTrendClassifier(SignalTrendClassifier):

    def classify_trend(self, signal) -> Tuple[SignalTrend, float]:
        derivatives = incremental_derivative(signal)
        trend = derivatives_voter(derivatives)
        # Compute velocity as the average derivative going in that direction
        velocity = get_average_derivative(derivatives, trend)
        return trend, velocity
