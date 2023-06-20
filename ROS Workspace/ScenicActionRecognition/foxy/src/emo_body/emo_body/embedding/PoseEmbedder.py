from abc import ABC, abstractmethod


class PoseEmbedder(ABC):
    """
    An interface describing an object that produces a zoom-invariant embedding of a pose,
    as well as normalized joint position, to be used when training a neural network or when computing other metrics
    """

    def __call__(self, pose):
        """
        Generate an embedding of the given pose
        :param pose: the object representing the pose (compatible data structure)
        :return: A tuple containing the embedding and the normalized pose
        """
        pass
