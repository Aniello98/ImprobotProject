from emo_body.embedding.PoseEmbedder import PoseEmbedder
import numpy as np


def _get_distance(lmk_from, lmk_to):
    return lmk_to - lmk_from

class MpEmbedder(PoseEmbedder):

    def __init__(self, torso_size_multiplier=1.5):
        super().__init__()
        self._torso_size_multiplier = torso_size_multiplier
        # Names of the landmarks as they appear in the prediction.
        self._landmark_names = [
            'nose',
            'left_shoulder', 'right_shoulder',
            'left_elbow', 'right_elbow',
            'left_wrist', 'right_wrist',
            'left_hip', 'right_hip',
            'left_knee', 'right_knee',
            'left_ankle', 'right_ankle'
        ]

    def __call__(self, pose_np):
        """
        Generate an embedding of the provided pose
        :param results: the list of landmarks detected by MediaPipe
        :return: A tuple containing the embedding and the normalized pose
        """
        super().__call__(pose_np)

        normalized_pose = self._normalize_pose_landmarks(pose_np)
        embedding = self._get_pose_distance_embedding(normalized_pose)

        return embedding

    def _normalize_pose_landmarks(self, landmarks):
        """Normalizes landmarks translation and scale."""
        landmarks = np.copy(landmarks)

        # Normalize translation.
        pose_center = self._get_pose_center(landmarks, "left_hip", "right_hip")
        landmarks -= pose_center

        # Normalize scale.
        pose_size = self._get_pose_size(landmarks, self._torso_size_multiplier)
        landmarks /= pose_size

        return landmarks

    def _get_pose_center(self, landmarks, left_part, right_part):
        """Calculates pose center as point between hips."""
        left_hip = landmarks[self._landmark_names.index(left_part)]
        right_hip = landmarks[self._landmark_names.index(right_part)]
        center = (left_hip + right_hip) * 0.5
        return center

    def _get_pose_size(self, landmarks, torso_size_multiplier):
        """Calculates pose size.

        It is the maximum of two values:
          * Torso size multiplied by `torso_size_multiplier`
          * Maximum distance from pose center to any pose landmark
        """

        # Hips center.
        hips = self._get_pose_center(landmarks, "left_hip", "right_hip")
        shoulders = self._get_pose_center(
            landmarks, "left_shoulder", "right_shoulder")

        # Torso size as the minimum body size.
        torso_size = np.linalg.norm(shoulders - hips)

        pose_center = self._get_pose_center(landmarks, "left_hip", "right_hip")
        max_dist = np.max(np.linalg.norm(landmarks - pose_center, axis=1))

        return max(torso_size * torso_size_multiplier, max_dist)

    def _get_pose_distance_embedding(self, landmarks):
        embedding = np.array([
            # One joint.

            _get_distance(
                self._get_average_by_names(landmarks, 'left_hip', 'right_hip'),
                self._get_average_by_names(landmarks, 'left_shoulder', 'right_shoulder')),

            self._get_distance_by_names(
                landmarks, 'left_shoulder', 'left_elbow'),
            self._get_distance_by_names(
                landmarks, 'right_shoulder', 'right_elbow'),

            self._get_distance_by_names(landmarks, 'left_elbow', 'left_wrist'),
            self._get_distance_by_names(
                landmarks, 'right_elbow', 'right_wrist'),

            self._get_distance_by_names(landmarks, 'left_hip', 'left_knee'),
            self._get_distance_by_names(landmarks, 'right_hip', 'right_knee'),

            self._get_distance_by_names(landmarks, 'left_knee', 'left_ankle'),
            self._get_distance_by_names(
                landmarks, 'right_knee', 'right_ankle'),

            # Two joints.

            self._get_distance_by_names(
                landmarks, 'left_shoulder', 'left_wrist'),
            self._get_distance_by_names(
                landmarks, 'right_shoulder', 'right_wrist'),

            self._get_distance_by_names(landmarks, 'left_hip', 'left_ankle'),
            self._get_distance_by_names(landmarks, 'right_hip', 'right_ankle'),

            # Four joints.

            self._get_distance_by_names(landmarks, 'left_hip', 'left_wrist'),
            self._get_distance_by_names(landmarks, 'right_hip', 'right_wrist'),

            # Five joints.

            self._get_distance_by_names(
                landmarks, 'left_shoulder', 'left_ankle'),
            self._get_distance_by_names(
                landmarks, 'right_shoulder', 'right_ankle'),

            self._get_distance_by_names(landmarks, 'left_hip', 'left_wrist'),
            self._get_distance_by_names(landmarks, 'right_hip', 'right_wrist'),

            # Cross body.

            self._get_distance_by_names(
                landmarks, 'left_elbow', 'right_elbow'),
            self._get_distance_by_names(landmarks, 'left_knee', 'right_knee'),

            self._get_distance_by_names(
                landmarks, 'left_wrist', 'right_wrist'),
            self._get_distance_by_names(
                landmarks, 'left_ankle', 'right_ankle'),

            # Body bent direction.

            _get_distance(
                self._get_average_by_names(
                    landmarks, 'left_wrist', 'left_ankle'),
                landmarks[self._landmark_names.index('left_hip')]),
            _get_distance(
                self._get_average_by_names(
                    landmarks, 'right_wrist', 'right_ankle'),
                landmarks[self._landmark_names.index('right_hip')]),
        ])

        return embedding

    def _get_average_by_names(self, landmarks, name_from, name_to):
        lmk_from = landmarks[self._landmark_names.index(name_from)]
        lmk_to = landmarks[self._landmark_names.index(name_to)]
        return (lmk_from + lmk_to) * 0.5

    def _get_distance_by_names(self, landmarks, name_from, name_to):
        lmk_from = landmarks[self._landmark_names.index(name_from)]
        lmk_to = landmarks[self._landmark_names.index(name_to)]
        return _get_distance(lmk_from, lmk_to)
