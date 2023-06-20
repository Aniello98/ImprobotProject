from proxemics_analizer.actor_position.ActorPositionComputer import ActorPositionComputer
from impro_msgs.msg import ActorPosition


class StandardActorPositionComputer(ActorPositionComputer):

    def compute_position(self, angle: float) -> int:
        # determine the actor_position according to the angle of the person
        if -45 <= angle < 45:
            return ActorPosition.FRONT
        elif 45 <= angle < 135:
            return ActorPosition.LEFT
        elif 135 <= angle <= 180 or -180 <= angle < -135:
            return ActorPosition.BEHIND
        else:
            return ActorPosition.RIGHT
