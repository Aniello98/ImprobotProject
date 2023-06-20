class ProxemicAnalysisResult:
    __linear_movement_embedding: int
    __zone_embedding: int
    __position_embedding: int

    def __init__(self, linear_movement: int, lateral_movement: int, zone: int, position: int, linear_speed: float, angular_speed: float):
        self.__linear_movement_embedding = linear_movement
        self.__zone_embedding = zone
        self.__position_embedding = position
        self.__lateral_movement_embedding = lateral_movement
        self.__linear_speed = linear_speed
        self.__angular_speed = angular_speed

    @property
    def linear_movement(self):
        return self.__linear_movement_embedding

    @property
    def lateral_movement(self):
        return self.__lateral_movement_embedding

    @property
    def zone(self):
        return self.__zone_embedding

    @property
    def actor_position(self):
        return self.__position_embedding

    @property
    def linear_speed(self):
        return self.__linear_speed

    @property
    def angular_speed(self):
        return self.__angular_speed
