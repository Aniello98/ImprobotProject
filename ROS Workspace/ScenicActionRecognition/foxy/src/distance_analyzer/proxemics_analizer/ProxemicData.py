import time


class ProxemicData:
    """Class representing the proxemic information detected by the system
    """

    __x: float
    __y: float
    __timestamp: int
    __orientation: float  # TODO: Check if it makes sense and we can sense this

    def __init__(self, x: float, y: float):
        self.__x = x
        self.__y = y
        # Add current timestamp
        self.__timestamp = round(time.time() * 1000)

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def timestamp(self):
        return self.__timestamp
