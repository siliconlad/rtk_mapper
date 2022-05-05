from enum import Enum


class TypeEnum(Enum):
    # Changes the string format of enum members
    def __str__(self):
        return str(self.name).lower()


class RTKType(TypeEnum):
    BLUE = 1
    YELLOW = 2
    ORANGE = 3
    BIG_ORANGE = 4
    CAR_START = 5
