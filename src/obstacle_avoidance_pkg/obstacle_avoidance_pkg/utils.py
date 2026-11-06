from enum import Enum, auto
import math

class State(Enum):
    CRUISE = auto()
    TURN_AWAY = auto()
    SIDESTEP = auto()
    PASS = auto()
    CORNER = auto()

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
