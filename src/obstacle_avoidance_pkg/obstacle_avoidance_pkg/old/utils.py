"""
NOT USED ANYMORE
"""

from enum import Enum

class NavigationMode(Enum):
    """Enum class for navigation modes."""
    STRAIGHT = 1
    OBSTACLE_AVOIDANCE = 2
    CORNER = 3

class BufferElement:
    """Class representing the items stored in the buffer stack.
    Eacn element has a timestamp, angular velocity, and linear velocity.
    """
    def __init__(self, timestamp, angular_velocity, linear_velocity):
        self.timestamp = timestamp
        self.angular_velocity = angular_velocity
        self.linear_velocity = linear_velocity