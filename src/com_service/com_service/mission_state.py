from enum import Enum, auto


class MissionState(Enum):
    """SDD-001 3.1절 - 상태 머신 상태 정의"""
    SYSTEM_INIT = auto()
    LOCALIZING = auto()
    READY = auto()
    GUIDING = auto()
    OBSTACLE_STOP = auto()
    ARRIVED = auto()
    PERSON_WAIT = auto()
    RETURNING = auto()
    DOCKING = auto()
    DOCKED = auto()
    PAUSED = auto()
    EMERGENCY_STOP = auto()
    ERROR = auto()
    SHUTDOWN = auto()
