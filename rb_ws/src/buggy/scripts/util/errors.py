import enum

class BuggyError(enum.IntFlag):
    """Describes the various types of errors/warnings emitted by the buggy.

    Errors and warnings are generally emitted to the Watchdog on the "buggy_errors" topic, which will
    handle proper LED indications and logging, although subsystem specific data should be logged.
    """

    @classmethod
    @property
    def ros_topic_name(cls):
        return 'buggy_errors'

    REALLY_FREAKING_BAD = enum.auto()

    PATH_PLANNING_FAULT = enum.auto()
    STANLEY_CRAPPED_ITSELF = enum.auto()
    VISION_UNAVAILABLE = enum.auto()
    SENSOR_UNAVAILABLE = enum.auto()

    PNEUMATIC_FAULT = enum.auto()
    MOTOR_FAULT = enum.auto()
    SENSOR_FAULT = enum.auto()

    CONTROLLER_CONNECTION_LOST = enum.auto()
    INTERBUGGY_COMMUNICATION_LOST = enum.auto()
    GPS_UNAVAILABLE = enum.auto()

    LOW_BATTERY = enum.auto()
