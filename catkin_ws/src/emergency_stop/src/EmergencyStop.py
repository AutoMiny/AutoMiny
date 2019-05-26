#!/usr/bin/env python

PACKAGE = "emergency_stop"


class EmergencyStop:
    _emergencyStopConfig = None
    _currentSpeed = 0.0
    _wantedSpeed = 0
    _emergencyStop = True

    def __init__(self):
        pass

    def set_config(self, emergencyStopConfig):
        self._emergencyStopConfig = emergencyStopConfig

    def check_emergency_stop(self, laserScan):
        # code
        return True

    def set_current_speed(self, speed):
        self._currentSpeed = speed

    def set_wanted_speed(self, speed):
        self._wantedSpeed = speed

    def get_safe_speed(self):
        # code
        return None
