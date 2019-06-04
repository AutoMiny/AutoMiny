#!/usr/bin/env python
from EmergencyStopSharedUtils import get_break_distance
from EmergencyStopForwardUtils import moving_forward_evaluation
from EmergencyStopBackwardUtils import moving_backward_evaluation

PACKAGE = "emergency_stop"


class EmergencyStop:
    _emergencyStopConfig = None
    _currentSpeed = 0.0
    _wantedSpeed = 0
    _emergencyStop = True

    def __init__(self):
        pass

    def set_config(self, emergency_stop_config):
        self._emergencyStopConfig = emergency_stop_config

    def set_current_speed(self, speed):
        self._currentSpeed = speed

    def set_wanted_speed(self, speed):
        self._wantedSpeed = speed

    def get_safe_speed(self):

        if self._emergencyStop:
            speed_command = 0
        else:
            speed_command = self._wantedSpeed
        return speed_command

    def check_emergency_stop(self, laser_scan):

        config = self._emergencyStopConfig
        current_speed = self._currentSpeed
        break_distance = get_break_distance(config, current_speed)

        plan_is_to_move_forward = self._wantedSpeed >= 0

        if plan_is_to_move_forward:
            emergency_stop_is_necessary = moving_forward_evaluation(config, laser_scan, break_distance)
            if emergency_stop_is_necessary:
                self._emergencyStop = True
                return

        else:
            emergency_stop_is_necessary = moving_backward_evaluation(config, laser_scan, break_distance)

            if emergency_stop_is_necessary:
                self._emergencyStop = True
                return

        self._emergencyStop = False
