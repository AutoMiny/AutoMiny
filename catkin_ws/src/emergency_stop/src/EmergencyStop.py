#!/usr/bin/env python
from EmergencyStopUtils import first_forward_emergency_stop_evaluation, calculate_break_distance, \
    second_forward_emergency_stop_evaluation, move_forward

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

    def check_emergency_stop(self, laser_scan):

        config = self._emergencyStopConfig
        break_distance = self._get_break_distance()

        plan_is_to_move_forward = self._wantedSpeed >= 0

        if plan_is_to_move_forward:
            emergency_stop_is_necessary = move_forward(config, laser_scan, break_distance)
            if emergency_stop_is_necessary:
                self._emergencyStop = True
                return

        else:
            emergency_stop_is_necessary = self._move_backward(config, laser_scan, break_distance)

            if emergency_stop_is_necessary:
                self._emergencyStop = True
                return

        self._emergencyStop = False

    def _get_break_distance(self):
        config = self._emergencyStopConfig

        some_condition_i_dont_understand = config.break_distance_based_on_speed

        if some_condition_i_dont_understand:

            current_speed = self._currentSpeed
            negative_acceleration = config.negative_acceleration

            break_distance = calculate_break_distance(current_speed, negative_acceleration)
        else:
            break_distance = config.break_distance

        return break_distance

    def set_current_speed(self, speed):
        self._currentSpeed = speed

    def set_wanted_speed(self, speed):
        self._wantedSpeed = speed

    def get_safe_speed(self):
        # code
        return None

    def _move_backward(self, config, laser_scan, break_distance):
        return True or False
