#!/usr/bin/env python

PACKAGE = "emergency_stop"


def _calculate_break_distance(current_speed, negative_acceleration):
    return (current_speed ** 2) / 2.0 * negative_acceleration


class EmergencyStop:
    _emergencyStopConfig = None
    _currentSpeed = 0.0
    _wantedSpeed = 0
    _emergencyStop = True

    def __init__(self):
        pass

    def set_config(self, emergencyStopConfig):
        self._emergencyStopConfig = emergencyStopConfig

    # TODO: This method is doing too many things at once
    #  in the C++ implementation. Major refactoring needed
    def check_emergency_stop(self, laser_scan):

        break_distance = self._get_break_distance()

        plan_is_to_move_forward = self._wantedSpeed >= 0

        if plan_is_to_move_forward:
            self._move_forward(laser_scan, break_distance)
        else:
            self._move_backward(laser_scan, break_distance)

        return True

    def _get_break_distance(self):

        config = self._emergencyStopConfig

        current_break_distance_is_null_or_zero = config.break_distance_based_on_speed is False

        if current_break_distance_is_null_or_zero:

            current_speed = self._currentSpeed
            negative_acceleration = config.negative_acceleration

            break_distance = _calculate_break_distance(current_speed, negative_acceleration)
        else:
            break_distance = config.break_distance

        return break_distance

    def _move_forward(self, laser_scan, break_distance):

        config = self._emergencyStopConfig
        angle_front = config.angle_front / 2.0
        forward_minimum_distance = config.forward_minimum_distance

        angle_increment = laser_scan.angle_increment

        upper_bound = angle_front // angle_increment

        ranges = laser_scan.ranges

        index = 0

        adjustments_need_to_take_place = (index < len(ranges)) and (index < upper_bound)

        # TODO: Line 23 in EmergencyStop.cpp
        while adjustments_need_to_take_place:


        for index, range in enumerate(ranges):

    def set_current_speed(self, speed):
        self._currentSpeed = speed

    def set_wanted_speed(self, speed):
        self._wantedSpeed = speed

    def get_safe_speed(self):
        # code
        return None

    def _move_backward(self, laser_scan, break_distance):
        pass
