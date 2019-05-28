#!/usr/bin/env python

PACKAGE = "emergency_stop"


def _calculate_break_distance(current_speed, negative_acceleration):
    return (current_speed ** 2) / 2.0 * negative_acceleration


def _is_emergency_stop_necessary(break_distance, forward_minimum_distance, index, ranges):

    range_is_leq_break_distance_and_forward_minimum_distance = \
        ranges[index] <= break_distance + forward_minimum_distance

    range_is_greater_than_forward_minimum_distance = ranges[index] > forward_minimum_distance

    emergency_stop_needs_to_happen = \
        range_is_leq_break_distance_and_forward_minimum_distance \
        and range_is_greater_than_forward_minimum_distance

    return emergency_stop_needs_to_happen


def _first_forward_emergency_stop_evaluation(angle_front, angle_increment, break_distance,
                                             forward_minimum_distance, ranges):
    index = 0
    upper_bound = angle_front // angle_increment
    emergency_stop_does_not_need_to_happen = True

    while emergency_stop_does_not_need_to_happen:

        necessary = _is_emergency_stop_necessary(break_distance, forward_minimum_distance, index, ranges)

        if necessary:
            return True

        index += 1
        emergency_stop_does_not_need_to_happen = (index < len(ranges)) and (index < upper_bound)


class EmergencyStop:
    _emergencyStopConfig = None
    _currentSpeed = 0.0
    _wantedSpeed = 0
    _emergencyStop = True

    def __init__(self):
        pass

    def set_config(self, emergency_stop_config):
        self._emergencyStopConfig = emergency_stop_config

    # TODO: This method is doing too many things at once
    #  in the C++ implementation. Major refactoring needed
    def check_emergency_stop(self, laser_scan):

        break_distance = self._get_break_distance()

        plan_is_to_move_forward = self._wantedSpeed >= 0

        if plan_is_to_move_forward:
            self._move_forward(laser_scan, break_distance)
            return
        elif not plan_is_to_move_forward:
            self._move_backward(laser_scan, break_distance)
            return

        self._emergencyStop = False
        return

    def _get_break_distance(self):

        config = self._emergencyStopConfig

        current_break_distance_is_null_or_zero = bool(config.break_distance_based_on_speed) is False

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
        ranges = laser_scan.ranges

        first_evaluation = _first_forward_emergency_stop_evaluation

        emergency_stop_is_necessary = first_evaluation(angle_front, angle_increment, break_distance, forward_minimum_distance, ranges)

        # TODO: Add second evaluation, cpp line 44

        if emergency_stop_is_necessary:
            self._emergencyStop = True

        return



    def set_current_speed(self, speed):
        self._currentSpeed = speed

    def set_wanted_speed(self, speed):
        self._wantedSpeed = speed

    def get_safe_speed(self):
        # code
        return None

    def _move_backward(self, laser_scan, break_distance):
        pass
