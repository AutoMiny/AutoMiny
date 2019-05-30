def move_forward(config, laser_scan, break_distance):
    angle_front = config.angle_front / 2.0
    forward_minimum_distance = config.forward_minimum_distance

    angle_increment = laser_scan.angle_increment
    ranges = laser_scan.ranges

    emergency_stop_is_necessary = first_forward_emergency_stop_evaluation(angle_front, angle_increment,
                                                                          break_distance,
                                                                          forward_minimum_distance, ranges)
    if emergency_stop_is_necessary:
        return True

    emergency_stop_is_necessary = second_forward_emergency_stop_evaluation(angle_front, angle_increment,
                                                                           break_distance,
                                                                           forward_minimum_distance, ranges)
    if emergency_stop_is_necessary:
        return True

    return False


def first_forward_emergency_stop_evaluation(angle_front, angle_increment, break_distance,
                                            forward_minimum_distance, ranges):
    index = 0
    upper_bound = angle_front // angle_increment
    emergency_stop_does_not_need_to_happen = True

    while emergency_stop_does_not_need_to_happen:

        necessary = is_emergency_stop_necessary_when_moving_forward(break_distance, forward_minimum_distance, index,
                                                                    ranges)

        if necessary:
            return True

        index += 1
        emergency_stop_does_not_need_to_happen = (index < len(ranges)) and (index < upper_bound)

    return False


def second_forward_emergency_stop_evaluation(angle_front, angle_increment, break_distance,
                                             forward_minimum_distance, ranges):
    upper_bound = len(ranges)
    index = upper_bound - 1 - (angle_front // angle_increment)
    emergency_stop_does_not_need_to_happen = True

    while emergency_stop_does_not_need_to_happen:

        necessary = is_emergency_stop_necessary_when_moving_forward(break_distance, forward_minimum_distance, index,
                                                                    ranges)

        if necessary:
            return True

        index += 1
        emergency_stop_does_not_need_to_happen = (index < upper_bound)

    return False


def is_emergency_stop_necessary_when_moving_forward(break_distance, forward_minimum_distance, index, ranges):
    range_is_leq_break_distance_and_forward_minimum_distance = \
        ranges[index] <= break_distance + forward_minimum_distance

    range_is_greater_than_forward_minimum_distance = ranges[index] > forward_minimum_distance

    emergency_stop_needs_to_happen = \
        range_is_leq_break_distance_and_forward_minimum_distance \
        and range_is_greater_than_forward_minimum_distance

    return emergency_stop_needs_to_happen
