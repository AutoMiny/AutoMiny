def calculate_break_distance(current_speed, negative_acceleration):
    return (current_speed ** 2) / 2.0 * negative_acceleration


def is_emergency_stop_necessary_when_moving_forward(break_distance, forward_minimum_distance, index, ranges):
    range_is_leq_break_distance_and_forward_minimum_distance = \
        ranges[index] <= break_distance + forward_minimum_distance

    range_is_greater_than_forward_minimum_distance = ranges[index] > forward_minimum_distance

    emergency_stop_needs_to_happen = \
        range_is_leq_break_distance_and_forward_minimum_distance \
        and range_is_greater_than_forward_minimum_distance

    return emergency_stop_needs_to_happen


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


def second_forward_emergency_stop_evaluation(angle_front, angle_increment, break_distance,
                                             forward_minimum_distance, ranges):
    upper_bound = len(ranges)
    index = upper_bound - 1 - (angle_front / angle_increment)
    emergency_stop_does_not_need_to_happen = True

    while emergency_stop_does_not_need_to_happen:

        necessary = is_emergency_stop_necessary_when_moving_forward(break_distance, forward_minimum_distance, index,
                                                                    ranges)

        if necessary:
            return True

        index += 1
        emergency_stop_does_not_need_to_happen = (index < upper_bound)


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


def move_backward(config, laser_scan, break_distance):
    angle_back = config.angle_back / 2.0
    ranges = laser_scan.ranges
    angle_increment = laser_scan.angle_increment
    upper_bound = len(ranges)
    index = (upper_bound // 2) - (angle_back // angle_increment)
    end = (upper_bound // 2) + (angle_back // angle_increment)
    reverse_minimum_distance = config.reverse_minimum_distance

    emergency_stop_is_necessary = backward_emergency_stop_evaluation(index, end, upper_bound, break_distance,
                                                                     reverse_minimum_distance)

    if emergency_stop_is_necessary:
        return True

    return False


def is_emergency_stop_necessary_when_moving_backward():
    return True or False


def backward_emergency_stop_evaluation(index, end, upper_bound, break_distance, reverse_minimum_distance):

    emergency_stop_does_not_need_to_happen = True

    while emergency_stop_does_not_need_to_happen:

        necessary = is_emergency_stop_necessary_when_moving_backward(params)

        if necessary:
            return True

        index += 1
        emergency_stop_does_not_need_to_happen = True

    return True or False


def get_break_distance(config, current_speed):
    some_condition_i_dont_understand = config.break_distance_based_on_speed

    if some_condition_i_dont_understand:

        negative_acceleration = config.negative_acceleration

        break_distance = calculate_break_distance(current_speed, negative_acceleration)
    else:
        break_distance = config.break_distance

    return break_distance
