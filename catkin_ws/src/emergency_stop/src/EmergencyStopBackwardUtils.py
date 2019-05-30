def move_backward(config, laser_scan, break_distance):
    angle_back = config.angle_back / 2.0
    ranges = laser_scan.ranges
    angle_increment = laser_scan.angle_increment
    upper_bound = len(ranges)
    index = (upper_bound // 2) - (angle_back // angle_increment)
    end = (upper_bound // 2) + (angle_back // angle_increment)
    reverse_minimum_distance = config.reverse_minimum_distance

    emergency_stop_is_necessary = backward_emergency_stop_evaluation(index, end, upper_bound, break_distance,
                                                                     reverse_minimum_distance, ranges)

    if emergency_stop_is_necessary:
        return True

    return False


def backward_emergency_stop_evaluation(index, end, upper_bound, break_distance, reverse_minimum_distance, ranges):
    emergency_stop_does_not_need_to_happen = True

    while emergency_stop_does_not_need_to_happen:

        necessary = is_emergency_stop_necessary_when_moving_backward(index, break_distance, reverse_minimum_distance,
                                                                     ranges)
        if necessary:
            return True

        index += 1
        emergency_stop_does_not_need_to_happen = (index < end) and (index < upper_bound)

    return False


def is_emergency_stop_necessary_when_moving_backward(index, break_distance, reverse_minimum_distance,
                                                     ranges):
    range_is_leq_sum_of_break_distance_and_reverse_min_distance = \
        ranges[index] <= break_distance + reverse_minimum_distance

    range_is_greater_than_reverse_min_distance = ranges[index] > reverse_minimum_distance

    return range_is_leq_sum_of_break_distance_and_reverse_min_distance and range_is_greater_than_reverse_min_distance

