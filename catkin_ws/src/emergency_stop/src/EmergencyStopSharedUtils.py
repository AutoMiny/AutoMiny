def get_break_distance(config, current_speed):
    some_condition_i_dont_understand = config.break_distance_based_on_speed

    if some_condition_i_dont_understand:

        negative_acceleration = config.negative_acceleration

        break_distance = calculate_break_distance(current_speed, negative_acceleration)
    else:
        break_distance = config.break_distance

    return break_distance


def calculate_break_distance(current_speed, negative_acceleration):
    return (current_speed ** 2) / 2.0 * negative_acceleration
