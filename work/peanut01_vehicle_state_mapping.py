TOD_GEAR_PARK = 0
TOD_GEAR_REVERSE = 1
TOD_GEAR_NEUTRAL = 2
TOD_GEAR_DRIVE = 3

TOD_INDICATOR_OFF = 0
TOD_INDICATOR_LEFT = 1
TOD_INDICATOR_RIGHT = 2
TOD_INDICATOR_BOTH = 3


def map_gear(report):
    if report == 22:
        return TOD_GEAR_PARK
    if report in (20, 21):
        return TOD_GEAR_REVERSE
    if report == 1:
        return TOD_GEAR_NEUTRAL
    if 2 <= report <= 19 or report in (23, 24):
        return TOD_GEAR_DRIVE
    return TOD_GEAR_PARK


def map_indicator(turn_report, hazard_report):
    if hazard_report == 2:
        return TOD_INDICATOR_BOTH
    if turn_report == 2:
        return TOD_INDICATOR_LEFT
    if turn_report == 3:
        return TOD_INDICATOR_RIGHT
    return TOD_INDICATOR_OFF
