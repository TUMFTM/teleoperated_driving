import dataclasses
import math


TOD_PARK = 0
TOD_REVERSE = 1
TOD_NEUTRAL = 2
TOD_DRIVE = 3
TOD_SPORT = 4
TOD_HAUL = 5

AW_NEUTRAL = 1
AW_DRIVE = 2
AW_REVERSE = 20
AW_PARK = 22

AW_TURN_DISABLE = 1
AW_TURN_LEFT = 2
AW_TURN_RIGHT = 3
AW_HAZARD_DISABLE = 1
AW_HAZARD_ENABLE = 2


@dataclasses.dataclass(frozen=True)
class ConvertedCommand:
    velocity_mps: float
    steering_tire_angle_rad: float
    gear: int
    turn: int
    hazard: int


def convert_command(velocity, steering_wheel_angle, gear, indicator, steering_ratio):
    values = (velocity, steering_wheel_angle, steering_ratio)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("control values must be finite")
    if velocity < 0.0:
        raise ValueError("TOD velocity must be a non-negative magnitude")
    if steering_ratio <= 0.0:
        raise ValueError("steering_ratio must be positive")

    if gear == TOD_REVERSE:
        signed_velocity, aw_gear = -velocity, AW_REVERSE
    elif gear in (TOD_DRIVE, TOD_SPORT, TOD_HAUL):
        signed_velocity, aw_gear = velocity, AW_DRIVE
    elif gear == TOD_PARK:
        signed_velocity, aw_gear = 0.0, AW_PARK
    elif gear == TOD_NEUTRAL:
        signed_velocity, aw_gear = 0.0, AW_NEUTRAL
    else:
        raise ValueError(f"unsupported TOD gear: {gear}")

    if indicator == 0:
        turn, hazard = AW_TURN_DISABLE, AW_HAZARD_DISABLE
    elif indicator == 1:
        turn, hazard = AW_TURN_LEFT, AW_HAZARD_DISABLE
    elif indicator == 2:
        turn, hazard = AW_TURN_RIGHT, AW_HAZARD_DISABLE
    elif indicator == 3:
        turn, hazard = AW_TURN_DISABLE, AW_HAZARD_ENABLE
    else:
        raise ValueError(f"unsupported TOD indicator: {indicator}")

    return ConvertedCommand(
        velocity_mps=signed_velocity,
        steering_tire_angle_rad=steering_wheel_angle / steering_ratio,
        gear=aw_gear,
        turn=turn,
        hazard=hazard,
    )
