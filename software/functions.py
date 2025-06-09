from time import time
from conf import LINEAR_A, LINEAR_B
from math import log10
def time_passed(input_timer, time_to_wait):
    """
    Function to check if a set amount of time has passed.
    The function starts counting time from [input_timer]
    and if more than [time_to_wait] passes, returns true.
    """
    if time() - input_timer >= time_to_wait:
        return True
    else:
        return False

def clamp(value, low, high):
    """
    Funtion to clamp values between [high] and [low], only above [low] or only below [high].
    Clamp type determined by None values.
    """
    if high is not None and low is not None:
        clamped = min(max(low, value), high)
    elif high is None and low is not None:
        clamped = max(low, value)
    else:
        clamped = min(high, value)
    return clamped

def calc_turn_speed(error, constant):
    "returns values that turn more agressively than when using linear values"
    if error > 0:
        return constant * (1 - (1 / (5 * error + 1)))
    elif error < 0:
        return constant * ((1 / (-5 * error + 1)) - 1)
    else:
        return 0

def calc_move_speed(distance, constant):
    "returns values that move more agressively than when using linear values"
    #return distance * constant
    return(clamp(1 + log10(distance)/1.5, 0, None) * constant)
