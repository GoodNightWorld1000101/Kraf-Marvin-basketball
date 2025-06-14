from color import Color

"RealsenseCamera constants"
DEBUG = True
RGB_WIDTH: int = 848
RGB_HEIGHT: int = 480
RGB_FRAMERATE: int = 60
DEPTH_WIDTH: int = 848
DEPTH_HEIGHT: int = 480
DEPTH_FRAMERATE: int = 60
EXPOSURE: float = 50.0
WHITE_BALACE: int = 3500
DEPTH_ENABLED: bool = True

""""Image processing constants"""
LINE_COLOR_THRESHOLD: int = 3
LINE_COLOR_SEQUENCE: list = [Color.WHITE, Color.BLACK]
RANDOM_BASKET_PIXELS: int = 60
MEMORY_DELAY: float = 0.2

"""seek constants"""
SEEK_ROTATION_SPEED = 4
SEEK_SPIN_TIME = 0.5
SEEK_LOOK_TIME = 0.35

"""meet constants"""
MEET_MOVEMENT_SPEED = 1.4#4
MEET_ROTATION_SPEED = 4
MEET_MIN_SPEED = 0
RESET_TIMER = 0.8
GRABBER_STOP_DELAY = 0.9
GRABBER_ADJUST_DELAY = 0.2

GRABBER_START = int(0.12 * RGB_HEIGHT)

"""lineup constants"""
LINEUP_ROTATION_SPEED = 2
LINEUP_SEEK_SPEED = 2
LINEUP_MOVEMENT_SPEED = 1

LINEUP_THRESHOLD = 0.065
LOW_CLAMP = 0.3

DISTANCE_TO_SPEED = 0.01

MIN_DISTANCE = 400
MAX_DISTANCE = 2600

"""yeet constants"""
YEET_ROTATION_SPEED = 3
THROW_START = 75#20
MIN_THROW_START = 0.005
MAX_THROW_START = 0.03

YEET_LIMIT = 0.1
SPINUP_TIME = 0.6

INTEGRAL_WEIGHT = 0.03
DERIVATIVE_WEIGHT = 3
I_CLAMP = 5

LINEAR_A = 0.3183
LINEAR_B = 860
SHORT_A = 0.485
SHORT_B = 660

"""robot properties constants"""
ROBOT_RADIUS = 0.1525
WHEEL_RADIUS = 0.033
GEARBOX_REDUCTION_RATIO = 18.75
ENCODER_EDGES = 64
PID_FREQUENCY = 100
GRABBER_SPEED = 30
REVERSE_GRABBER_SPEED = 3000
MINIMUM_THROWER_SPEED = 48
THROWER_SCREEN_POSITION = 0.517
THROWER_X = THROWER_SCREEN_POSITION * RGB_WIDTH


"websocket variables"
ID = "marvin"
IP = "ws://192.168.3.220:8111"
BYPASS_REFEREE = False
MAGENTA_BASKET = "magenta"
BLUE_BASKET = "blue"
TARGET_BASKET = BLUE_BASKET
