"""Modules used to transmit and create movement data"""
import struct
import math
import serial
import serial.tools.list_ports
from conf import ROBOT_RADIUS, WHEEL_RADIUS, ENCODER_EDGES, GEARBOX_REDUCTION_RATIO, PID_FREQUENCY, LINEAR_A, LINEAR_B, MINIMUM_THROWER_SPEED,SHORT_A,SHORT_B

# Motor angles for omni motion movement (constants, no need to change)
MOTOR1_ANGLE = 0
MOTOR2_ANGLE = (4*math.pi) / 3
MOTOR3_ANGLE = (2*math.pi) / 3

class IRobotMotion:
    """Base class for robot motion"""
    def open(self):
        """does nothing"""
        pass
    def close(self):
        """does nothing"""
        pass
    def move(self, x_speed, y_speed, rot_speed):
        """does nothing"""
        pass

class OmniMotionRobot(IRobotMotion):
    """Controls communication with mainboard"""
    def __init__(self, polarity):
        self.polarity = polarity
        self.thrower_speed = 0
        self.serial = 0
        self.depth_memory = []
        self.robot_radius = ROBOT_RADIUS
        self.meters_to_mb_units = 1 / (2 * math.pi * WHEEL_RADIUS) * (ENCODER_EDGES * GEARBOX_REDUCTION_RATIO) / PID_FREQUENCY
        self.grabber_speed = 0
        self.ball_loaded = 0


    def scan_ports(self):
        """Find the port that is connected to robot mainboard"""
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if port.description == "STM32 Virtual ComPort":
                port = str(port.device)
                return port
        return "ERROR"

    def open(self):
        try:
            self.serial = serial.Serial(self.scan_ports(), 9600, 8,"N",1,0)
        except(OSError):
            pass

    def close(self):
        self.serial.close()
    def read_IR_sensor(self):
        buffer = self.serial.read(size=16)
        if len(buffer) == 15:
            # hack fix bacause mainboard only sent half of delimiter
            buffer += b'\x00'
            self.ball_loaded = (struct.unpack('hhhhhhBH', buffer))[6]

    def move(self, x_speed:float, y_speed:float, rot_speed:float):
        robot_speed = math.sqrt(x_speed*x_speed + y_speed*y_speed)
        robot_direction_angle = math.atan2(y_speed,x_speed)

        robot_rotation_speed = ROBOT_RADIUS * rot_speed

        wheel_1 = robot_speed*math.cos(robot_direction_angle - MOTOR1_ANGLE + robot_rotation_speed) + robot_rotation_speed
        wheel_2 = robot_speed*math.cos(robot_direction_angle - MOTOR2_ANGLE + robot_rotation_speed) + robot_rotation_speed
        wheel_3 = robot_speed*math.cos(robot_direction_angle - MOTOR3_ANGLE + robot_rotation_speed) + robot_rotation_speed

        lin_motor_1 = int(wheel_1*self.meters_to_mb_units)
        lin_motor_2 = int(wheel_2*self.meters_to_mb_units)
        lin_motor_3 = int(wheel_3*self.meters_to_mb_units)
        clamped_thrower_speed = max(MINIMUM_THROWER_SPEED, self.thrower_speed)
        #struct.pack('<hhhHHHBH', speed1, speed2, speed3, thrower_speed, grabber, angle, disable_failsafe, 0xAAAA)
        self.serial.write(struct.pack('<hhhHHHBH', lin_motor_1, lin_motor_2, lin_motor_3, clamped_thrower_speed, self.grabber_speed, 0, 0, 0xAAAA))

    def set_thrower_speed(self, speed: int):
        """sets the thrower speed that is called with every move_omni call"""
        self.thrower_speed = int(speed)

    def update_polarity(self,new_polarity):
        """Changes the variable that controls the robot's polarity"""
        self.polarity = new_polarity

    def conf_PID(self, p_gain: int, i_gain: int):
        """configures mainboard PID"""
        self.serial.write(struct.pack('<iiH', p_gain, i_gain, 0xAABB))

    def thrower_speed_calc(self, dist):
        """function to calculate thrower speed according to distance using average filter"""
        if dist > 1200:
            formula = LINEAR_A*dist + LINEAR_B
        else:
            formula = SHORT_A*dist + SHORT_B
        #formula = 531.2592 + 0.06192992*dist + 0.00002583662*dist**2
        self.thrower_speed = int(formula)
