"""modules to get ball and target data, move robot and  keep track of states"""
import time
import struct
import logging
import threading
import cv2
import image_processor
import camera
import motion
from conf import *
from functions import time_passed, clamp, calc_turn_speed, calc_move_speed
from state import RobotState
from referee import Referee

referee_logger = logging.getLogger(__name__)
seek_logger = logging.getLogger(__name__)
meet_logger = logging.getLogger(__name__)
orbit_logger = logging.getLogger(__name__)
yeet_logger = logging.getLogger(__name__)
logging.basicConfig(filename='debug.log', encoding='utf-8', level=logging.CRITICAL)

def main_loop():
    """the main loop responsible for robot behaviour"""
    #variables in use to keep track of states
    state = RobotState.SEEK

    #main objects that provide data and control of robot
    controller = motion.OmniMotionRobot(1)
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=DEBUG)
    referee = Referee(ID,IP)

    #frame info variables
    width = cam.rgb_width
    half_width = width / 2
    height = cam.rgb_height
    aligned = False

    #Initializing objects and giving startup signal to thrower
    processor.start()
    controller.open()

    #Startup signal to thrower
    controller.set_thrower_speed(2000)
    controller.set_thrower_speed(0)

    # timer variables
    seek_spin_timer = time.time()
    seek_look_timer = time.time()

    thrower_timer = 0
    grabber_timer = 0
    spinup_timer = 0
    lineup_timer = 0

    #lineup and yeet variables
    error_sum = 0
    yeet_integral = 0
    yeet_derivative = 0
    thrower_from_basket = 0

    # Records Which way a ball dissapeared out of the frame
    # Rotation direction of SEEK is multiplied by ball_memory
    ball_memory = 1

    # Records Which way the basket dissapeared out of the frame
    # Sideways speed of ORBIT is multiplied by basket_memory
    basket_memory = -1

    # FPS calculation variables
    frame = 0
    start_fps=time.time()
    end = 0

    # Websocket variables and initialising
    ws_thread = threading.Thread(target=referee.listen)
    ws_thread.daemon = True  # This ensures the thread will close when the main program exits
    ws_thread.start()

    # Target ball and basket info variables
    closest_ball_x, closest_ball_y, basket_x = None, None, None
    try:
        while True:
            #Has argument aligned_depth that enables depth frame to color frame alignment.
            #Costs performance
            processed_data = processor.process_frame(aligned_depth=aligned)

            # Check if ball is in robot (controller.ball_loaded == 1 if true, 0 if false)
            controller.read_IR_sensor()
            referee_logger.info("CONNECTED: %s", referee.connected)
            referee_logger.info("MATCH IN PROGRESS: %s", referee.match_in_progress)
            referee_logger.info("TARGET BASKET: %s", referee.basket_color)

            if referee.match_in_progress or BYPASS_REFEREE:
                # Grabbing target ball data
                if len(processed_data.balls) >= 1:
                    closest_ball_x = (processed_data.balls[-1]).x
                    closest_ball_y = (processed_data.balls[-1]).y
                # Remembering which way a ball is in case of no targets visible
                # FIX: added small delay to ignore flying ball right after throw
                if len(processed_data.balls) >= 2 and time_passed(thrower_timer, MEMORY_DELAY):
                    if processed_data.balls[-2].x < half_width:
                        ball_memory = -1
                    else:
                        ball_memory = 1

                # Grabbing data about the target basket
                # If target basket is magenta, then set target as basket_m else target blue
                if BYPASS_REFEREE:
                    target_basket = processed_data.basket_m if TARGET_BASKET == MAGENTA_BASKET else processed_data.basket_b
                else:
                    target_basket = processed_data.basket_m if referee.basket_color == MAGENTA_BASKET else processed_data.basket_b
                # Remembering which way a basket is in case of no basket visible
                if target_basket.exists:
                    basket_x = target_basket.x
                    if basket_x < half_width:
                        basket_memory = 1
                    else:
                        basket_memory = -1

                #SEEK state rotates the robot until it finds a ball, then passes to MEET
                if state == RobotState.SEEK:
                    controller.set_thrower_speed(48)
                    seek_logger.info("SEEK DIRECTION: %s", ball_memory)
                    seek_logger.info("BALLS DETECTED: %s", len(processed_data.balls))

                    if controller.ball_loaded:
                        aligned = True
                        state = RobotState.LINEUP

                    elif len(processed_data.balls) >= 1:
                        aligned = False
                        state = RobotState.MEET

                    else:
                        controller.grabber_speed = REVERSE_GRABBER_SPEED
                        if time_passed(seek_spin_timer, SEEK_SPIN_TIME):
                            controller.move(0, 0, -SEEK_ROTATION_SPEED * ball_memory)
                            if time_passed(seek_look_timer, SEEK_LOOK_TIME):
                                seek_spin_timer = time.time()
                        else:
                            controller.move(0, 0, 0)
                            seek_look_timer = time.time()

                #MEET state approaches the ball and picks it up
                if state == RobotState.MEET:
                    forward_speed = 0
                    ball_direction = 0
                    if controller.ball_loaded:
                        if time_passed(grabber_timer, GRABBER_STOP_DELAY):
                            if time_passed(grabber_timer, GRABBER_STOP_DELAY + GRABBER_ADJUST_DELAY):
                                controller.grabber_speed = 0
                                aligned = True
                                state = RobotState.LINEUP
                            else:
                                controller.grabber_speed = REVERSE_GRABBER_SPEED
                        else:
                            controller.grabber_speed = GRABBER_SPEED
                    elif len(processed_data.balls) == 0 and time_passed(grabber_timer, RESET_TIMER):
                        seek_spin_timer = time.time()
                        seek_look_timer = time.time()
                        controller.grabber_speed = 0
                        aligned = False
                        state = RobotState.SEEK

                    elif len(processed_data.balls) >= 1:
                        ball_from_center = (half_width - closest_ball_x) / (half_width)
                        ball_distance = height - closest_ball_y
                        ball_direction = calc_turn_speed(ball_from_center, MEET_ROTATION_SPEED)
                        # speed is lowered the closer the ball gets to the bottom of the screen
                        # It is currently done using the function y = (2**4x) / 16
                        forward_speed = calc_move_speed(ball_distance / height, MEET_MOVEMENT_SPEED)
                        forward_speed = clamp(forward_speed, MEET_MIN_SPEED, None)
                        print(forward_speed)
                        if ball_distance <= GRABBER_START:
                            controller.grabber_speed = GRABBER_SPEED
                            grabber_timer = time.time()
                        meet_logger.info("BALL DIRECTION: %s", ball_direction)
                        meet_logger.info("FORWARD SPEED: %s", forward_speed)
                    controller.move(0, forward_speed, ball_direction)

                if state == RobotState.LINEUP:
                    controller.conf_PID(3000, 200)
                    ready = True
                    clamped_speed = 0
                    basket_direction = 0
                    if not controller.ball_loaded:
                        seek_spin_timer = time.time()
                        seek_look_timer = time.time()
                        controller.grabber_speed = 0
                        aligned = False
                        state = RobotState.SEEK
                        controller.conf_PID(2000, 50)
                    elif target_basket.exists:
                        aligned = True
                        thrower_from_basket = (THROWER_X - basket_x) / half_width
                        basket_direction = calc_turn_speed(thrower_from_basket, LINEUP_ROTATION_SPEED)

                        if MIN_DISTANCE is not None and target_basket.distance <= MIN_DISTANCE:
                            location_error = target_basket.distance - MIN_DISTANCE
                            move_speed = location_error * DISTANCE_TO_SPEED
                            clamped_speed = clamp(move_speed, -LOW_CLAMP, -LINEUP_MOVEMENT_SPEED)

                            ready = False

                        elif MAX_DISTANCE is not None and target_basket.distance >= MAX_DISTANCE:
                            location_error = target_basket.distance - MAX_DISTANCE
                            move_speed = location_error * DISTANCE_TO_SPEED
                            clamped_speed = clamp(move_speed, LOW_CLAMP, LINEUP_MOVEMENT_SPEED)
                            ready = False

                        elif thrower_from_basket <= LINEUP_THRESHOLD and ready:
                            lineup_timer = time.time()
                            spinup_timer = time.time()
                            aligned = True
                            state = RobotState.YEET
                    
                    else:
                        basket_direction = LINEUP_SEEK_SPEED * basket_memory
                        ready = False
                    controller.move(0, clamped_speed, basket_direction)

                #YEET state throws the ball into basket
                if state == RobotState.YEET:
                    #NOTE: For one frame the frame isn't depth aligned when switcing to YEET
                    if target_basket.exists:
                        yeet_logger.info("BASKET_DIRECTION: %s", basket_direction)
                        basket_direction = YEET_ROTATION_SPEED * (thrower_from_basket + yeet_integral + yeet_derivative)
                        prev_thrower_from_basket = thrower_from_basket
                        thrower_from_basket = (THROWER_X - basket_x) / half_width
                        print(target_basket.distance, controller.thrower_speed)
                        if time_passed(lineup_timer, YEET_LIMIT):
                            #INTEGRAL
                            error_sum += thrower_from_basket
                            yeet_integral = clamp(error_sum * INTEGRAL_WEIGHT, -I_CLAMP, I_CLAMP)
                            #DERIVATIVE
                            yeet_derivative = (thrower_from_basket - prev_thrower_from_basket) * DERIVATIVE_WEIGHT
                            throw_start_threshold = clamp(THROW_START/clamp(target_basket.distance, 1, None), MIN_THROW_START, MAX_THROW_START)
                            print(thrower_from_basket)
                            if abs(thrower_from_basket) <= throw_start_threshold:
                                if time_passed(spinup_timer, SPINUP_TIME):
                                    controller.grabber_speed = GRABBER_SPEED
                                    if controller.ball_loaded:
                                        thrower_timer = time.time()
                                controller.thrower_speed_calc(target_basket.distance)
                            else:
                                spinup_timer = time.time()                            
                    else:
                        controller.conf_PID(2000, 50)
                        seek_spin_timer = time.time()
                        seek_look_timer = time.time()
                        controller.grabber_speed = 0
                        yeet_integral = 0
                        error_sum = 0
                        aligned = False
                        state = RobotState.SEEK
                        
                        
                    
                    if not controller.ball_loaded and time_passed(thrower_timer, YEET_LIMIT):
                        error_sum = 0
                        controller.conf_PID(2000, 50)
                        seek_spin_timer = time.time()
                        seek_look_timer = time.time()
                        controller.grabber_speed = 0
                        yeet_integral = 0
                        error_sum = 0
                        aligned = False
                        state = RobotState.SEEK
                    controller.move(0, 0, basket_direction)
            else:
                # If Referee tells robot to stop
                controller.set_thrower_speed(0)
                controller.grabber_speed = 0
                controller.move(0,0,0)
            #print(state)

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start_fps)
                start_fps = end
                print("FPS:",fps)
            if DEBUG:
                
                debug_frame = processed_data.debug_frame
                #cv2.circle(debug_frame,(int(half_width),height-30),10,(255,0,0),2)
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break

    except KeyboardInterrupt:
        print("closing....")

    finally:
        controller.conf_PID(2000, 50)
        cv2.destroyAllWindows()
        processor.stop()
        controller.grabber_speed = 0
        controller.move(0,0,0)
        controller.serial.write(struct.pack('<hhhHHHBH', 0, 0, 0, 0, 0, 0, 0, 0xAAAA))
        controller.close()

main_loop()
