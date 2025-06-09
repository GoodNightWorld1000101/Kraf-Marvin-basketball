import time
import struct
import logging
import threading
import cv2
import image_processor
import camera
import motion
from conf import *
from functions import time_passed, clamp, thrower_speed_calc
from state import RobotState
from referee import Referee

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
    end = 0
    aligned = False

    #Initializing objects and giving startup signal to thrower
    processor.start()
    controller.open()
    #Startup signal to thrower
    controller.set_thrower_speed(2000)

    # Behaviour variables
    thrower_timer = 0
    grabber_timer = 0
    spinup_timer = 0
    error_sum = 0
    # Records Which way a ball dissapeared out of the frame
    # Rotation direction of SEEK is multiplied by ball_memory
    ball_memory = 1
    # Records Which way the basket dissapeared out of the frame
    # Sideways speed of ORBIT is multiplied by basket_memory
    basket_memory = 1
    frame = 0
    start_fps=time.time()

    # Websocket variables and initialising
    ws_thread = threading.Thread(target=referee.listen)
    ws_thread.daemon = True  # This ensures the thread will close when the main program exits
    ws_thread.start()

    ir_thread = threading.Thread(target=controller.read_IR_sensor())
    ir_thread.daemon = True  # This ensures the thread will close when the main program exits
    ir_thread.start()
    # Target ball and basket info variables
    closest_ball_x, closest_ball_y, basket_x = None, None, None
    thrower_speed = 2000
    thrower_loaded = False
    try:
        while True:
            print("aligned:",aligned,"state:",state)
             #Has argument aligned_depth that enables depth frame to color frame alignment.
            #Costs performance
            processed_data = processor.process_frame(aligned_depth=aligned)
            
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
                
                if controller.ball_loaded and state != RobotState.YEET and not thrower_loaded:
                        if time_passed(grabber_timer, GRABBER_STOP_DELAY):
                            controller.grabber_speed = 0
                            thrower_loaded = True
                            state = RobotState.LINEUP

                if state == RobotState.TEST:
                    n +=1
                    print(n)
                    controller.read_IR_sensor()
                    if n >= 120:
                        flip = -flip
                        n = 0
                    if flip == 1:
                        controller.serial.write(struct.pack('<hhhHHHBH', 0, 0, 0, 48, 30, 0, 0, 0xAAAA))
                    else:
                        controller.serial.write(struct.pack('<hhhHHHBH', 0, 0, 0, 48, 3000, 0, 0, 0xAAAA))
                
                if state == RobotState.SEEK:
                    if controller.ball_loaded:
                        state = RobotState.LINEUP
                    elif len(processed_data.balls) >= 1:
                        state = RobotState.MEET
                    else:
                        controller.move(0, 0, SEEK_ROTATION_SPEED * ball_memory, 0)
                
                if state == RobotState.MEET:
                    controller.grabber_speed = GRABBER_SPEED

                    if len(processed_data.balls) == 0 and time_passed(grabber_timer, GRABBER_STOP_DELAY):
                        controller.grabber_speed = 0
                        state = RobotState.SEEK
                    
                    elif len(processed_data.balls) >= 1:
                        ball_from_center = (half_width - closest_ball_x) / (half_width)
                        ball_distance = height - closest_ball_y
                        ball_direction = MEET_ROTATION_SPEED * ball_from_center

                        # speed is lowered the closer the ball gets to the bottom of the screen
                        # It is currently done using the function y = (2**4x) / 16
                        speed_regulator = 2**(4 * (ball_distance / height)) / 16
                        forward_speed = MEET_MOVEMENT_SPEED * speed_regulator

                        if ball_distance <= GRABBER_START:
                            controller.grabber_speed = GRABBER_SPEED
                            grabber_timer = time.time()

                        controller.move(0, forward_speed, ball_direction, 0)

                if state == RobotState.LINEUP:
                    ready = True
                    clamped_speed = 0
                    basket_direction = 0
                    if target_basket.exists:
                        
                        thrower_from_basket = (THROWER_X - basket_x) / half_width
                        #print("thrower_from_basket:",thrower_from_basket)
                        basket_direction = LINEUP_ROTATION_SPEED * thrower_from_basket

                        if target_basket.distance <= MIN_DISTANCE:
                            #print("[MIN] dist:", target_basket.distance)
                            location_error = target_basket.distance - MIN_DISTANCE
                            move_speed = location_error * DISTANCE_TO_SPEED
                            clamped_speed = clamp(move_speed, -LOW_CLAMP, -LINEUP_MOVEMENT_SPEED)
                            ready = False

                        elif target_basket.distance >= MAX_DISTANCE:
                            location_error = MAX_DISTANCE - target_basket.distance
                            move_speed = location_error * DISTANCE_TO_SPEED
                            clamped_speed = clamp(move_speed, LOW_CLAMP, LINEUP_MOVEMENT_SPEED)
                            ready = False

                        elif thrower_from_basket <= LINEUP_THRESHOLD and ready:
                            lineup_timer = time.time()
                            aligned = True
                            state = RobotState.YEET
                    else:
                        basket_direction = LINEUP_ROTATION_SPEED * basket_memory
                        ready = False

                    controller.move(0, clamped_speed, basket_direction, 0)

                if state == RobotState.YEET:
                    aligned = True
                    #NOTE: For one frame the frame isn't depth aligned when switcing to YEET
                    if target_basket.exists:
                        
                        thrower_from_basket = (THROWER_X - basket_x) / half_width
                        error_sum += thrower_from_basket
                        yeet_integral = error_sum * INTEGRAL_WEIGHT
                        basket_direction = SEEK_ROTATION_SPEED * (thrower_from_basket + yeet_integral)
                        if time_passed(lineup_timer, YEET_DELAY):
                            if abs(thrower_from_basket) <= THROW_START_THRESHOLD:
                                error_sum = 0
                                if time_passed(spinup_timer, SPINUP_TIME):
                                    controller.grabber_speed = GRABBER_SPEED
                                    if controller.ball_loaded:
                                        thrower_timer = time.time()
                                #print(target_basket.distance)
                                thrower_speed = thrower_speed_calc(target_basket.distance)
                            else:
                                spinup_timer = time.time()
                    else:
                        thrower_speed = 1952
                        state = RobotState.SEEK
                        error_sum = 0
                        aligned = False
                    
                    if not controller.ball_loaded and time_passed(thrower_timer, YEET_DELAY):
                        controller.grabber_speed = 0
                        aligned = False
                        thrower_loaded = False
                        state = RobotState.SEEK
                        error_sum = 0
                    controller.move(0, 0, basket_direction, thrower_speed)
                else:
                # If Referee tells robot to stop
                #controller.set_thrower_speed(0)
                    controller.grabber_speed = 0

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start_fps)
                start_fps = end
                print("STATE:",state,"fps:",fps)
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
        cv2.destroyAllWindows()
        processor.stop()
        controller.serial.write(struct.pack('<hhhHHHBH', 0, 0, 0, 0, 0, 0, 0, 0xAAAA))
        controller.close()

main_loop()