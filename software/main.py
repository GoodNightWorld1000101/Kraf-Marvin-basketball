"""modules to get ball and target data, move robot and  keep track of states"""
import time
import logging
import threading
import image_processor
import camera
import motion
import cv2
from conf import *
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
    end = 0
    aligned = False

    #Initializing objects and giving startup signal to thrower
    processor.start()
    controller.open()
    controller.conf_PID(2000, 50)
    #Startup signal to thrower
    controller.set_thrower_speed(2000)
    controller.set_thrower_speed(48)

    # Behaviour variables
    thrower_timer = 0
    # Records Which way a ball dissapeared out of the frame
    # Rotation direction of SEEK is multiplied by ball_memory
    ball_memory = 1
    # Records Which way the basket dissapeared out of the frame
    # Sideways speed of ORBIT is multiplied by basket_memory
    basket_memory = 1

    # FPS calculation variables
    frame = 0
    start_fps=time.time()

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
            referee_logger.info("CONNECTED: %s", referee.connected)
            referee_logger.info("MATCH IN PROGRESS: %s", referee.match_in_progress)
            referee_logger.info("TARGET BASKET: %s", referee.basket_color)
            if referee.match_in_progress or BYPASS_REFEREE:
                # Grabbing target ball data
                if len(processed_data.balls) >= 1:
                    closest_ball_x = (processed_data.balls[-1]).x
                    closest_ball_y = (processed_data.balls[-1]).y
                # Remembering which way a ball is in case of no targets visible
                if len(processed_data.balls) >= 2:
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
                    if len(processed_data.balls) >= 1:
                        state = RobotState.MEET
                    else:
                        controller.move(0, 0, SEEK_ROTATION_SPEED * ball_memory)

                #MEET state approaches the ball and corrects it's speed and trajectory of approach
                if state == RobotState.MEET:

                    if len(processed_data.balls) == 0:
                        state = RobotState.SEEK

                    elif (height - closest_ball_y) <= ORBIT_START_DISTANCE:
                        state = RobotState.ORBIT

                    elif len(processed_data.balls) >= 1:
                        ball_direction = MEET_ROTATION_SPEED * (half_width - closest_ball_x) / (half_width)
                        # speed is lowered linearly the closer the ball gets to the bottom of the screen
                        # It is currently done using the function y = 1.25*x - 0.25
                        speed_regulator = APPROACH_FUNCTION_A * ((height - closest_ball_y) / height) + APPROACH_FUNCTION_B
                        forward_speed = ROBOT_SPEED * speed_regulator
                        controller.move(0, forward_speed, ball_direction)
                        meet_logger.info("BALL DIRECTION: %s", ball_direction)
                        meet_logger.info("SPEED REGULATOR: %s", speed_regulator)
                        meet_logger.info("FORWARD SPEED: %s", forward_speed)

                #ORBIT state makes bot orbit around the ball until it lines up with a basket
                if state == RobotState.ORBIT:
                    if len(processed_data.balls) >= 1:
                        orbit_direction = 1
                        basket_from_center = 0
                        slow_down = 1
                        
                        # How centered the ball is to the thrower
                        # (0 when perfect, < 0 too right, > 0 too left)
                        ball_from_thrower = ((THROWER_SCREEN_POSITION * width - closest_ball_x) / half_width)

                        # Where basket is relative to thrower
                        if target_basket.exists:
                            basket_from_center = (basket_x - THROWER_SCREEN_POSITION * width)
                            # Which way to orbit around ball, determined by location of basket when visible
                            # If basket_from_center is negative, the basket is on the left
                            if basket_from_center <= 0:
                                orbit_direction = 1
                            else:
                                orbit_direction = -1

                            ball_from_basket = abs((closest_ball_x - basket_x) / half_width)
                            slow_down = max(ORBIT_MIN_SLOW_DOWN, min(1, ball_from_basket))

                            # Check if ball is centered and lined up with target basket
                            if abs(ball_from_thrower) <= THROW_START_THRESHOLD and ball_from_basket <= THROW_START_THRESHOLD:
                                state = RobotState.YEET
                                aligned = True
                                thrower_timer = time.time()

                        if closest_ball_y < ORBIT_STOP_DISTANCE:
                            state = RobotState.MEET
                        else:
                            # If basket isn't visible, operate by memory
                            orbit_direction = basket_memory

                        # How close the ball is to the desired distance when orbiting
                        # (0 when perfect, -1 on bottom edge of screen, 6.384 on top edge of screen)
                        ball_from_orbit_target = -(ORBIT_DISTANCE - (height - closest_ball_y)) / ORBIT_DISTANCE

                        # Forward speed is used to keep a certain distance from ball (min )
                        # (0 = perfect distance, < 0 too close, > 0 too far)
                        forward_speed = ORBIT_SPEED * ball_from_orbit_target
                        
                        # Sideways speed changes to keep the ball in the center of the screen
                        # For a smooth approach, the further away the ball is, the faster the bot moves sideways to
                        # EXPLANATION:   [Constant]         *       [If further away, move faster]          *             [which way]  *  [line up]
                        sideways_speed = ORBIT_SPEED * max(1, ball_from_orbit_target * ORBIT_DISTANCE_SPEEDUP_WEIGHT) * orbit_direction * slow_down

                        # EXPERIMENTAL: Add speed when ball starts moving out of frame to improve tracking
                        # EXPLANATION :     [speed] * [> 1 when ball too far in moving direction]
                        # sideways_speed = sideways_speed * max(1, 1 - ball_from_thrower)

                        # Rotation speed is used to keep the ball centered on the thrower
                        # EXPLANATION:      [turn by ball pos.]  *  [Change sensitivity]   /   [distance to ball]
                        rotation_speed = (ball_from_thrower * ORBIT_ROTATION_SPEED_WEIGHT) / max(1, ball_from_orbit_target)

                        controller.move(sideways_speed, forward_speed, rotation_speed)
                        
                        orbit_logger.info("BASKET EXISTS: %s", target_basket.exists)
                        orbit_logger.info("ORBIT DIRECTION: %s", orbit_direction)
                        orbit_logger.info("BASKET FROM CENTER: %s", basket_from_center)
                        orbit_logger.info("BALL FROM ORBIT TARGET: %s", ball_from_orbit_target)
                        orbit_logger.info("BALL FROM THROWER: %s", ball_from_thrower)
                        orbit_logger.info("FORWARD SPEED: %s", forward_speed)
                        orbit_logger.info("SIDEWAYS SPEED: %s", sideways_speed)
                        orbit_logger.info("ROTATION SPEED: %s", rotation_speed)
                    else:
                        state = RobotState.SEEK

                #YEET state activates thrower and drives toward the basket (ball should be in the way).
                if state == RobotState.YEET:
                    controller.conf_PID(4000, 200)
                    if target_basket.exists:
                        basket_direction = SEEK_ROTATION_SPEED * (THROWER_SCREEN_POSITION * width - basket_x) / (half_width)
                    else:
                        basket_direction = 0
                    yeet_logger.info("BASKET_DIRECTION: %s", basket_direction)

                    #NOTE: For one frame the frame isn't depth aligned when switcing to YEET
                    controller.thrower_speed_calc(target_basket.distance)
                    # EXPLANATION: [fixing robot left drift], [drive into ball], [aim for the basket]
                    controller.move(YEET_MOVEMENT_CORRECTION,   YEET_SPEED,     basket_direction)
                    if (time.time() - thrower_timer) >= BLIND_THROW_DURATION:
                        # Once done throwing, erase average distance buffer
                        controller.depth_memory = []
                        # Stop the thrower
                        controller.set_thrower_speed(48)
                        controller.conf_PID(2000, 50)
                        state = RobotState.SEEK
                # UNSTUCK state will be used when robot appears to be not moving or stuck
                if state == RobotState.UNSTUCK:
                    pass
            else:
                # If Referee tells robot to stop
                controller.conf_PID(2000, 50)
                controller.set_thrower_speed(48)

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start_fps)
                start_fps = end
            if DEBUG:
                debug_frame = processed_data.debug_frame
                
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break

    except KeyboardInterrupt:
        print("closing....")

    finally:
        cv2.destroyAllWindows()
        processor.stop()
        controller.close()

main_loop()
