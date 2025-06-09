import random
import segment
import _pickle as pickle
import numpy as np
import cv2
import color as c
from conf import RANDOM_BASKET_PIXELS, LINE_COLOR_THRESHOLD, LINE_COLOR_SEQUENCE

class Object():
    def __init__(self, x = -1, y = -1, size = -1, distance = -1, exists = False):
        self.x = x
        self.y = y
        self.size = size
        self.distance = distance
        self.exists = exists

    def __str__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)

    def __repr__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)


# results object of image processing. contains coordinates of objects and frame data used for these results
class ProcessedResults():

    def __init__(self, 
                balls=[], 
                basket_b = Object(exists = False), 
                basket_m = Object(exists = False), 
                color_frame = [],
                depth_frame = [],
                fragmented = [],
                debug_frame = []) -> None:


        self.balls = balls
        self.basket_b = basket_b
        self.basket_m = basket_m
        self.color_frame = color_frame
        self.depth_frame = depth_frame
        self.fragmented = fragmented

        # can be used to illustrate things in a separate frame buffer
        self.debug_frame = debug_frame


#Main processor class. processes segmented information
class ImageProcessor():
    def __init__(self, camera, color_config = "colors/colors.pkl", debug = False):
        self.camera = camera
        self.color_config = color_config
        with open(self.color_config, 'rb') as conf:
            self.colors_lookup = pickle.load(conf)
            self.set_segmentation_table(self.colors_lookup)

        self.fragmented	= np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

        self.t_balls = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_b = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_m = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_lines = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype = np.uint8)
        self.debug = debug
        self.debug_frame = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        
        #erode/dialate variables
        self.erode_circle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        self.dialate_circle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        self.erode_square = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        self.dialate_square = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))

    def set_segmentation_table(self, table):
        segment.set_table(table)

    def start(self):
        self.camera.open()

    def stop(self):
        self.camera.close()

    def point_detection(self,side_x:int,side_y:int, middle_x:int,middle_y:int)->dict:
        """Arguments
        function that returns a dictionary of the coordinates of 3 circles:LEFT,RIGHT,MIDDLE"""
        masks = {"RIGHT":None,"LEFT":None,"MIDDLE":None}
        center_left = (width/2 - side_x, side_y)
        center_right = (width/2 + side_x, side_y)
        center_middle = (middle_x, middle_y)
        
        radius = 10
        color = (0, 255, 0)  
        thickness = -1       
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, center_left, radius, color, thickness)
        blob_pixels = np.column_stack(np.where(mask == 255))
        masks["LEFT"]=(blob_pixels)

        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, center_right, radius, color, thickness)
        blob_pixels = np.column_stack(np.where(mask == 255))
        masks["RIGHT"]=blob_pixels

        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, center_middle, radius, color, thickness)
        blob_pixels = np.column_stack(np.where(mask == 255))
        masks["MIDDLE"]=blob_pixels

        return masks

    def is_ball_too_close_to_wall(self,ball_x,ball_y,depth_frame,color_frame):
        if ball_y > 50:
            ball_y-=50
        if depth_frame[ball_y][ball_x]<500:
            if color_frame[ball_y][ball_x] == 5 or color_frame[ball_y][ball_x]==6 or color_frame[ball_y][ball_x]==2 or color_frame[ball_y][ball_x]==3:
                return False
            else:
                return True
        else:
            return True
            

    def ball_inbounds_check(self, fragments, y_coords:list, x_coords:list):
        """Function that takes the fragmented frame and 2D matrix of line Y and X coordinates 
        and returns if balls are inside court or not. It does so by comparing a constant to the 
        last few colours detected."""
        #Small fix: Sometimes the line is given as empty, don't know why.
        #happens when ball leaves frame or is close to screen bottom.
        if y_coords.size == 0:
            return True
        #Reversing the order so we start analyzing from the robot to the ball
        y_coords = list(reversed(y_coords))
        x_coords = list(reversed(x_coords))
        
        #For logic reasons, gotta initialise color_data as first pixel in line.
        working_color = fragments[y_coords[0], x_coords[0]]
        working_counter = 0
        color_memory = []

        #How many datapoints we need to compare
        color_sequence_size = len(LINE_COLOR_SEQUENCE)
        for index, y in enumerate(y_coords):
            #Get colour of each pixel in line
            pixel_color = fragments[y, x_coords[index]]
            if index == len(y_coords) - 1:
                pixel_color = None
            #Count how many pixels are the same colour
            if pixel_color == working_color:
                working_counter += 1
            else:
                #If change in colour
                if working_counter >= LINE_COLOR_THRESHOLD:
                    #If new color is present enough
                    if len(color_memory) == 0 or color_memory[-1] != int(working_color):
                        # And wasn't added previously, add color to list.
                        color_memory.append(int(working_color))
                        color_memory_size = len(color_memory)
                        comparison_list = color_memory[color_memory_size - color_sequence_size:]
                        #Compare list and sequence constant to see if ball is in or out of court
                        if color_memory_size >= color_sequence_size and comparison_list == list(map(int, LINE_COLOR_SEQUENCE)):
                            return False
                working_color = pixel_color
                working_counter = 1
        return True
    


    def analyze_balls(self, t_balls, fragments,depth_frame) -> list:
        t_balls = cv2.erode(t_balls,self.erode_circle,iterations = 1)
        t_balls = cv2.dilate(t_balls,self.dialate_circle,iterations = 5)
        contours, hierarchy = cv2.findContours(t_balls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        balls = []

        for contour in contours:

            # ball filtering logic goes here. Example includes filtering by size and an example how to get pixels from
            # the bottom center of the fram to the ball

            size = cv2.contourArea(contour)
            
            if size <= 40:
                continue
            else:
                x, y, w, h = cv2.boundingRect(contour)

                ys	= np.array(np.arange(y + h, self.camera.rgb_height), dtype=np.uint16)
                xs	= np.array(np.linspace(x + w/2, self.camera.rgb_width / 2, num=len(ys)), dtype=np.uint16)

                obj_x = int(x + (w/2))
                obj_y = int(y + (h/2))
                obj_dst = obj_y
                #balls.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))
                if self.ball_inbounds_check(fragments, ys, xs) and self.is_ball_too_close_to_wall(obj_x,obj_y,depth_frame,fragments):
                    balls.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))
                    self.debug_frame[ys, xs] = [0, 0, 0]
                if self.debug:
                    cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)
        balls.sort(key= lambda x: x.distance)
        return balls

    def analyze_baskets(self, t_basket, depth,debug_color = (0, 255, 255)) -> list:
        t_basket = cv2.erode(t_basket,self.erode_square,iterations = 1)
        contours, hierarchy = cv2.findContours(t_basket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        baskets = []

        for contour in contours:
            distances = []
            # basket filtering logic goes here. Example includes size filtering of the basket

            size = cv2.contourArea(contour)

            if size > 100:
                #continue

                x, y, w, h = cv2.boundingRect(contour)

                obj_x = int(x + (w/2))
                obj_y = int(y + (h/2))
                # Choosing random pixels from basket, and averaging their distances to get a more accurate measurement
                mask = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
                cv2.drawContours(mask, [contour], -1, 255, -1)
                blob_pixels = np.column_stack(np.where(mask == 255))
                random_pixels = random.sample(list(blob_pixels), RANDOM_BASKET_PIXELS)
                for (random_x, random_y) in random_pixels:
                    distances.append(int(depth[random_x, random_y]))
                obj_dst = sum(distances) / len(distances)
                baskets.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))
        baskets.sort(key= lambda x: x.size)
        basket = next(iter(baskets), Object(exists = False))
        if self.debug:
            if basket.exists:
                cv2.circle(self.debug_frame,(basket.x, basket.y), 20, debug_color, -1)

        return basket

    def get_frame_data(self, aligned_depth = False):
        if self.camera.has_depth_capability():
            return self.camera.get_frames(aligned = aligned_depth)
        else:
            return self.camera.get_color_frame(), np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def process_frame(self, aligned_depth = False) -> ProcessedResults:
        color_frame, depth_frame = self.get_frame_data(aligned_depth = aligned_depth)
        segment.segment(color_frame, self.fragmented, self.t_balls, self.t_basket_m, self.t_basket_b)

        if self.debug:
            self.debug_frame = np.copy(color_frame)

        balls = self.analyze_balls(self.t_balls, self.fragmented,depth_frame)
        basket_b = self.analyze_baskets(self.t_basket_b,depth_frame, debug_color=c.Color.BLUE.color.tolist())
        basket_m = self.analyze_baskets(self.t_basket_m, depth_frame,debug_color=c.Color.MAGENTA.color.tolist())

        return ProcessedResults(balls = balls, 
                                basket_b = basket_b, 
                                basket_m = basket_m, 
                                color_frame=color_frame, 
                                depth_frame=depth_frame, 
                                fragmented=self.fragmented, 
                                debug_frame=self.debug_frame)
