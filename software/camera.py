import pyrealsense2 as rs
import numpy as np
import cv2
import conf

#Basic camera interface that can be extended to use different API-s. Realsense example below
class ICamera:
    def open(self):
        pass
    def close(self):
        pass
    def has_depth_capability(self) -> bool:
        pass
    def get_color_frame(self):
        pass
    def get_frames(self):
        pass


# Camera implementation using the pyrealsense2 provided API 
class RealsenseCamera(ICamera):
    def __init__(self, 
                rgb_width = None, 
                rgb_height = None,
                rgb_framerate = None,
                depth_width = None, 
                depth_height = None,
                depth_framerate = None,
                exposure = None, 
                white_balance = None,
                depth_enabled = None):

        self.rgb_width = conf.RGB_WIDTH if rgb_width is None else rgb_width
        self.rgb_height = conf.RGB_HEIGHT if rgb_height is None else rgb_height
        self.rgb_framerate = conf.RGB_FRAMERATE if rgb_framerate is None else rgb_framerate
        self.exposure = conf.EXPOSURE if exposure is None else exposure
        self.white_balace = conf.WHITE_BALACE if white_balance is None else white_balance

        self.depth_width = conf.DEPTH_WIDTH if depth_width is None else depth_width
        self.depth_height = conf.DEPTH_HEIGHT if depth_height is None else depth_height
        self.depth_framerate = conf.DEPTH_FRAMERATE if depth_framerate is None else depth_framerate

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.rgb_width, self.rgb_height, rs.format.bgr8, self.rgb_framerate)
        
        self.depth_enabled = conf.DEPTH_ENABLED if depth_enabled is None else depth_enabled
        if self.depth_enabled:
            self.config.enable_stream(rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, self.depth_framerate)
            
        self.align = rs.align(rs.stream.color)
        self.depth_scale = -1

            
    def open(self):
        profile = self.pipeline.start(self.config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_exposure, False)
        color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        color_sensor.set_option(rs.option.white_balance, self.white_balace)
        color_sensor.set_option(rs.option.exposure, self.exposure)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def close(self):
        self.pipeline.stop()
    
    def get_color_frame(self):
        frames = self.pipeline.wait_for_frames()
        return np.asanyarray(frames.get_color_frame().get_data())
    
    def has_depth_capability(self) -> bool:
        return self.depth_enabled

    def get_frames(self, aligned = False):
        frames = self.pipeline.wait_for_frames()
        if aligned:
            frames = self.align.process(frames)
        #processed_frame = frames.get_color_frame() # not in use for now
        return np.asanyarray(frames.get_color_frame().get_data()), np.asanyarray(frames.get_depth_frame().get_data())


# resolution numbers are sensitive with openCV. Implement a resolution setting mechanism here or use the default of the webcam to
# get a more robust solution
class OpenCVCamera(ICamera):
    def __init__(self, 
                RGB_WIDTH_CV = 848, 
                RGB_HEIGHT_CV  = 480,
                RGB_FRAMERATE_CV = 60,
                ID_CV = 0):

        self.rgb_width = RGB_WIDTH_CV
        self.rgb_height = RGB_HEIGHT_CV
        self.rgb_framerate = RGB_FRAMERATE_CV

        self.camera_id = ID_CV
        self.camera_stream = None

    def open(self):
        self.camera_stream = cv2.VideoCapture(self.camera_id)


    def close(self):
        self.camera_stream.release()


    def has_depth_capability(self) -> bool:
        return False

    def get_color_frame(self):
        ret, frame = self.camera_stream.read()
        return frame


    def get_frames(self):
        ret, frame = self.camera_stream.read()
        return frame, np.zeros(frame.shape, dtype=int)