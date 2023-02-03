import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self,width = 1280,height = 720):
        print("DepthCamera")
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.depth_frame = None
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        # Start streaming
        self.pipeline.start(config)
        self.frames = None
        self.depth_frame = None
        self.align = rs.align(rs.stream.color)
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)
    def get_frame(self):
        
        self.align = rs.align(rs.stream.color)
        self.frames = self.pipeline.wait_for_frames()
        self.frames = self.align.process(self.frames)
        # self.frames = decimation.process(self.frames)
        # self.frames = depth_to_disparity.process(self.framese)
        # self.frames = spatial.process(self.frames)
        # self.frames = temporal.process(self.frames)
        # self.frames = disparity_to_depth.process(self.frames)
        # self.frames = hole_filling.process())
        self.depth_frame = self.frames.get_depth_frame()
        color_frame = self.frames.get_color_frame()

        #depth_image = np.asanyarray(self.depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        

        # Create alignment primitive with color as its target stream:
        
        

        # # Update color and depth frames:
        self.depth_frame = self.frames.get_depth_frame()
       # colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        # Show the two frames together:
        #images = np.hstack((color_frame, aligned_depth_frame))
        #plt.imshow(images)


        if not color_frame:
                return False, None
            
        return True, color_image

    def get_depth(self,x,y):
        
        depth = self.depth_frame.get_distance(x,y)
        return depth

    def release(self):
        self.pipeline.stop()