import cv2
import pyrealsense2 as rs
import PoseModule as pm
import matlab.engine
import numpy as np
import socket
import time
import sys

class DepthCamera:
    def __init__(self, eng, camera_id, q1, q2, q3, coda_exit):
        self.init_camera(camera_id)      
        detector = pm.PoseDetector()
        self.acquire_data(detector, eng, camera_id, q1, q2, q3, coda_exit)
        
    
    def init_camera(self, camera_id):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        config.enable_device(camera_id)
        config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 90)
        config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 90)
        
        # Start streaming
        self.pipeline.start(config)
    
    def open_socket(self):
        self.active_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.active_socket.connect(('169.254.0.25', 2020))
        except:
            print('server not connected')
            self.active_socket=None
        
    
    def acquire_data(self, detector, eng, camera_id, q1, q2, q3, coda_exit):
        pTime = 0
        
        while True:
            if not coda_exit.empty():
                sys.exit(0)
            ret, depth_frame, color_frame, depth_intrin = self.get_frame()
            color_frame = detector.findPose(color_frame)
            
            coordinate3d=detector.getPosition(color_frame, depth_frame, depth_intrin)
            coordinate3d=coordinate3d.reshape(1,len(coordinate3d))
            
            camera_id_int=int(camera_id)
            coordinate3d=matlab.double(coordinate3d.tolist())
        
            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime
 
            cv2.putText(color_frame, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            cv2.imshow("Color_frame" +camera_id, color_frame)

            c_ass=eng.rototrasla_robot(coordinate3d,camera_id_int)

            c_str=camera_id+','+str(c_ass)
               
                
            if c_str[0:12]=='046122250287':
                    q1.put_nowait(c_str)

            elif c_str[0:12]=='046122251438':
                    q2.put_nowait(c_str)
           
            elif c_str[0:12]=='046122250173':
                    q3.put_nowait(c_str)
                    
 
            cv2.waitKey(1)
           
            
            if cv2.getWindowProperty("Color_frame" +camera_id, cv2.WND_PROP_VISIBLE) < 1:
               break

             
        coda_exit.put(0)
            
        sys.exit(0)
          

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        #Alignement
        align_to = rs.stream.color
        align = rs.align(align_to)
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        depth_frame= aligned_frames.get_depth_frame() #depth frame aligned
        color_frame= aligned_frames.get_color_frame()


        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        #depth and color intrinsic are equal because the frames are aligned

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        return True, depth_image, color_image, depth_intrin

    def release(self):
        self.pipeline.stop()