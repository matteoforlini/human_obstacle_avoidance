import cv2
import mediapipe as mp
import time
import numpy as np
import pyrealsense2 as rs


class PoseDetector:

    def __init__(self):
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(
                   
                   static_image_mode=False,
                   model_complexity=1,
                   smooth_landmarks=True,
                   enable_segmentation=False,
                   smooth_segmentation=False,
                   min_detection_confidence=0.85,
                   min_tracking_confidence=0.85)

    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        #print(results.pose_landmarks)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
        return img

    def getPosition(self, image, depth_frame, depth_intrin, draw=True):
        lmList= []
        dlist= []
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                vis=lm.visibility
                if cx<w and cy<h and vis>0.85:
                    distance = depth_frame[cy, cx]
                    lmList.append([id, cx, cy, distance])
                    result = rs.rs2_deproject_pixel_to_point(
                                depth_intrin, [cx, cy], distance)
                    if not result==[0,0,0]: 
                        try:
                            dlist.append([id, vis, round(result[0]), round(result[1]), round(result[2])])
                            
                        except:
                            dlist.append([id,1,1,1])
   
                if draw:
                    cv2.circle(image, (cx, cy), 5, (255, 80, 0), cv2.FILLED)
        coordinate_array=np.asarray(dlist)
        coordi_flatten=coordinate_array.flatten()        
        return coordi_flatten
