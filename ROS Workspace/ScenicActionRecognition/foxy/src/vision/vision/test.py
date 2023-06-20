from typing import List
import cv2
import numpy as np
from collections import deque
import threading
import time

from face_emo.EmotionProcessor import EmotionProcessor

TARGET_FRAME_H = 480

def threaded(fn):
    def wrapper(*args, **kwargs):
        threading.Thread(target=fn, args=args, kwargs=kwargs).start()

    return wrapper

class VisionNode:

    __alive: bool
    #__publisher: VisionPublisher

    def __init__(self):

        # Conect to webcam
        self.__cap = cv2.VideoCapture(0)   

        # Create face emotion process
        self.__emo_detector = EmotionProcessor()
        # Create buffer to store at most 1 frame
        self.__face_buffer = deque(maxlen=1)
        self.__alive = True

        #self.__publisher = VisionPublisher()

        # Start threads
        self.get_camera_frames()

    @threaded
    def get_camera_frames(self):
        ret = True
        while(ret and self.__alive):
            # Get frame
            ret, frame = self.__cap.read()
            if frame is None:
                continue
            # Resize to target shape
            h, w, _  = frame.shape
            frame = cv2.resize(frame, (TARGET_FRAME_H, int(TARGET_FRAME_H*h/w)))
            s = time.time()
            result = self.__emo_detector(frame)
            e = time.time()
            print(f"Face emotion model took {e-s}")
            time.sleep(0.01)


def main(args=None):
    #rclpy.init(args=args)
    node = VisionNode()


if __name__ == "__main__":
    main()
