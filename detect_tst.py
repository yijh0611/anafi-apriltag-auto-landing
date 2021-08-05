#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.


# AnafiSensors.update - 뭔지 알아보기

import anafi_keyboard
import anafi_streaming

import csv
import cv2
import math
import os
import queue # 데이터 선입 선출용
import shlex
import subprocess
import tempfile
import threading
import traceback
import time
import socket
import pupil_apriltags
import numpy as np
import matplotlib.pyplot as plt

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from pynput.keyboard import Listener, Key, KeyCode
from collections import defaultdict
from enum import Enum
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.gimbal import set_target
from olympe.messages.move import extended_move_by

olympe.log.update_config({"loggers": {"olympe": {"level": "ERROR"}}}) # 이거 수정하면 로그가 어떤게 출력되는지 바꿀 수 있음(CMD)

# 드론 와이파이 인지 아닌지 자동 판별
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try: # 시뮬레이션
    s.connect(('8.8.8.8', 0))
    DRONE_IP= "10.202.0.1"
except: # 드론 연결
    DRONE_IP = '192.168.42.1' # 드론

parent_tst = anafi_streaming.StreamingExample

class child_tst(parent_tst):

    def __init__(self):
        pass
    
    def show_yuv_frame(self, window_name, yuv_frame):
        height = 720
        time_now = time.time()

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[1]

        # yuv to Grayscale
        self.cv2frame = yuv_frame.as_ndarray()[:-1][:720]

        txt_scrn = []
        txt_scrn.append('y : {}'.format(self.drn[1]))


        for i in range(len(txt_scrn)):
            cv2.putText(self.cv2frame, "{}".format(txt_scrn[i]), (50, 50 * (i + 1)), # 50,50
                        cv2.FONT_HERSHEY_COMPLEX, 1, (255, 50, 0), 2, lineType=cv2.LINE_AA)
        cv2.imshow(window_name, self.cv2frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...

    def run(self):
        window_name = "Streaming"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.show_yuv_frame(window_name, yuv_frame)
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
        cv2.destroyWindow(window_name)
  
if __name__ == "__main__":
    strm = child_tst()
    # Start the video stream
    strm.start()
    drone = strm.drone
    strm.gimbal_angle = 0

    drone(GPSFixStateChanged(_policy = 'wait'))

    print(DRONE_IP)

    # 짐벌 최고속도
    drone(
        olympe.messages.gimbal.set_max_speed(0, 10, 3600, 30, _timeout=10, _no_expect=False, _float_tol=(0.1, 0.1)) # 짐벌 id, yaw pitch roll
    ) # 세번째 숫자가 짐벌 속도 큰 숫자로 두기
    drone(olympe.messages.ardrone3.PilotingSettings.MinAltitude(0.1))

# # # # ###################### 여기서 부터 비행 시작 ####################

    print('Set gimbal to 0deg')
    drone(set_target(
            gimbal_id = 0,
            control_mode="position",
            yaw_frame_of_reference="none",   # None instead of absolute
            yaw = 0.0,
            pitch_frame_of_reference="absolute",
            pitch = strm.gimbal_angle, # 45.0
            roll_frame_of_reference="none",     # None instead of absolute
            roll = 0.0,
        )).wait().success()
    time.sleep(2)

    # # # 드론이 날고 있지 않을때 이륙
    if drone(FlyingStateChanged(state="hovering", _policy="check")): 
        print('Hovering')
    else :
        print('Takeoff')

        assert drone(
                TakeOff()
            ).wait().success()
    
    while 1:
        pass
