import csv
import cv2
import math
import os
import queue # 데이터 선입 선출용
import tempfile
import threading
import traceback
import socket
import time
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

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}}) # 이거 수정하면 로그가 어떤게 출력되는지 바꿀 수 있음(CMD)

# 드론 와이파이 인지 아닌지 자동 판별
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try: # 시뮬레이션
    s.connect(('8.8.8.8', 0))
    # ip = s.getsockname()[0]
    DRONE_IP = "10.202.0.1"
except: # 드론 연결
    DRONE_IP = '192.168.42.1' # 드론

class StreamingExample(threading.Thread):
    # Create the olympe.Drone object from its IP address
    drone = olympe.Drone(DRONE_IP)
    tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")

    # print("Olympe streaming example output dir: {}".format(self.tempd))
    h264_frame_stats = []
    h264_stats_file = open(
        os.path.join(tempd, 'h264_stats.csv'), 'w+')
    h264_stats_writer = csv.DictWriter(
        h264_stats_file, ['fps', 'bitrate'])
    h264_stats_writer.writeheader()
    frame_queue = queue.Queue()
    flush_queue_lock = threading.Lock()

    def __init__(self):
        # self.drone = olympe.Drone(DRONE_IP)
        # 멀티 쓰레드
        for i_for in range(10):
            print(var.test)
        super().__init__()
        super().start()

    def start(self):
        # Connect the the drone
        print('Connecting to drone')
        print(var.test)
        self.drone.connect()
        print('Drone connected')

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()

    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnect()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def mov_gim(self,pch):
        self.drone(set_target(
            gimbal_id = 0,
            control_mode="position",
            yaw_frame_of_reference="none",   # None instead of absolute
            yaw = 0.0,
            pitch_frame_of_reference="absolute",
            pitch = pch,
            roll_frame_of_reference="none",     # None instead of absolute
            roll = 0.0,
        ))


    # def show_yuv_frame(self, window_name, yuv_frame):
    #     # the VideoFrame.info() dictionary contains some useful information
    #     # such as the video resolution
    #     info = yuv_frame.info()
    #     height, width = info["yuv"]["height"], info["yuv"]["width"]

    #     # yuv_frame.vmeta() returns a dictionary that contains additional
    #     # metadata from the drone (GPS coordinates, battery percentage, ...)

    #     # convert pdraw YUV flag to OpenCV YUV flag
    #     cv2_cvt_color_flag = {
    #         olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
    #         olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
    #     }[info["yuv"]["format"]]

    #     # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
    #     # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

    #     # Use OpenCV to convert the yuv frame to RGB
    #     cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
    #     # Use OpenCV to show this frame
    #     cv2.imshow(window_name, cv2frame)
    #     cv2.waitKey(1)  # please OpenCV for 1 ms...

    # def run(self):
    #     window_name = "Streaming"
    #     cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    #     main_thread = next(
    #         filter(lambda t: t.name == "MainThread", threading.enumerate())
    #     )
    #     while main_thread.is_alive():
    #         with self.flush_queue_lock:
    #             try:
    #                 yuv_frame = self.frame_queue.get(timeout=0.01)
    #             except queue.Empty:
    #                 continue
    #             try:
    #                 self.show_yuv_frame(window_name, yuv_frame)
    #             except Exception:
    #                 # We have to continue popping frame from the queue even if
    #                 # we fail to show one frame
    #                 traceback.print_exc()
    #             finally:
    #                 # Don't forget to unref the yuv frame. We don't want to
    #                 # starve the video buffer pool
    #                 yuv_frame.unref()
    #     cv2.destroyWindow(window_name)

