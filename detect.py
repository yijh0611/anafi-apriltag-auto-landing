#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.


# AnafiSensors.update - 뭔지 알아보기

if __name__ == "__main__":
    import anafi_streaming

import anafi_keyboard
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
import argparse

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

# # 와이파이 신호 확인용
# parser = argparse.ArgumentParser(description='Display WLAN signal strength.')
# parser.add_argument(dest='interface', nargs='?', default='wlp1s0', # wlan0
#                     help='wlan interface (default: wlan0)')
# args = parser.parse_args()

# 와이파이 신호 확인용
class w_lan():
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='Display WLAN signal strength.')
        self.parser.add_argument(dest='interface', nargs='?', default='wlp1s0', # wlan0
                            help='wlan interface (default: wlan0)')
        self.args = self.parser.parse_args()
        self.signal = 0.0
        self.signal_full = 70.0

    def get_signal(self):

        while True:
            cmd = subprocess.Popen('iwconfig %s' % self.args.interface, shell=True,
                                stdout=subprocess.PIPE)
            for line in cmd.stdout:

                line_str = line.decode('utf-8')

                if 'Link Quality' in line_str: # in line
                    signal = line_str.lstrip(' ').split('/')[0].split('=')[1]
                    signal_full = line_str.lstrip(' ').split('/')[1].split(' ')[0]
                    print(f'signal strength is : {signal} out of {signal_full}')
                    self.signal = float(signal)
                    self.signal_full = float(signal_full)
                elif 'Not-Associated' in line_str:
                    print('No signal')
                    self.signal = 0.0

# olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}}) # 이거 수정하면 로그가 어떤게 출력되는지 바꿀 수 있음(CMD)
olympe.log.update_config({"loggers": {"olympe": {"level": "ERROR"}}}) # 이거 수정하면 로그가 어떤게 출력되는지 바꿀 수 있음(CMD)

# 드론 와이파이 인지 아닌지 자동 판별
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try: # 시뮬레이션
    s.connect(('8.8.8.8', 0))
    # ip = s.getsockname()[0]
    DRONE_IP= "10.202.0.1"
except: # 드론 연결
    DRONE_IP = '192.168.42.1' # 드론

class Variables():
    def __init__(self):
        self.test = 0

        self.time_tag = 0 # 태그 놓치는 시간이 길이지면 착륙
        self.van = time.time()
        self.gogh = time.time()
        self.tag_detected = []
        self.gimbal_angle = 0
        self.gim_ang = [0,0,0] # xyz
        self.drn_prev = np.array([[0.0],[0.0],[0.0]]) # xyz
        self.drn = np.array([[0.0],[0.0],[0.0]]) # xyz
        self.drn_i = np.array([[0.0],[0.0],[0.0]]) # xyz
        self.d_drn = np.array([[0.0],[0.0],[0.0]]) # 거리 차이
        self.spd_f_graph = np.array([0.0]) # 임시로 만든 어레이
        self.graph_time = np.array([time.time()])
        self.reset = 0 # 임시

        self.drn_vel = np.array([[0.0],[0.0],[0.0],[0.0]]) # 속도 차이 Pitch, Roll
        self.drn_vel_prev = np.array([[0.0],[0.0],[0.0],[0.0]]) # 속도 차이 Pitch, Roll
        self.error_vel = np.array([[0.0],[0.0],[0.0],[0.0]]) # 속도 차이 Pitch, Roll
        self.error_vel_prev = np.array([[0.0],[0.0],[0.0],[0.0]]) # 속도 차이 Pitch, Roll
        self.d_error_vel = np.array([[0.0],[0.0],[0.0],[0.0]]) # 속도 차이 Pitch, Roll
        self.i_error_vel = np.array([[0.0],[0.0],[0.0],[0.0]]) # 속도 차이 Pitch, Roll
        
        self.can_i_move = 0
        self.can_i_move_original = 0
        self.start_log = 0 # 1 when logging, 0 when not logging

        ##### 속도 제어를 위한 변수들
        self.vel = [[0],[0],[0],[0]] # forward, right, up, clockwise
        self.vel_prev = [[0],[0],[0],[0]] # forward, right, up, clockwise
        self.tilt = [[0],[0],[0],[0]] # pitch, roll, throttle, clockwise
        # 시뮬레이션 용 - 밑에서 다시 정의하기 때문에 0이어도 괜찮다.
        self.kf_sim_tilt = np.array([0,0,0]) # forward PDI
        self.kr_sim_tilt = np.array([0,0,0]) # right PDI
        self.kt_sim_tilt = np.array([0,0,0])
        self.kc_sim_tilt = np.array([0,0,0])
        # 실제 드론 용 - 밑에서 다시 정의하기 때문에 0이어도 괜찮다.
        self.kf_real_tilt = np.array([0,0,0]) # forward PDI
        self.kr_real_tilt = np.array([0,0,0]) # right PDI
        self.kt_real_tilt = np.array([0,0,0])
        self.kc_real_tilt = np.array([0,0,0])

        self.newton = time.time()
        self.hdg = 0
        self.hdg_prev = 0 # 이렇게 정의하면 처음에 시작할때 문제가 될 수도 있다. - 바로 이륙안하니까 괜찮을지도?
                    # hdg - hdg_prev를 해야되는데, 90-0 이런식으로 될 수도 있어서 입력이 크게 들어갈 수도 있다.
        # newton = time.time()
        self.isacc = self.newton + 0.1

        ##### 속도 제어를 위한 변수 여기까지

        self.center_x = -1
        self.center_y = -1
        self.detector = pupil_apriltags.Detector()
        self.camera_params = [920.6649,920.4479,652.8415,355.9656]
        pass

if __name__ == '__main__':
    parent = anafi_streaming.StreamingExample
    class ChildStreaming(anafi_streaming.StreamingExample):
        
        def __init__(self):
            self.detector = pupil_apriltags.Detector()
            self.drone = parent.drone
            pass

        def log(self):
            '''
            드론의 속도 정보를 계속 받아서 저장하는 코드이다.
            병렬로 돌고 있기 때문에 로그 기록하는 속도가 빠르다.
            파일 용량이 너무 커지게 된다면 시간 간격을 두고 로그를 기록하게 수정하기.

            로그 나중에 클래스 따로 만들어서 위치 바꾸기
            '''
            log_path = '/home/aims/git_anafi/anafi-apriltag-auto-landing/logs/' # 로그 저장되는 위치
            log_array = np.array([['time','speed_north','speed_east','speed_forward','speed_right','speed_up',
                                    'North','East','Forward','Right','Altitude','Altitude_sensor','Heading','lat2','lat3']])
            if DRONE_IP == "10.202.0.1": # 시뮬
                name = time.strftime('log_%Y-%m-%d %H:%M:%s_sim', time.localtime(time.time()))
            else : # 실제
                name = time.strftime('log_%Y-%m-%d %H:%M:%s_real', time.localtime(time.time()))
            self.time_delay = time.time()
            self.time_log_prev = time.time()
            self.location = np.array([0,0,0,0,0])
            print(f'start log {name}')

            while 1:
                # 그냥 하면 1초만에 메모리를 다 써버려서 딜레이를 넣어야 함.
                if time.time() > self.time_delay + 1/50 : # 초당 50번 기록 - 기록개수 제한 있는 듯(얼마인지는 잘 모르겠다.). 더 필요할거 같으면 나중에 로그파일 더 늘릴 수 있게 코드 수정하기
                    # 배열 크기 늘려서 state까지 기록 할 수 있게 수정하기. - 비행중인지 착륙중인지 이륙중인지 등
                    self.time_delay = time.time()

                    drone_gps2 = self.drone.get_state(olympe.messages.ardrone3.PilotingState.GpsLocationChanged)
                    lat2 = drone_gps2['latitude']
                    drone_gps3 = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PositionChanged)
                    lat3 = drone_gps3['latitude']

                    # airspeed = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AirSpeedChanged)
                    # airspeed = airspeed['airSpeed']

                    drone_speed = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)
                    drone_hdg = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged) # rad
                    hdg = drone_hdg['yaw'] # rad
                    spdx = drone_speed['speedX'] # N
                    spdy = drone_speed['speedY'] # E
                    spdz = drone_speed['speedZ'] * (-1) # 상
                    spdf = spdx * math.cos(hdg) + spdy * math.sin(hdg) # forward
                    spdr = spdx * math.sin(hdg) * (-1) + spdy * math.cos(hdg) # right
                    time_now = time.strftime('%H:%M:%s', time.localtime(time.time()))

                    drone_poi = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AltitudeAboveGroundChanged)
                    alt = drone_poi['altitude']
                    
                    # 거리 추정
                    dt = time.time() - self.time_log_prev
                    self.time_log_prev = time.time()
                    var.location = var.location + dt * np.array([spdx,spdy,spdz,spdf,spdr])

                    tmp = np.array([[time_now,spdx,spdy,spdf,spdr,spdz,
                                    var.location[0],var.location[1],var.location[3],var.location[4],var.location[2],alt,hdg * 180/math.pi,lat2,lat3]])

                    # 로그 합치기
                    log_array = np.append(log_array,tmp,axis = 0)

                    # 로그 모양 확인
                    a,b = log_array.shape

                    # 로그 기록
                    if a % 50 == 0: # 1초에 한번씩 기록
                        np.savetxt(f'{log_path}{name}.csv', log_array, fmt = '%s', delimiter = ',')
                        print('saved',a)

        # def yuv_frame_cb(self, yuv_frame):
        #     """
        #     This function will be called by Olympe for each decoded YUV frame.

        #         :type yuv_frame: olympe.VideoFrame
        #     """
        #     yuv_frame.ref()
        #     self.frame_queue.queue.clear() # 딜레이 줄이기 위함 - 나중에 queue 안쓰는 방향으로 코드 수정
        #     self.frame_queue.put_nowait(yuv_frame)

        def spd_realtime(self):
            while 1:
                drone_speed = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)
                drone_hdg = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged) # rad
                hdg = drone_hdg['yaw'] # rad
                spdx = drone_speed['speedX'] # N
                spdy = drone_speed['speedY'] # E
                spdz = drone_speed['speedZ'] * (-1) # 상
                spdf = spdx * math.cos(hdg) + spdy * math.sin(hdg) # forward
                spdr = spdx * math.sin(hdg) * (-1) + spdy * math.cos(hdg) # right

        def show_yuv_frame(self, window_name, yuv_frame):
            height = 720
            dali = time.time()

            # convert pdraw YUV flag to OpenCV YUV flag
            cv2_cvt_color_flag = {
                olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
            }[1]
            
            # yuv to Grayscale
            self.cv2frame = yuv_frame.as_ndarray()[:-1][:720]
            tag_detected = self.detector.detect(self.cv2frame,estimate_tag_pose = True, camera_params = var.camera_params, tag_size = 0.18) # 크기 0.18 - 실제랑 같게 나오는지 확인
            var.center_x = -1 # 놓치는거 방지 하기 위한 변수 -1 이면 놓침

            if len(tag_detected) > 0: # 이 부분이 오래 걸리면 0.1초 정도 걸린다. - 최대한 줄여보기
                euler = time.time()
                tag = tag_detected[0]

                rotM = tag.pose_R
                thetaZ = math.atan2(rotM[1, 0], rotM[0, 0])*180.0/math.pi
                thetaY = math.atan2(-1.0*rotM[2, 0], math.sqrt(rotM[2, 1]**2 + rotM[2, 2]**2))*180.0/math.pi
                thetaX = math.atan2(rotM[2, 1], rotM[2,2])*180.0/math.pi

                self.gim_ang = [thetaX,thetaY,thetaZ]

                theta = math.atan(tag.pose_t[1]/abs(tag.pose_t[2])) # * 180/3.141592
                self.gimbal_angle = self.drone.get_state(olympe.messages.gimbal.attitude)['pitch_absolute']
                
                # 짐벌 각도 수정 - 이거를 병렬로 돌려야 하나??
                drone_angle = -1 * (self.gimbal_angle * 3.141592 / 180 - theta)
                pch = -1 * drone_angle * 180/3.141592
                gim = threading.Thread(target = parent.mov_gim,args = (self,pch,))
                gim.start()

                drone_x = tag.pose_t[0] # 좌 우
                drone_y = math.cos(drone_angle) * (tag.pose_t[1] ** 2 + tag.pose_t[2] ** 2) ** 0.5 # distance
                drone_z = math.sin(drone_angle) * (tag.pose_t[1] ** 2 + tag.pose_t[2] ** 2) ** 0.5 # altitude

                var.drn_prev = var.drn
                var.drn_i += var.drn
                var.drn = [drone_x , drone_y, drone_z]
                var.drn = np.array(var.drn,dtype = float)
                if DRONE_IP == '10.202.0.1': # 시뮬레이션 일때 크기 보정
                    var.drn = var.drn * 2 / 1.6
                var.d_drn = var.drn - var.drn_prev

                var.gogh = var.van
                var.van = time.time() # PID 제어를 위해 필요하다.

                # 중심 좌표 업데이트
                var.center_x = tag.center[0]
                var.center_y = tag.center[1]

                cv2.circle(self.cv2frame, tuple(tag.corners[3].astype(int)), 4,(100,0,0), 2) # left-bottom
                cv2.circle(self.cv2frame, tuple(tag.center.astype(int)), 4,(255,0,0), 2) # center

                if var.can_i_move_original == var.can_i_move :
                    var.can_i_move += 1
                ######## 여기까지 Tag가 보일때 ########
            # 태그가 안보일때
            else :
                tag = 0 # imshow 때문.
                if control.end_control == 0:
                    var.vel = np.array([[0.],[0.],[0.],[0.]]) # 속도 컨트롤러 끄기 위함 - 이거는 수정해야 할 수도 있음
                    # print('stop; tag not found')

            salvador = time.time()
            var.dt = salvador - dali

            # heading test
            hdg = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged) # rad
            hdg = hdg['yaw'] # rad

            # GPS test - GPS 켜져있는지 확인 - 코드 없으면 좌표 안줄때 꺼져있다고 판단
            # olympe.messages.ardrone3.GPSSettings.SendControllerGPS

            # Use OpenCV to show this frame
            # 글자 표시할게 있으면, 화면 출력 직전에 하기
            txt_scrn = []
            txt_scrn.append('y : {}'.format(var.drn[1]))
            
            drone_speed = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)
            var.speedx = drone_speed['speedX']
            var.speedy = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)['speedY']
            spd = (var.speedx**2 + var.speedy**2)**0.5

            var.spdf = var.speedx * math.cos(hdg) + var.speedy * math.sin(hdg)
            var.spdr = var.speedx * math.sin(hdg) * (-1) + var.speedy * math.cos(hdg)

            # ##### 스피드 플롯하기
            # if var.error_vel[0] != var.error_vel_prev[0]:
            #     var.spd_f_graph = np.array([0.0])
            #     for i in range(10):
            #         print(7982379324798324)
            # var.spd_f_graph = np.append(var.spd_f_graph,var.spdf)
            # print(np.shape(var.spd_f_graph))
            # plt.close
            # plt.plot(var.spd_f_graph)
            # plt.ylabel('speed_f')
            # plt.show(block = False)
            # plt.pause(0.01)
            # ##### 스피드 플롯 종료
            
            txt_scrn.append('')
            txt_scrn.append(f'input : {var.vel[0][0]}')
            txt_scrn.append(f'forward : {var.spdf}')
            txt_scrn.append(f'tilt : {var.tilt[0][0]}')
            txt_scrn.append(f'speed   : {control.keyboard_spd}')
            if DRONE_IP == "10.202.0.1": # 시뮬레이션
                txt_scrn.append(f'P:{var.kf_sim_tilt[0]} I:{var.kf_sim_tilt[2]} D:{var.kf_sim_tilt[1]}')
            else :
                txt_scrn.append(f'P:{var.kf_real_tilt[0]} I:{var.kf_real_tilt[2]} D:{var.kf_real_tilt[1]}')
            txt_scrn.append(f'{var.i_error_vel}')
            
            # txt_scrn.append(f'speed   : {spd}')

            # drone_poi = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AltitudeAboveGroundChanged)
            # var.alt = drone_poi['altitude']
            # txt_scrn.append(f'alt : {var.alt}')
            
            # #GPS
            # # drone_gps1 = self.drone.get_state(olympe.messages.ardrone3.GPSSettings.SendControllerGPS)
            # drone_gps2 = self.drone.get_state(olympe.messages.ardrone3.PilotingState.GpsLocationChanged)
            # drone_gps3 = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PositionChanged)
            # # drone_gps = self.drone.get_state()
            # # drone_gps = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PositionChanged)
            # # drone_gps = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PositionChanged)
            # # drone_gps = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PositionChanged)
            # # drone_gps = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PositionChanged)

            for i_for in range(len(txt_scrn)):
                cv2.putText(self.cv2frame, "{}".format(txt_scrn[i_for]), (50, 50 * (i_for + 1)), # 50,50
                            cv2.FONT_HERSHEY_COMPLEX, 1, (255, 50, 0), 2, lineType=cv2.LINE_AA)
            # for i_for in range (10):
            #     print("imshow")
            cv2.imshow(window_name, self.cv2frame)
            cv2.waitKey(1)  # please OpenCV for 1 ms...

        def vel_controller(self):
            # while control.end_control == 0:
            while 1:
                # 시간 측정
                dt = var.isacc - var.newton

                var.newton = var.isacc
                var.isacc = time.time()

                # 드론 속도 받아오기
                drone_speed = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)
                drone_hdg = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged) # radian
                spdx = drone_speed['speedX']
                spdy = drone_speed['speedY']
                spdz = drone_speed['speedZ']
                spdc = (var.hdg - var.hdg_prev) * (180/math.pi) / dt # 현재 회전 속도 얻을 수 있는지 확인해보기
                var.hdg_prev = var.hdg
                var.hdg = drone_hdg['yaw'] # radian
                spdf = spdx * math.cos(var.hdg) + spdy * math.sin(var.hdg)
                spdr = spdx * math.sin(var.hdg) * (-1) + spdy * math.cos(var.hdg)

                # 에러 구하기
                var.error_vel = np.array([[var.vel[0][0]-spdf],[var.vel[1][0]-spdr],[var.vel[2][0]-spdz],[var.vel[3][0]-spdc]])
                var.d_error_vel = (var.error_vel - var.error_vel_prev) / dt
                var.error_vel_prev = var.error_vel
                var.i_error_vel += var.error_vel * dt
                if var.vel[0][0] != var.vel_prev[0][0]: # 이거는 forward에 대해만 짠거라서 추가 해야 됨
                    var.i_error_vel[0] = [0.0]
                    var.reset = 1
                var.vel_prev = var.vel

                # pitch
                var.tilt[0] = var.error_vel[0] * kf_tilt[0] + var.d_error_vel[0] * kf_tilt[1] + var.i_error_vel[0] * kf_tilt[2]
                # roll
                var.tilt[1] = var.error_vel[1] * kr_tilt[0] + var.d_error_vel[1] * kr_tilt[1] + var.i_error_vel[1] * kr_tilt[2]
                # throttle
                var.tilt[2] = var.error_vel[2] * kt_tilt[0] + var.d_error_vel[2] * kt_tilt[1] + var.i_error_vel[2] * kt_tilt[2]
                # yaw
                var.tilt[3] = var.error_vel[3] * kc_tilt[0] + var.d_error_vel[3] * kc_tilt[1] + var.i_error_vel[3] * kc_tilt[2]


                if var.tilt[0][0] > 100 : # 상황 봐서 수정하기 - for 문 써서 모든 tilt 갑ㅅ 수정할 수 있도록 하기
                    var.tilt[0][0] = 100
                elif var.tilt[0][0] < -100 :
                    var.tilt[0][0] = -100

                self.drone(
                        PCMD(
                        1,
                        var.tilt[1], # roll
                        var.tilt[0], # pitch
                        var.tilt[3], # yaw
                        var.tilt[2], # throttle
                        timestampAndSeqNum=0,
                    )
                )
        
        def kbrd(self):
            print('kbrd_start')
            # control = KeyboardCtrl()
            # control.keyboard_spd = 70
            while not control.quit():
                if control.takeoff():
                    self.drone(TakeOff())

                elif control.landing():
                    self.drone(Landing())
                    print("Drone has Landed")
                    print("Press ESC to end")

                # # # 원래 제어 코드
                # if control.has_piloting_cmd():
                #     self.drone(
                #             PCMD(
                #             1,
                #             control.roll(),
                #             control.pitch(),
                #             control.yaw(),
                #             control.throttle(),
                #             timestampAndSeqNum=0,
                #         )
                #     )

                # else:
                #     self.drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))

                # # # 속도 제어기를 이용한 제어
                if control.has_piloting_cmd(): # 속도 입력이 있을 때
                    var.vel[0][0] = control.pitch()
                    var.vel[1][0] = control.roll()
                    var.vel[2][0] = control.throttle()
                    var.vel[3][0] = control.yaw()

                else: # 속도 입력이 없을 때
                    var.vel = np.array([[0],[0],[0],[0]])

                time.sleep(0.05) # 이건 왜 있는거지?

                if control.gim_up():
                    print(1,var.gimbal_angle)
                    var.gimbal_angle += 5
                    if var.gimbal_angle > 90:
                        var.gimbal_angle = -90
                    print(2,var.gimbal_angle)

                    self.drone(set_target(
                        gimbal_id = 0,
                        control_mode="position",
                        yaw_frame_of_reference="none",   # None instead of absolute
                        yaw = 0.0,
                        pitch_frame_of_reference="absolute",
                        pitch = var.gimbal_angle, # 45.0
                        roll_frame_of_reference="none",     # None instead of absolute
                        roll = 0.0,
                    # )).wait().success()
                    ))
                    time.sleep(0.1) # 0.3초 딜레이가 없으면 너무 많이 이동한다.
                elif control.gim_down():
                    print(1,var.gimbal_angle)
                    var.gimbal_angle -= 5
                    if var.gimbal_angle < -90:
                        var.gimbal_angle = 90
                    print(2,var.gimbal_angle)

                    self.drone(set_target(
                        gimbal_id = 0,
                        control_mode="position",
                        yaw_frame_of_reference="none",   # None instead of absolute
                        yaw = 0.0,
                        pitch_frame_of_reference="absolute",
                        pitch = var.gimbal_angle, # 45.0
                        roll_frame_of_reference="none",     # None instead of absolute
                        roll = 0.0,
                    ))
                    time.sleep(0.1)
                
                if control.spd_up():
                    print('spd_up')
                    control.keyboard_spd += 0.5
                    print(control.keyboard_spd)
                    # time.sleep(0.05)
                    if control.keyboard_spd > 10:
                        control.keyboard_spd = 10
                    time.sleep(0.1)

                if control.spd_down():
                    print('spd_down')
                    control.keyboard_spd -= 0.5
                    # time.sleep(0.05)
                    if control.keyboard_spd < 0.5:
                        control.keyboard_spd = 0.5
                    time.sleep(0.1)

                if control.p_up():
                    print('p_up')
                    var.kf_sim_tilt[0] += 5.0
                    time.sleep(0.1)
                
                if control.p_down():
                    print('p_down')
                    var.kf_sim_tilt[0] -= 0.5
                    time.sleep(0.1)

                if control.i_up():
                    print('i_up')
                    var.kf_sim_tilt[2] += 0.5
                    time.sleep(0.1)

                if control.i_down():
                    print('i_down')
                    var.kf_sim_tilt[2] -= 0.5
                    time.sleep(0.1)

                if control.d_up():
                    print('d_up')
                    var.kf_sim_tilt[1] += 0.5
                    time.sleep(0.1)

                if control.d_down():
                    print('d_down')
                    var.kf_sim_tilt[1] -= 0.5
                    time.sleep(0.1)

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

# global var_tst
# var_tst = 10
# global control
# global var
# control = anafi_keyboard.KeyboardCtrl()
# var = Variables()

if __name__ == "__main__":
    # 와이파이 신호 받기
    wl = w_lan()
    wl_multi = threading.Thread(target = wl.get_signal)
    wl_multi.start()

    var = Variables()
    # strm = StreamingExample()
    strm = ChildStreaming()
    # Start the video stream
    strm.start()
    # 속도 실시간으로 받아오기
    spd_realtime = threading.Thread(target = strm.spd_realtime)
    spd_realtime.start()

    drone = strm.drone
    var.gimbal_angle = 0

    drone(GPSFixStateChanged(_policy = 'wait'))

    # start logging
    if var.start_log > 0 :
        for i in range(10):
            print('로그 시작')
        log_start = threading.Thread(target = strm.log)
        log_start.start()

    # for i in range(10):
    #     print(123123)
    var.can_i_move_original = 0 # 연산이 끝나고 이동을 하기 위한 변수
    # 드론 가지고 오기 - streaming 에서 이미 드론을 받아왔으므로 거기서 불러와야 한다.
    
    print(DRONE_IP)
    control = anafi_keyboard.KeyboardCtrl()
    # control = KeyboardCtrl()

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
            pitch = var.gimbal_angle, # 45.0
            roll_frame_of_reference="none",     # None instead of absolute
            roll = 0.0,
        )).wait().success()
    time.sleep(2)

    # 스트리밍 시작
    stream_video = threading.Thread(target = strm.run) # 이렇게 하면, strm을 멀티쓰레드 하는게 의미가 있나?
    stream_video.start()
    
    # # # 드론이 날고 있지 않을때 이륙
    if drone(FlyingStateChanged(state="hovering", _policy="check")): 
        print('Hovering')
    else :
        print('Takeoff')

        assert drone(
                TakeOff()
            ).wait().success()

    # start logging
    if var.start_log > 0 :
        for i in range(10):
            print('로그 시작')
        log_start = threading.Thread(target = strm.log)
        log_start.start()
    
################################################
# #     # # Apriltag가 보이지 않고, 키보드 입력 아닐때
    # 시뮬레이션 용
    kf_sim = np.array([0.3,0.0,0.0]) # forward PDI
    kr_sim = np.array([0.5,0.0,0.0]) # right PDI # 50,0,0 
    kc_sim = np.array([0.5,0.0,0.0]) # clockwise PDI

    var.kf_sim_tilt = np.array([5.0,0.0,0.0]) # forward PDI #[98.0,25.0,6.0]
    var.kr_sim_tilt = np.array([1.0,0.0,0.0]) # right PDI
    var.kt_sim_tilt = np.array([1.0,0.0,0.0]) # right PDI
    var.kc_sim_tilt = np.array([1.0,0.0,0.0]) # right PDI

    # 실제 드론 용
    kf_real = np.array([0.1,0.0,0.0])
    kr_real = np.array([0.1,0.0,0.0])
    kc_real = np.array([0.1,0.0,0.0])

    var.kf_real_tilt = np.array([1,0,0]) # forward PDI
    var.kr_real_tilt = np.array([1,0,0]) # right PDI
    var.kt_real_tilt = np.array([1,0,0]) # right PDI
    var.kc_real_tilt = np.array([1,0,0]) # right PDI

    if DRONE_IP == "10.202.0.1" : # 시뮬레이션
        kf = kf_sim
        kr = kr_sim
        kc = kc_sim

        kf_tilt = var.kf_sim_tilt
        kr_tilt = var.kr_sim_tilt
        kt_tilt = var.kt_sim_tilt
        kc_tilt = var.kc_sim_tilt

    else : # 실제
        kf = kf_real
        kr = kr_real
        kc = kc_real

        kf_tilt = var.kf_real_tilt
        kr_tilt = var.kr_real_tilt
        kt_tilt = var.kt_real_tilt
        kc_tilt = var.kc_real_tilt
################################################
    while var.center_x == -1 and control.end_control == 0:

        if wl.signal < 0.5 * wl.signal_full : # 이거는 나중에 신호 봐서 갑ㅅ 수정하기
            drone(
                        PCMD(
                        1,
                        0, # roll
                        0, # pitch
                        0, # yaw
                        0, # throttle
                        timestampAndSeqNum=0,
                    )
                )

            drone(Landing())
            print('Weak Signal')

            break

        # 이륙 후 Apriltag 찾을때 - 더 좋은 방법이 있을 듯
        if var.gimbal_angle < -120 : # -120
            var.gimbal_angle = 30
        else :
            var.gimbal_angle = var.gimbal_angle - 1

        drone(set_target(
            gimbal_id = 0,
            control_mode="position",
            yaw_frame_of_reference="none",   # None instead of absolute
            yaw = 0.0,
            pitch_frame_of_reference="absolute",
            pitch = var.gimbal_angle, # 45.0
            roll_frame_of_reference="none",     # None instead of absolute
            roll = 0.0,
        )).wait()
    print('Tag found')

    # # 속도 컨트롤러 시작
    print('Velocity controller start')
    vel_controller = threading.Thread(target = ChildStreaming.vel_controller ,args = (strm,))
    vel_controller.start()

    # 키보드 조종으로 전환하지 않았을때
    while control.end_control == 0:
        if var.can_i_move_original < var.can_i_move: # 이 조건을 넣는게 맞는지 잘 모르겠음
                                        # 뒤에 조건이 없으면 태그를 detection하는 순간에 이동 명령이 평균 10회 정도
                                        # 들어가기 때문에 이걸 1회로 제한하기 위한 부분이다.
            var.can_i_move_original += 1 # 한번만 하기 위함
            theta_cc = math.atan(var.drn[0]/var.drn[1]) * 180 / math.pi

            if var.center_x > -1: # 태그가 보일때
                ### 속도 제어 ###
                time_now = time.time()
                dt = var.van - var.gogh

                # Forward
                if var.gimbal_angle > -75 : # 짐벌이 바로 아래를 보지 않는 다면 - 이 조건 때문에 드론이 뒤로 날지 않는 듯하다. 이거도 나중에 수정하기
                # if abs(strm.y) < 0.001: # 시뮬레이션인지 실제인지 보고 수정하기
                    var.vel[0] = kf[0] * var.drn[1] + kf[1] * var.d_drn[1]/dt + kf[2] * var.drn_i[1] * dt
                    print(f'Vel forward {var.vel[0]}')
                else : # 이거는 나중에 바꾸기
                    var.vel[0] = np.array([0])
                    print(f'Vel forward {var.vel[0]}')

                # Right = theta_cc * kc[0]
                if abs(var.center_x - 640) > 100 : # 1280 / 2 = 640 : 픽셀을 재는 방향이 오른쪽에서 왼쪽인 듯
                    var.vel[1] = kr[0] * var.drn[0] + kr[1] * var.d_drn[0]/dt + kr[2] * var.drn_i[0] * dt
                    print(f'Vel right {var.vel[1]}')
                else : # 이거는 나중에 수정하기
                    var.vel[1] = np.array([0])
                    print(f'Vel right {var.vel[1]}')

                # # Clocklwise
                # if abs(strm.center_x - 640) > 150 : # 중심이 화면 중앙에 있지 않을 때
                #     vel[3] = kc[0] * strm.drn[3] + kc[1] * strm.d_drn[3]/dt + kc[2] * strm.drn_i[3] * dt
                    # print(f'Clockwise {strm.vel[3]}')

                # Throttle - 안건드리는게 나을 듯
                # if 고도가 낮을때?

                # Landing
                # if strm.gim_ang[0] > -15 and abs(strm.center_x - 640) < 50: # 착륙 - 나중에 수정하기 # 거리가 확실해지기 전까지는 최대한 각도 데이터 이용하기
                if abs(var.drn[1]) < 0.2: # 착륙 - 나중에 수정하기 # 거리가 확실해지기 전까지는 최대한 각도 데이터 이용하기 # 0.15
                    drone(Landing())
                    print('landing') # 이게 오래 걸리므로 몇초 뒤에 종료하거나 고도 이용해서 종료하기

                # print('이동 끝')
                var.time_tag = time.time() # 태그 놓칠때 - 이거는 방법 바꾸면 지우기
            else : # 태그가 안보일때
                pass

        else : # can_i_move >= strm_can_i_move
            # 태그 놓쳤을 때
            if var.center_x == -1:
                if var.gimbal_angle < -120 :
                        var.gimbal_angle = 30
                else:
                    var.gimbal_angle = var.gimbal_angle - 1

                drone(set_target(
                    gimbal_id = 0,
                    control_mode="position",
                    yaw_frame_of_reference="none",   # None instead of absolute
                    yaw = 0.0,
                    pitch_frame_of_reference="absolute",
                    pitch = var.gimbal_angle, # 45.0
                    roll_frame_of_reference="none",     # None instead of absolute
                    roll = 0.0,
                )).wait().success()

                # 5초 이상 태그를 못 찾으면 착륙
                if time.time() - var.time_tag > 10:
                    drone(Landing())
    ############# *중요* 비상 조종용
    print('keyborad control start')
    print('Press tab to takeoff')
    print('l to land')
    print('r,f for gimbal')
    print('t,g for speed')
    print('y,h for p')
    print('u,j for i')
    print('i,k for d')
    print('Press esc to end')
    kbrd_parallel = threading.Thread(target = strm.kbrd)
    kbrd_parallel.start()

    while 1: # 속도 plot 하기
        plt.close
        if var.reset == 1 :
            var.spd_f_graph = np.array([0.0])
            var.graph_time = np.array([time.time()])
            var.reset = 0
            plt.cla()
        var.spd_f_graph = np.append(var.spd_f_graph,var.spdf)
        var.graph_time = np.append(var.graph_time,time.time())
        # plt.plot(strm.spd_f_graph)
        plt.scatter(var.graph_time, var.spd_f_graph, s = 100, c = 'g')
        plt.ylabel('speed_f')
        plt.show(block = False)
        plt.pause(0.0001)

        pass

'''
Log :
수정 된 것들
streaming_example -> strm
PD Control
화면에 글자 출력 쉽게
짐벌 수동조작 가능
변수들 배열로 저장
시뮬레이션이랑 실제랑 계수만 다르게 하고 이동하는 코드는 동일하게 바꾸기
github upload test
드론 비행 가능 최저 고도 확인 : 50cm - 변경 안됨
시뮬레이션 상의 거리랑 실제 거리가 유사하게 변환 - 시뮬레이션은 0.7배속 정도로 돌아가서 드론의 속도가 비슷하게 나오는지는 잘 모르겠음.
위치 및 속력 기록하는 로그 파일 생성 코드 추가 (수치 적분)
로그 기록시 고도는 적분하지 말고 센서에서 받아온 갑ㅅ으로도 기록
속도 컨트롤러 병렬로 따로 돌리게 수정


수정할 것들
드론이 뒤로 이동하지 않음 - 뒤로 이동할 필요가 있나? 회전을 하는게 더 좋은가?
코드 실행 후에 버튼 누르면 로그 기록하는 코드 추가하기
지금은 영상 불러오는 속도가 연산하는 속도보다 빨라서 문제가 안되지만, 연산이 빨라지게 되면 문제가 생길 수도 있다.
키 입력으로 PID 계수 바꿀수 있게 수정 + 화면에 출력이 가능하게 하기

GPS 꺼져있는지 확인하는 코드
와이파이 신호 세기 확인하는 코드

 ########################## ########################## ########################## ##########################
344번째 줄에
# 드론 속도 받아오기
이라고 되어 있는 주석이 있습니다. 그 밑에 드론의 속도가 있습니다.
여기서 spdf(forward), spdr(right), spdz 부분에 teensy에서 받은 속도를 넣어주면 됩니다.

spd_realtime을 병렬로 돌려서 속도를 실시간으로 받아옵니다.

'''