#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.

import csv
import cv2
import math
import os
import queue # 병렬 연산하는데 필요하다고 함
import shlex
import subprocess
import tempfile
import threading
import traceback
import time
# import apriltag
import socket
# from pupil_apriltags import Detector
import pupil_apriltags
import numpy as np

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

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

# 드론 와이파이 인지 아닌지 자동 판별
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try: # 시뮬레이션
    s.connect(('8.8.8.8', 0))
    # ip = s.getsockname()[0]
    DRONE_IP= "10.202.0.1"
except: # 드론 연결
    DRONE_IP = '192.168.42.1' # 드론

class Ctrl(Enum):
    (
        QUIT,
        TAKEOFF,
        LANDING,
        MOVE_LEFT,
        MOVE_RIGHT,
        MOVE_FORWARD,
        MOVE_BACKWARD,
        QUIT_AUTO_PILOT,
        MOVE_UP,
        MOVE_DOWN,
        TURN_LEFT,
        TURN_RIGHT,
        GIM_UP,
        GIM_DOWN,
    ) = range(14)

QWERTY_CTRL_KEYS = {
    Ctrl.QUIT: Key.esc,
    Ctrl.TAKEOFF: "t",
    Ctrl.LANDING: "l",
    Ctrl.MOVE_LEFT: "a",
    Ctrl.MOVE_RIGHT: "d",
    Ctrl.MOVE_FORWARD: "w",
    Ctrl.MOVE_BACKWARD: "s",
    Ctrl.QUIT_AUTO_PILOT: "q",
    Ctrl.MOVE_UP: Key.up,
    Ctrl.MOVE_DOWN: Key.down,
    Ctrl.TURN_LEFT: Key.left,
    Ctrl.TURN_RIGHT: Key.right,
    Ctrl.GIM_UP: "u",
    Ctrl.GIM_DOWN: "j",
}

AZERTY_CTRL_KEYS = QWERTY_CTRL_KEYS.copy()
AZERTY_CTRL_KEYS.update(
    {
        Ctrl.MOVE_LEFT: "q",
        Ctrl.MOVE_RIGHT: "d",
        Ctrl.MOVE_FORWARD: "z",
        Ctrl.MOVE_BACKWARD: "s",
    }
)

class KeyboardCtrl(Listener):
    end_control = 0
    def __init__(self, ctrl_keys=None):
        self._ctrl_keys = self._get_ctrl_keys(ctrl_keys)
        self._key_pressed = defaultdict(lambda: False)
        self._last_action_ts = defaultdict(lambda: 0.0)
        super().__init__(on_press=self._on_press, on_release=self._on_release)
        global end_control
        self.start()

    def _on_press(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = True
        elif isinstance(key, Key):
            self._key_pressed[key] = True
        if self._key_pressed[self._ctrl_keys[Ctrl.QUIT_AUTO_PILOT]]:
            print('quit auto pilot')
            self.end_control = 1 # 1이면 오토파일럿 중지
        if self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]:
            return False
        else:
            return True

    def _on_release(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = False
        elif isinstance(key, Key):
            self._key_pressed[key] = False
        return True

    def quit(self):
        return not self.running or self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]

    def gim_up(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.GIM_UP]]

    def gim_down(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.GIM_DOWN]]

    def _axis(self, left_key, right_key):
        return 50 * ( # 원래 50이 아니라 100인데, 속도 반으로 줄임
            int(self._key_pressed[right_key]) - int(self._key_pressed[left_key])
        )

    def roll(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_LEFT],
            self._ctrl_keys[Ctrl.MOVE_RIGHT]
        )

    def pitch(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_BACKWARD],
            self._ctrl_keys[Ctrl.MOVE_FORWARD]
        )

    def yaw(self):
        return self._axis(
            self._ctrl_keys[Ctrl.TURN_LEFT],
            self._ctrl_keys[Ctrl.TURN_RIGHT]
        )

    def throttle(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_DOWN],
            self._ctrl_keys[Ctrl.MOVE_UP]
        )

    def has_piloting_cmd(self):
        return (
            bool(self.roll())
            or bool(self.pitch())
            or bool(self.yaw())
            or bool(self.throttle())
        )

    def _rate_limit_cmd(self, ctrl, delay):
        now = time.time()
        if self._last_action_ts[ctrl] > (now - delay):
            return False
        elif self._key_pressed[self._ctrl_keys[ctrl]]:
            self._last_action_ts[ctrl] = now
            return True
        else:
            return False

    def takeoff(self):
        return self._rate_limit_cmd(Ctrl.TAKEOFF, 2.0)

    def landing(self):
        return self._rate_limit_cmd(Ctrl.LANDING, 2.0)

    def _get_ctrl_keys(self, ctrl_keys):
        # Get the default ctrl keys based on the current keyboard layout:
        if ctrl_keys is None:
            ctrl_keys = QWERTY_CTRL_KEYS
            try:
                # Olympe currently only support Linux
                # and the following only works on *nix/X11...
                keyboard_variant = (
                    subprocess.check_output(
                        "setxkbmap -query | grep 'variant:'|"
                        "cut -d ':' -f2 | tr -d ' '",
                        shell=True,
                    )
                    .decode()
                    .strip()
                )
            except subprocess.CalledProcessError:
                pass
            else:
                if keyboard_variant == "azerty":
                    ctrl_keys = AZERTY_CTRL_KEYS
        return ctrl_keys

class StreamingExample(threading.Thread):
    time_tag = 0 # 태그 놓치는 시간이 길이지면 착륙
    time_time = time.time()
    time_time_prev = time.time()
    tag_detected = []
    gimbal_angle = 0
    gim_ang = [0,0,0] # xyz
    drn_prev = np.array([[0.0],[0.0],[0.0]]) # xyz
    drn = np.array([[0.0],[0.0],[0.0]]) # xyz
    drn_i = np.array([[0.0],[0.0],[0.0]]) # xyz
    d_drn = np.array([[0.0],[0.0],[0.0]]) # 거리 차이
    can_i_move = 0
    can_i_move_original = 0
    start_log = 0 # 1 when logging, 0 when not logging

    center_x = -1
    center_y = -1
    # detector = apriltag.Detector()
    detector = pupil_apriltags.Detector()
    camera_params = [920.6649,920.4479,652.8415,355.9656]

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        # print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        super().__init__()
        super().start()

    def start(self):
        # Connect the the drone
        self.drone.connect()

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()
    
    def log(self):
        '''
        드론의 속도 정보를 계속 받아서 저장하는 코드이다.
        병렬로 돌고 있기 때문에 로그 기록하는 속도가 빠르다.
        파일 용량이 너무 커지게 된다면 시간 간격을 두고 로그를 기록하게 수정하기.
        
        '''
        log_path = '/home/aims/git_anafi/anafi-apriltag-auto-landing/logs/' # 로그 저장되는 위치
        # log_array = np.array([['time','speed_north','speed_east','speed_down']])
        log_array = np.array([['time','speed_north','speed_east','speed_forward','speed_right','speed_up','North','East','Forward','Right','Altitude','Heading']])
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
                drone_speed = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)
                drone_hdg = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged) # rad
                hdg = drone_hdg['yaw'] # rad
                spdx = drone_speed['speedX']
                spdy = drone_speed['speedY']
                spdz = drone_speed['speedZ'] * (-1)
                spdf = spdx * math.cos(hdg) + spdy * math.sin(hdg)
                spdr = spdx * math.sin(hdg) * (-1) + spdy * math.cos(hdg)
                time_now = time.strftime('%H:%M:%s', time.localtime(time.time()))
                
                # 거리 추정
                dt = time.time() - self.time_log_prev
                self.time_log_prev = time.time()
                # print('dt',print(type(dt)))
                self.location = self.location + dt * np.array([spdx,spdy,spdz,spdf,spdr])
                # print(self.location)
                # print('loc',print(type(self.location[0])))

                # tmp = np.array([[time_now,spdx,spdy,spdz]])
                tmp = np.array([[time_now,spdx,spdy,spdf,spdr,spdz,self.location[0],self.location[1],self.location[3],self.location[4],self.location[2],hdg * 180/math.pi]])

                # 로그 합치기
                log_array = np.append(log_array,tmp,axis = 0)

                # 로그 모양 확인
                # print(log_array.shape)
                a,b = log_array.shape

                # 로그 기록
                if a % 50 == 0: # 1초에 한번씩 기록
                    # print('a : ',a)
                    np.savetxt(f'{log_path}{name}.csv', log_array, fmt = '%s', delimiter = ',')
                    print('saved',a)
                
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
        self.frame_queue.queue.clear() # 딜레이 줄이기 위함 - 나중에 queue 안쓰는 방향으로 코드 수정
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

    def show_yuv_frame(self, window_name, yuv_frame):
        # print('DETECTion start')
        height = 720
        time_now = time.time()

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[1]
        
        # yuv to Grayscale
        self.cv2frame = yuv_frame.as_ndarray()[:-1][:720]

        self.tag_detected = self.detector.detect(self.cv2frame,estimate_tag_pose = True, camera_params = self.camera_params, tag_size = 0.18) # 크기 0.18 - 실제랑 같게 나오는지 확인
        
        self.center_x = -1 # 놓치는거 방지 하기 위한 변수 -1 이면 놓침

        self.b = 0
        if len(self.tag_detected) > 0: # 이 부분이 오래 걸리면 0.1초 정도 걸린다. - 최대한 줄여보기
            a = time.time()
            tag = self.tag_detected[0]

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
            gim = threading.Thread(target = StreamingExample.mov_gim,args = (self,pch,))
            gim.start()

            drone_x = tag.pose_t[0] # 좌 우
            drone_y = math.cos(drone_angle) * (tag.pose_t[1] ** 2 + tag.pose_t[2] ** 2) ** 0.5 # distance
            drone_z = math.sin(drone_angle) * (tag.pose_t[1] ** 2 + tag.pose_t[2] ** 2) ** 0.5 # altitude

            self.drn_prev = self.drn
            self.drn_i += self.drn
            self.drn = [drone_x , drone_y, drone_z]
            self.drn = np.array(self.drn,dtype = float)
            if DRONE_IP == '10.202.0.1': # 시뮬레이션 일때 크기 보정
                self.drn = self.drn * 2 / 1.6
            self.d_drn = self.drn - self.drn_prev

            self.time_time_prev = self.time_time
            self.time_time = time.time() # PID 제어를 위해 필요하다.

            # 중심 좌표 업데이트
            self.center_x = tag.center[0]
            self.center_y = tag.center[1]

            cv2.circle(self.cv2frame, tuple(tag.corners[3].astype(int)), 4,(100,0,0), 2) # left-bottom
            cv2.circle(self.cv2frame, tuple(tag.center.astype(int)), 4,(255,0,0), 2) # center
            if self.can_i_move_original == self.can_i_move :
                self.can_i_move += 1
            self.b = time.time() - a
            ######## 여기까지 Tag가 보일때 ########
        else :
            tag = 0 # imshow 때문.

        self.dt = time.time()-time_now

        # heading test
        hdg = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged) # rad
        hdg = hdg['yaw'] # rad

        # Use OpenCV to show this frame
        # 글자 표시할게 있으면, 화면 출력 직전에 하기
        txt_scrn = []
        txt_scrn.append('Distance')
        txt_scrn.append('x : {}'.format(self.drn[0]))
        txt_scrn.append('y : {}'.format(self.drn[1]))
        txt_scrn.append('z : {}'.format(self.drn[2]))
        txt_scrn.append('')
        txt_scrn.append('time : {:.4f}'.format(self.dt))
        # txt_scrn.append('time : {:.4f}'.format(self.b))
        txt_scrn.append('')

        drone_speed = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)
        self.speedx = drone_speed['speedX']
        self.speedy = self.drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)['speedY']
        spd = (self.speedx**2+self.speedy**2)**0.5

        self.spdf = self.speedx * math.cos(hdg) + self.speedy * math.sin(hdg)
        self.spdr = self.speedx * math.sin(hdg) * (-1) + self.speedy * math.cos(hdg)
        txt_scrn.append('')
        txt_scrn.append(f'forward : {self.spdf}')
        txt_scrn.append(f'right   : {self.spdr}')
        txt_scrn.append(f'speed   : {spd}')

        # drone_poi = self.drone.get_state(olympe.messages.ardrone3.PilotingState.AltitudeAboveGroundChanged)
        # self.alt = drone_poi['altitude']
        # txt_scrn.append(f'alt : {self.alt}')
        # print(f'alt : {self.alt}')

        # drone_poi1 = self.drone.get_state(olympe.messages.ardrone3.PilotingState.PositionChanged)
        # self.alt1 = drone_poi1['altitude']
        # txt_scrn.append(f'alt : {self.alt1}')

        # drone_position = self.drone.get_state(olympe.messages.ardrone3.PilotingState.moveByChanged)
        # self.alt2 = drone_position['dXAsked']
        # print(f'alt : {self.alt2}')
        # txt_scrn.append(f'alt : {self.alt2}')

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

    def kbrd(self): # 여기 없어도 될거 같다.
        control = KeyboardCtrl()
        while not control.quit():
            if control.takeoff():
                self.drone(TakeOff())
            elif control.landing():
                self.drone(Landing())
                print("Drone has Landed")
                print("Press ESC to end")

            if control.has_piloting_cmd():
                self.drone(
                        PCMD(
                        1,
                        control.roll(),
                        control.pitch(),
                        control.yaw(),
                        control.throttle(),
                        timestampAndSeqNum=0,
                    )
                )

            else:
                self.drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))
            time.sleep(0.05)

            if control.gim_up():
                time.sleep(0.3) # 0.3초 딜레이가 없으면 너무 많이 이동한다.
                print(1,self.gimbal_angle)
                self.gimbal_angle += 5
                if self.gimbal_angle > 90:
                    self.gimbal_angle = -90
                print(2,self.gimbal_angle)

                self.drone(set_target(
                    gimbal_id = 0,
                    control_mode="position",
                    yaw_frame_of_reference="none",   # None instead of absolute
                    yaw = 0.0,
                    pitch_frame_of_reference="absolute",
                    pitch = self.gimbal_angle, # 45.0
                    roll_frame_of_reference="none",     # None instead of absolute
                    roll = 0.0,
                # )).wait().success()
                ))
            elif control.gim_down():
                time.sleep(0.3)
                print(1,self.gimbal_angle)
                self.gimbal_angle -= 5
                if self.gimbal_angle < -90:
                    self.gimbal_angle = 90
                print(2,self.gimbal_angle)

                self.drone(set_target(
                    gimbal_id = 0,
                    control_mode="position",
                    yaw_frame_of_reference="none",   # None instead of absolute
                    yaw = 0.0,
                    pitch_frame_of_reference="absolute",
                    pitch = self.gimbal_angle, # 45.0
                    roll_frame_of_reference="none",     # None instead of absolute
                    roll = 0.0,
                ))

if __name__ == "__main__":
    strm = StreamingExample()
    # Start the video stream
    strm.start()
    # start logging
    if strm.start_log > 0 :
        for i in range(10):
            print('로그 시작')
        log_start = threading.Thread(target = strm.log)
        log_start.start()

    for i in range(10):
        print(123123)
    strm.can_i_move_original = 0 # 연산이 끝나고 이동을 하기 위한 변수
    # 드론 가지고 오기 - streaming 에서 이미 드론을 받아왔으므로 거기서 불러와야 한다.
    drone = strm.drone
    strm.gimbal_angle = 0

    print(DRONE_IP)
    control = KeyboardCtrl()

    # 짐벌 최고속도
    drone(
        olympe.messages.gimbal.set_max_speed(0, 10, 3600, 30, _timeout=10, _no_expect=False, _float_tol=(0.1, 0.1)) # 짐벌 id, yaw pitch roll
    ) # 세번째 숫자가 짐벌 속도 큰 숫자로 두기

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

    # # 드론이 날고 있지 않을때 이륙
    if drone(FlyingStateChanged(state="hovering", _policy="check")): 
        print('Hovering')
    else :
        print('Takeoff')

        assert drone(
                TakeOff()
            ).wait().success()
    
    # time.sleep(3)
    # ### 테스트용 코드 - 속도 확인용
    # # while 1:
    # if DRONE_IP == "10.202.0.1" : # 시뮬레이션
    #     # print(123)
    #     abc = time.time()
    #     tilt = 15
    #     spd = np.array([])
    #     spd_average = np.array([[0,0,0]])

    #     # while time.time() - abc < 5:
    #     while 1:
    #         drone(
    #                 PCMD(
    #                 1,
    #                 0, # roll
    #                 tilt, # pitch
    #                 0, # yaw
    #                 0, # throttle
    #                 timestampAndSeqNum=0,
    #             )
    #         )

    #         drone_speed = drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged)
    #         speedx = drone_speed['speedX']
    #         speedy = drone_speed['speedY']
    #         speed = (speedx**2+speedy**2)**0.5

    #         print(f'speed : {speed}\ntilt : {tilt}\n')

    #         spd = np.append(spd,[speed])

    #         # if 0 < spd[len(spd)-1] - spd[0] < 0.01:
    #         #     tilt += 10
    #         #     spd_average = np.append(spd_average,np.average(spd))
    #         #     spd = np.array([])
    #         #     if tilt > 100:
    #         #         np.savetxt(f'/home/aims/Desktop/Anafi_code/logs/spd.csv', spd_average, fmt = '%s', delimiter = ',')
    #         #         break
    #         if time.time()-abc > 5:
    #             if len(spd) > 30:
    #                 if 0 < np.average(spd[25:30]) - np.average(spd[0:5]) < 0.001:
    #                     spd_average = np.append(spd_average,np.array([[tilt,np.average(spd),time.time()-abc]]),axis = 0)
    #                     tilt += 5
    #                     if tilt > 100:
    #                         np.savetxt(f'/home/aims/Desktop/Anafi_code/logs/spd.csv', spd_average, fmt = '%s', delimiter = ',')
    #                         break

    #                     # 드론 속도 초기화
    #                     drone(
    #                             PCMD(
    #                             1,
    #                             0, # roll
    #                             0, # pitch
    #                             0, # yaw
    #                             0, # throttle
    #                             timestampAndSeqNum=0,
    #                         )
    #                     )
    #                     time.sleep(30)
    #                     abc = time.time()
    #                     spd = np.array([])
                        
    #                 spd = np.array([speed])


    ### 테스트 용 코드 - 여기다가 무한루프 만들면 아래 코드 안돌아감
    # print('Test')
    # print('Press U or J to move gimbal')
    # # a = time.time()

    # strm.kbrd()
    # while 1: # 짐벌 수동 조작 확인용
    #     a = 1

    # print('Test end')

################################################

# #     # # Apriltag가 보이지 않고, 키보드 입력 아닐때

    # 이동하는데 필요한 변수들
    mov = [0,0,0,0] # forward, right, up, clockwise

    # 시뮬레이션 용
    kf_sim = np.array([2.5,0,0]) # forward pdi
    kr_sim = np.array([2.5,0,0]) # right pdi # 50,0,0 
    kc_sim = np.array([1,0,0]) # clockwise pdi

    # 실제 드론 용
    kf_real = np.array([2.5,0,0]) # 2.5까지는 안전한거 확인
    kr_real = np.array([2.5,0,0])
    kc_real = np.array([1,0,0])

    if DRONE_IP == "10.202.0.1" : # 시뮬레이션
        kf = kf_sim
        kr = kr_sim
        kc = kc_sim

    else : # 실제
        kf = kf_real
        kr = kr_real
        kc = kc_real

################################################

    while strm.center_x == -1 and control.end_control == 0:

        # 이륙 후 Apriltag 찾을때 - 더 좋은 방법이 있을 듯
        if strm.gimbal_angle < -120 : # -120
            strm.gimbal_angle = 30
        else :
            strm.gimbal_angle = strm.gimbal_angle - 1

        drone(set_target(
            gimbal_id = 0,
            control_mode="position",
            yaw_frame_of_reference="none",   # None instead of absolute
            yaw = 0.0,
            pitch_frame_of_reference="absolute",
            pitch = strm.gimbal_angle, # 45.0
            roll_frame_of_reference="none",     # None instead of absolute
            roll = 0.0,
        )).wait()
    print('Tag found')

    # 키보드 조종으로 전환하지 않았을때
    while control.end_control == 0:
        if strm.can_i_move_original < strm.can_i_move: # 이 조건을 넣는게 맞는지 잘 모르겠음
                                        # 뒤에 조건이 없으면 태그를 detection하는 순간에 이동 명령이 평균 10회 정도
                                        # 들어가기 때문에 이걸 1회로 제한하기 위한 부분이다.
            strm.can_i_move_original += 1 # 한번만 하기 위함
            theta_cc = math.atan(strm.drn[0]/strm.drn[1]) * 180 / 3.141592# * 180/3.141592

            if strm.center_x > -1: # 태그가 보일때
                time_now = time.time()
                dt = strm.time_time - strm.time_time_prev

                # Forward
                if strm.gimbal_angle > -75 : # 짐벌이 바로 아래를 보지 않는 다면
                # if abs(strm.y) < 0.001: # 시뮬레이션인지 실제인지 보고 수정하기
                    mov[0] = kf[0] * strm.drn[1] + kf[1] * strm.d_drn[1]/dt + kf[2] * strm.drn_i[1]
                    print('Moving Forward {}'.format(mov[0]))

                # Right
                if abs(strm.center_x - 640) > 100 : # 1280 / 2 = 640 : 픽셀을 재는 방향이 오른쪽에서 왼쪽인 듯
                    mov[1] = kr[0] * strm.drn[0] + kr[1] * strm.d_drn[0]/dt + kr[2] * strm.drn_i[0]
                    print('Moving right {}'.format(mov[1]))

                # # Clocklwise
                # if abs(strm.center_x - 640) > 150 : # 중심이 화면 중앙에 있지 않을 때
                #     mov[3] = theta_cc * kc[0]

                # Landing
                # if strm.gim_ang[0] > -15 and abs(strm.center_x - 640) < 50: # 착륙 - 나중에 수정하기 # 거리가 확실해지기 전까지는 최대한 각도 데이터 이용하기
                if abs(strm.drn[1]) < 0.15: # 착륙 - 나중에 수정하기 # 거리가 확실해지기 전까지는 최대한 각도 데이터 이용하기
                    drone(Landing())
                    print('landing') # 이게 오래 걸리므로 몇초 뒤에 종료하거나 고도 이용해서 종료하기

                # PCMD로 이동
                drone(
                        PCMD(
                        1,
                        mov[1], # roll
                        mov[0], # pitch
                        mov[3], # yaw
                        mov[2], # throttle
                        timestampAndSeqNum=0,
                    )
                )
                
                # print('이동 끝')
                strm.time_tag = time.time() # 태그 놓칠때 - 이거는 방법 바꾸면 지우기

        else : # can_i_move >= strm_can_i_move
            # 태그 놓쳤을 때
            if strm.center_x == -1:
                if strm.gimbal_angle < -120 :
                        strm.gimbal_angle = 30
                else:
                    strm.gimbal_angle = strm.gimbal_angle - 1

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

                # 5초 이상 태그를 못 찾으면 착륙
                if time.time() - strm.time_tag > 10:
                    drone(Landing())

    ############# *중요* 비상 조종용

    print('keyborad control start')
    print('Press esc to end')
    strm.kbrd()

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

드론 비행 가능 최저 고도 확인 : 50cm
시뮬레이션 상의 거리랑 실제 거리가 유사하게 변환 - 시뮬레이션은 0.7배속 정도로 돌아가서 드론의 속도가 비슷하게 나오는지는 잘 모르겠음.

위치 및 속력 기록하는 로그 파일 생성 코드 추가 (수치 적분)
 
수정할 것들
드론이 뒤로 이동하지 않음 - 뒤로 이동할 필요가 있나? 회전을 하는게 더 좋은가?
로그 기록시 고도는 적분하지 말고 센서에서 받아온 갑ㅅ으로 기록하는 코드 추가하기

'''