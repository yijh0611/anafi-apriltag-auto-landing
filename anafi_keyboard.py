'''
키보드 부분 다른 파일로 옮기기 위해 따로 파이썬 파일을 만들었다.
'''

import subprocess
import time
import socket
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
        SPD_UP,
        SPD_DOWN,
        P_UP,
        P_DOWN,
        I_UP,
        I_DOWN,
        D_UP,
        D_DOWN,
    ) = range(22)

QWERTY_CTRL_KEYS = {
    Ctrl.QUIT: Key.esc,
    Ctrl.TAKEOFF: Key.tab,
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
    Ctrl.GIM_UP: "r",
    Ctrl.GIM_DOWN: "f",
    Ctrl.SPD_UP: "t",
    Ctrl.SPD_DOWN: "g",
    Ctrl.P_UP: "y",
    Ctrl.P_DOWN: "h",
    Ctrl.I_UP: "u",
    Ctrl.I_DOWN: "j",
    Ctrl.D_UP: "i",
    Ctrl.D_DOWN: "k",
    
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
    keyboard_spd = 5
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

    def spd_up(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.SPD_UP]]

    def spd_down(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.SPD_DOWN]]
    
    def p_up(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.P_UP]]

    def p_down(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.P_DOWN]]

    def i_up(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.I_UP]]

    def i_down(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.I_DOWN]]

    def d_up(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.D_UP]]

    def d_down(self):
        return self._key_pressed[self._ctrl_keys[Ctrl.D_DOWN]]

    def _axis(self, left_key, right_key):
        return self.keyboard_spd * ( # 원래 50이 아니라 100인데, 속도 반으로 줄임
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