# -*- coding: UTF-8 -*-

import olympe
from logging import getLogger
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})
# olympe.log.update_config({
#     "handlers": {
#         "olympe_log_file": {
#             "class": "logging.FileHandler",
#             "formatter": "default_formatter",
#             "filename": "olympe.log"
#         },
#         "ulog_log_file": {
#             "class": "logging.FileHandler",
#             "formatter": "default_formatter",
#             "filename": "ulog.log"
#         },
#     },
#     "loggers": {
#         "olympe": {
#             "handlers": ["console", "olympe_log_file"]
#         },
#         "ulog": {
#             "level": "DEBUG",
#             "handlers": ["console", "ulog_log_file"],
#         }
#     }
# })

DRONE_IP = "10.202.0.1"

if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)#, name="toto")
    drone.connect()

    # a = getLogger("Olympe.drone")
    # print(123)
    # print(a.log)

    assert drone(TakeOff()).wait().success()
    assert drone.start_video_streaming()
    # drone(olympe.messages.ardrone3.Piloting.moveTo(0,0,0,0,10))
    # time.sleep(3)
    assert drone.stop_video_streaming()
    assert drone(Landing()).wait().success()

    # abc = drone.get_state(olympe.messages.ardrone3.PilotingState.moveByChanged) # 안됨
    # abc = drone.get_state(olympe.messages.ardrone3.PilotingState.SpeedChanged) # 잘 됨
    # abc = drone.get_state(olympe.messages.ardrone3.PilotingSettings.CirclingRadius) # 안됨
    # abc = drone.get_state(olympe.messages.ardrone3.Piloting.Circle)
    # abc = drone.get_state(olympe.messages.ardrone3.Piloting.moveBy)
    # abc = drone.get_state(olympe.enums.ardrone3.PilotingState.MoveToChanged_Status)
    # abc = drone.get_state(olympe.enums.ardrone3.PilotingState.MoveByChanged_Status)
    abc = drone.get_state(olympe.messages.ardrone3.PilotingState.AttitudeChanged)
    print(abc['yaw']*180/3.141592)

    # drone_poi = drone.get_state(olympe.messages.ardrone3.PilotingState.AltitudeAboveGroundChanged)
    # alt = drone_poi['altitude']
    # print(1)
    # a = drone(olympe.enums.ardrone3.PilotingState.MoveToChanged_Orientation_mode)
    # print(2)
    # print(type(a))
    # print(a,12312132132)
    # for i in range(10):
    #     print(drone.get_state(olympe.enums.ardrone3.PilotingState.MoveToChanged_Orientation_mode))
    # drone_heading = drone.get_state(olympe.messages.ardrone3.PilotingState.moveToChanged)
    # drone_heading = drone_heading['altitude']
    # for i in range(10):
    #     print(alt)
    #     print(drone_poi)
    #     print(drone_heading)

    # # # a = drone(olympe.log.get_config({
    # # #     "handlers": {
    # # #         "olympe_log_file": {
    # # #             "class": "logging.FileHandler",
    # # #             "formatter": "default_formatter",
    # # #             "filename": "olympe.log"
    # # #         },
    # # #         "ulog_log_file": {
    # # #             "class": "logging.FileHandler",
    # # #             "formatter": "default_formatter",
    # # #             "filename": "ulog.log"
    # # #         },
    # # #     },
    # # #     "loggers": {
    # # #         "olympe": {
    # # #             "handlers": ["console", "olympe_log_file"]
    # # #         },
    # # #         "ulog": {
    # # #             "level": "DEBUG",
    # # #             "handlers": ["console", "ulog_log_file"],
    # # #         }
    # # #     }
    # # # })
    # # # )
    # # # print(123,type(a))
    # # # print(len(a))
    # # # # print(a.values)
    # # # print(a.get('loggers'))
    # # print(olympe.log.get_config)
    # # print(123)
    # # print(olympe.log.set_config)
    # # print(124)
    # # print(olympe.log.update_config)
    # # print(125)
    # # print(olympe.log)
    # # # print(drone(olympe.log.get_config))
    # print(drone(olympe.log))
    # print(123)
    
    # print(a.log)
    # print(getLogger("olympe.drone"))
    # print(type(getLogger("olympe.drone")))
    # print(a.log)

    drone.disconnect()
    # print(a.log)