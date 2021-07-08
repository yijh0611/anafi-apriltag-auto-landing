# -*- coding: UTF-8 -*-

import olympe
from logging import getLogger
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing

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

    a = getLogger("Olympe.drone")
    print(123)
    print(a.log)
    
    assert drone(TakeOff()).wait().success()
    assert drone.start_video_streaming()
    time.sleep(3)
    assert drone.stop_video_streaming()
    assert drone(Landing()).wait().success()

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
    print(123)
    
    print(a.log)
    print(getLogger("olympe.drone"))
    print(type(getLogger("olympe.drone")))
    print(a.log)

    drone.disconnect()
    print(a.log)