idf_component_register(SRCS "main.c" "imu.c" "gait.c"
                    INCLUDE_DIRS "."
                    REQUIRES bluepad32 btstack)
