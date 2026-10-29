idf_component_register(SRCS "main.c" "pca9685.c" "power.c" "cmd.c" "ik.c" "pose.c" "gait.c" "ina226.c" "safety.c" "pca9554.c" "bt_spp.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver bt nvs_flash)