#include <stdio.h>
#include <string.h>
#include "power.h"
#include "pca9685.h"


void cmd_task(void *arg)
{
    char line[64];
    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (strncmp(line, "!M", 2) == 0) { for (uint8_t i = 0; i <= 17; i++) servo_set_angle(I2C_NUM_0, i, 0); }  // 18 路回中位
        else if (strncmp(line, "!O", 2) == 0) { power_servo_on(); }
        else if (strncmp(line, "!S", 2) == 0) { power_servo_off(); }
        else if (strncmp(line, "!T", 2) == 0) {
            int id = -1, deg = 0;
            if (sscanf(line, "!T %d %d", &id, &deg) == 2 && id >= 0 && id <= 17) {
                /* servo_set_angle(I2C_NUM_0, id, deg)，失败打一行 */
                printf("servo_set_angle(%d, %d) = %s\n", id, deg, esp_err_to_name(servo_set_angle(I2C_NUM_0, id, deg)));
            } else {
                printf("usage: !T <0-17> <deg_x10>\n");
            }
        } else {
            printf("unknown: %s", line);
        }
        fflush(stdout);
    }
}
