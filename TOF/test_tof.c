/*
 * VL53L5CX 8x8 测距：I2C6 = GPIO_24(SDA) / GPIO_25(SCL)
 * 对应官方 VL53L5CX_SimpleRanging 流程（ULD 初始化 + 连续测距）。
 */
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#include "i2c_gpio.h"
#include "vl53l5cx_api.h"

#define GPIO_CHIP 4
#define SDA_LINE 24
#define SCL_LINE 25

static volatile int running = 1;

static void on_stop(int sig)
{
    (void)sig;
    running = 0;
}

static int status_ok(uint8_t st)
{
    /* 5=valid, 9=valid wraparound */
    return st == 5 || st == 9;
}

static void print_grid(const VL53L5CX_ResultsData *res, int res_zones)
{
    int n = (res_zones == VL53L5CX_RESOLUTION_8X8) ? 8 : 4;
    int r, c;

    printf("\033[H");
    printf("VL53L5CX  %dx%d  单位 mm    状态 5/9 有效    Ctrl+C 退出\n\n", n, n);
    for (r = 0; r < n; r++) {
        for (c = n - 1; c >= 0; c--) {
            int idx = r * n + c;
            uint8_t st = res->target_status[idx];
            if (res->nb_target_detected[idx] > 0 && status_ok(st))
                printf("%5d ", (int)res->distance_mm[idx]);
            else
                printf("   -- ");
        }
        printf("\n");
    }
    printf("\n");
    fflush(stdout);
}

int main(int argc, char **argv)
{
    VL53L5CX_Configuration dev;
    VL53L5CX_ResultsData results;
    uint8_t alive = 0, ready = 0, status;
    int pullup = 0;
    int i;

    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-' && argv[i][1] == 'p')
            pullup = 1;
    }

    signal(SIGINT, on_stop);
    signal(SIGTERM, on_stop);

    if (i2c_gpio_open(GPIO_CHIP, SDA_LINE, SCL_LINE, pullup) != 0) {
        fprintf(stderr, "打开 I2C6 失败: %s\n", i2c_gpio_error());
        return 1;
    }
    usleep(20000);

    dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    status = vl53l5cx_is_alive(&dev, &alive);
    if (!alive) {
        fprintf(stderr, "VL53L5CX 未响应（status=%u alive=%u）。确认 0x29 在 I2C6 上。\n",
                status, alive);
        i2c_gpio_close();
        return 1;
    }
    if (status)
        fprintf(stderr, "警告: is_alive status=%u，但器件已识别，继续初始化\n", status);
    fprintf(stderr, "VL53L5CX 在线，开始下载固件（软件 I2C，可能需要几十秒）…\n");

    status = vl53l5cx_init(&dev);
    if (status) {
        fprintf(stderr, "vl53l5cx_init 失败: %u\n", status);
        i2c_gpio_close();
        return 1;
    }

    status = vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);
    status |= vl53l5cx_set_ranging_frequency_hz(&dev, 5);
    status |= vl53l5cx_set_ranging_mode(&dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    status |= vl53l5cx_start_ranging(&dev);
    if (status) {
        fprintf(stderr, "启动测距失败: %u\n", status);
        i2c_gpio_close();
        return 1;
    }

    printf("\033[2J");
    while (running) {
        status = vl53l5cx_check_data_ready(&dev, &ready);
        if (status == 0 && ready) {
            if (vl53l5cx_get_ranging_data(&dev, &results) == 0)
                print_grid(&results, VL53L5CX_RESOLUTION_8X8);
        }
        usleep(20000);
    }

    vl53l5cx_stop_ranging(&dev);
    i2c_gpio_close();
    printf("已停止\n");
    return 0;
}
