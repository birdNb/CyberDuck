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

#define TOF_ZONES 64
#define TOF_DEAD_FRAMES 8
#define TOF_MIN_LIVE 32

static uint8_t dead_zone[TOF_ZONES];
static uint8_t invalid_run[TOF_ZONES];
static int dead_reported = -1;

static int zone_reading(const VL53L5CX_ResultsData *res, int i, int *mm)
{
    if (res->nb_target_detected[i] == 0 || !status_ok(res->target_status[i]))
        return 0;
    *mm = (int)res->distance_mm[i];
    if (*mm < 20 || *mm > 4000)
        return 0;
    return 1;
}

static int median_of(const int *v, int n)
{
    int a[TOF_ZONES];
    int i, j, key;

    if (n <= 0)
        return -1;
    for (i = 0; i < n; i++)
        a[i] = v[i];
    for (i = 1; i < n; i++) {
        key = a[i];
        j = i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
    return a[n / 2];
}

/* 和 base 接近的有效点有几个。孤立的假近距离凑不齐这一簇。 */
static int near_count(const int *mm, const int *valid, int base)
{
    int i, n = 0;

    for (i = 0; i < TOF_ZONES; i++) {
        int d;
        if (!valid[i])
            continue;
        d = mm[i] - base;
        if (d < 0)
            d = -d;
        if (d <= 80 || (base > 0 && d * 5 <= base))
            n++;
    }
    return n;
}

/*
 * 距离只有在“还活着的点这一帧全都测到”时才有效。
 * 连续多帧没有读数的点记成坏点，不再要求它。
 * 比整帧中值近很多、又凑不齐 4 个邻近点的，当成坏点的假距离，不拿来当最近值。
 */
static int trusted_mm(const VL53L5CX_ResultsData *res)
{
    int mm[TOF_ZONES];
    int valid[TOF_ZONES];
    int pool[TOF_ZONES];
    int i, n_pool = 0, live = 0, dead_n = 0, best = -1;
    int med;

    for (i = 0; i < TOF_ZONES; i++) {
        valid[i] = zone_reading(res, i, &mm[i]);
        if (valid[i])
            invalid_run[i] = 0;
        else if (invalid_run[i] < 250)
            invalid_run[i]++;
        if (!dead_zone[i] && invalid_run[i] >= TOF_DEAD_FRAMES)
            dead_zone[i] = 1;
        if (dead_zone[i])
            dead_n++;
        else
            live++;
    }
    if (dead_n != dead_reported) {
        fprintf(stderr, "TOF 坏点 %d 个。剩下的点都测到距离，才采用这一帧。\n", dead_n);
        dead_reported = dead_n;
    }
    if (live < TOF_MIN_LIVE)
        return -1;
    for (i = 0; i < TOF_ZONES; i++) {
        if (dead_zone[i])
            continue;
        if (!valid[i])
            return -1;
        pool[n_pool++] = mm[i];
    }
    med = median_of(pool, n_pool);
    for (i = 0; i < TOF_ZONES; i++) {
        int isolated;
        if (dead_zone[i] || !valid[i])
            continue;
        isolated = (mm[i] + 100 < med) && near_count(mm, valid, mm[i]) < 4;
        if (isolated)
            continue;
        if (best < 0 || mm[i] < best)
            best = mm[i];
    }
    return best;
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
    int machine = 0;
    int i;

    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-' && argv[i][1] == 'p')
            pullup = 1;
        if (argv[i][0] == '-' && argv[i][1] == 'm')
            machine = 1;
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

    if (!machine)
        printf("\033[2J");
    while (running) {
        status = vl53l5cx_check_data_ready(&dev, &ready);
        if (status == 0 && ready) {
            if (vl53l5cx_get_ranging_data(&dev, &results) == 0) {
                if (machine) {
                    printf("D %d\n", trusted_mm(&results));
                    fflush(stdout);
                } else {
                    print_grid(&results, VL53L5CX_RESOLUTION_8X8);
                }
            }
        }
        usleep(20000);
    }

    vl53l5cx_stop_ranging(&dev);
    i2c_gpio_close();
    printf("已停止\n");
    return 0;
}
