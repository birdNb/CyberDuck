/*
 * WS2812 单总线：Radxa Dragon Q6A GPIO_22（40 针 PIN_8）。
 *
 * 该脚默认是 UART5_TX。占用期间必须保持 GPIO 模式，关灯后不要把串口绑回去，
 * 否则 TX 空闲高电平 / 控制台输出会再次点亮灯珠。
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define TLMM_PHYS 0x0f100000ULL
#define TLMM_MAP_SIZE 0x100000
#define GPIO_STRIDE 0x1000
#define GPIO_CTL 0x0
#define GPIO_IO 0x4
#define OUT_BIT 1
#define OE_BIT 9
#define MUX_BIT 2
#define DRV_BIT 6
#define EGPIO_BIT 12

#define T1H_NS 700
#define T1L_NS 600
#define T0_PERIOD_NS 1250
#define RESET_NS 300000

static char errbuf[256];
static int chip_fd = -1;
static int line_fd = -1;
static int mem_fd = -1;
static int nleds;
static int took_uart;
static void *tlmm;
static volatile uint32_t *io_reg;
static volatile uint32_t *ctl_reg;
static uint64_t cntfrq;
static uint64_t t1h, t1l, t0period, treset;

const char *ws2812_error(void) { return errbuf; }

static void set_err(const char *msg) {
    snprintf(errbuf, sizeof(errbuf), "%s: %s", msg, strerror(errno));
}

static inline uint64_t ticks(void) {
    uint64_t t;
    asm volatile("mrs %0, cntvct_el0" : "=r"(t));
    return t;
}

static inline uint64_t ns_to_ticks(uint64_t ns) {
    return (ns * cntfrq) / 1000000000ULL;
}

static inline void wait_until(uint64_t deadline) {
    while ((int64_t)(deadline - ticks()) > 0)
        ;
}

static inline void gpio_hi(void) {
    *io_reg = (1u << OUT_BIT);
}

static inline void gpio_lo(void) {
    *io_reg = 0;
}

static int unbind_debug_uart(void) {
    int fd = open("/sys/bus/platform/drivers/qcom_geni_serial/unbind", O_WRONLY);
    if (fd < 0)
        return -1;
    ssize_t n = write(fd, "994000.serial", 13);
    close(fd);
    return n == 13 ? 0 : -1;
}

static int request_gpio_line(int chip, int line) {
    char path[64];
    snprintf(path, sizeof(path), "/dev/gpiochip%d", chip);
    chip_fd = open(path, O_RDONLY);
    if (chip_fd < 0) {
        set_err("open gpiochip");
        return -1;
    }

    struct gpio_v2_line_request req;
    memset(&req, 0, sizeof(req));
    req.offsets[0] = (uint32_t)line;
    req.num_lines = 1;
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT | GPIO_V2_LINE_FLAG_BIAS_DISABLED;
    strncpy(req.consumer, "ws2812", sizeof(req.consumer) - 1);

    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        set_err("request GPIO line");
        close(chip_fd);
        chip_fd = -1;
        return -1;
    }
    line_fd = req.fd;
    return 0;
}

static int map_tlmm(int line) {
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        set_err("open /dev/mem (需要 root)");
        return -1;
    }
    tlmm = mmap(NULL, TLMM_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd,
                (off_t)TLMM_PHYS);
    if (tlmm == MAP_FAILED) {
        set_err("mmap TLMM");
        tlmm = NULL;
        return -1;
    }
    uintptr_t base = (uintptr_t)tlmm + (uintptr_t)line * GPIO_STRIDE;
    ctl_reg = (volatile uint32_t *)(base + GPIO_CTL);
    io_reg = (volatile uint32_t *)(base + GPIO_IO);

    uint32_t ctl = *ctl_reg;
    ctl &= ~(0x7u << MUX_BIT);
    ctl &= ~(1u << EGPIO_BIT);
    ctl |= (1u << OE_BIT);
    ctl &= ~0x3u;
    ctl = (ctl & ~(0x7u << DRV_BIT)) | (7u << DRV_BIT);
    *ctl_reg = ctl;
    asm volatile("dsb sy" ::: "memory");
    gpio_lo();
    return 0;
}

static void setup_realtime(void) {
    (void)mlockall(MCL_CURRENT | MCL_FUTURE);
    struct sched_param sp = {.sched_priority = 80};
    (void)sched_setscheduler(0, SCHED_FIFO, &sp);
}

int ws2812_open(int chip, int line, int led_count) {
    errbuf[0] = 0;
    if (led_count < 1 || led_count > 1024) {
        snprintf(errbuf, sizeof(errbuf), "invalid led count");
        return -1;
    }
    nleds = led_count;
    took_uart = 0;

    asm volatile("mrs %0, cntfrq_el0" : "=r"(cntfrq));
    if (cntfrq == 0)
        cntfrq = 19200000;
    t1h = ns_to_ticks(T1H_NS);
    t1l = ns_to_ticks(T1L_NS);
    t0period = ns_to_ticks(T0_PERIOD_NS);
    treset = ns_to_ticks(RESET_NS);

    if (request_gpio_line(chip, line) != 0) {
        if (line == 22 && unbind_debug_uart() == 0) {
            took_uart = 1;
            usleep(50000);
            if (request_gpio_line(chip, line) != 0)
                return -1;
        } else {
            return -1;
        }
    }

    if (map_tlmm(line) != 0)
        return -1;

    setup_realtime();
    return 0;
}

void ws2812_show(const uint8_t *grb) {
    if (!io_reg || !grb)
        return;

    const int nbytes = nleds * 3;
    for (int i = 0; i < nbytes; i++) {
        uint8_t byte = grb[i];
        for (int b = 7; b >= 0; b--) {
            uint64_t t0 = ticks();
            if ((byte >> b) & 1) {
                gpio_hi();
                wait_until(t0 + t1h);
                gpio_lo();
                wait_until(t0 + t1h + t1l);
            } else {
                /* 0 码：立刻拉低，避免 MMIO 延迟把 0 打成 1（全白发烫） */
                gpio_hi();
                gpio_lo();
                wait_until(t0 + t0period);
            }
        }
    }
    uint64_t t0 = ticks();
    gpio_lo();
    wait_until(t0 + treset);
}

void ws2812_hold_low(void) {
    if (io_reg)
        gpio_lo();
}

void ws2812_close(void) {
    if (io_reg)
        gpio_lo();
    if (tlmm && tlmm != MAP_FAILED) {
        munmap(tlmm, TLMM_MAP_SIZE);
        tlmm = NULL;
    }
    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
    if (line_fd >= 0) {
        close(line_fd);
        line_fd = -1;
    }
    if (chip_fd >= 0) {
        close(chip_fd);
        chip_fd = -1;
    }
    io_reg = NULL;
    ctl_reg = NULL;
}
