/*
 * 用户态开漏 I2C：Radxa Dragon Q6A GPIO_24 / GPIO_25
 * （40 针 PIN_3 = I2C6_SDA，PIN_5 = I2C6_SCL）
 *
 * 优先 gpiochip 开漏；若总线被拉死（无上拉），再 mmap TLMM 打开内部上拉。
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
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
#define IN_BIT 0
#define OUT_BIT 1
#define MUX_BIT 2
#define DRV_BIT 6
#define OE_BIT 9
#define EGPIO_BIT 12
#define PULL_UP 3u

#define SDA_IDX 0
#define SCL_IDX 1

static char errbuf[256];
static int chip_fd = -1;
static int line_fd = -1;
static int mem_fd = -1;
static void *tlmm;
static volatile uint32_t *sda_ctl, *sda_io, *scl_ctl, *scl_io;
static int use_tlmm;
static uint64_t half_ticks;

const char *i2c_gpio_error(void) { return errbuf; }

static void set_err(const char *msg) {
    snprintf(errbuf, sizeof(errbuf), "%s: %s", msg, strerror(errno));
}

static inline uint64_t ticks(void) {
    uint64_t t;
    asm volatile("mrs %0, cntvct_el0" : "=r"(t));
    return t;
}

static inline void i2c_delay(void) {
    uint64_t t = ticks() + half_ticks;
    while ((int64_t)(t - ticks()) > 0)
        ;
}

static int gpio_drive(int idx, int high) {
    struct gpio_v2_line_values v;
    memset(&v, 0, sizeof(v));
    v.mask = 1ull << idx;
    v.bits = high ? (1ull << idx) : 0ull;
    if (ioctl(line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) < 0) {
        set_err("gpio set");
        return -1;
    }
    return 0;
}

static int gpio_get_sda(void) {
    struct gpio_v2_line_values v;
    memset(&v, 0, sizeof(v));
    v.mask = 1ull;
    if (ioctl(line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0)
        return -1;
    return (int)(v.bits & 1ull);
}

static int gpio_get_scl(void) {
    struct gpio_v2_line_values v;
    memset(&v, 0, sizeof(v));
    v.mask = 2ull;
    if (ioctl(line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0)
        return -1;
    return (int)((v.bits >> 1) & 1ull);
}

static void tlmm_cfg(volatile uint32_t *ctl) {
    uint32_t v = *ctl;
    v &= ~(0x7u << MUX_BIT);
    v &= ~(1u << EGPIO_BIT);
    v = (v & ~0x3u) | PULL_UP;
    v = (v & ~(0x7u << DRV_BIT)) | (2u << DRV_BIT);
    v &= ~(1u << OE_BIT);
    *ctl = v;
    asm volatile("dsb sy" ::: "memory");
}

static void sda_lo(void) {
    if (use_tlmm) {
        *sda_io = 0;
        *sda_ctl |= (1u << OE_BIT);
        asm volatile("dsb sy" ::: "memory");
    } else {
        gpio_drive(SDA_IDX, 0);
    }
}

static void sda_rel(void) {
    if (use_tlmm) {
        *sda_ctl &= ~(1u << OE_BIT);
        asm volatile("dsb sy" ::: "memory");
    } else {
        gpio_drive(SDA_IDX, 1);
    }
}

static void scl_lo(void) {
    if (use_tlmm) {
        *scl_io = 0;
        *scl_ctl |= (1u << OE_BIT);
        asm volatile("dsb sy" ::: "memory");
    } else {
        gpio_drive(SCL_IDX, 0);
    }
}

static void scl_rel(void) {
    if (use_tlmm) {
        *scl_ctl &= ~(1u << OE_BIT);
        asm volatile("dsb sy" ::: "memory");
    } else {
        gpio_drive(SCL_IDX, 1);
    }
}

static int read_sda(void) {
    if (use_tlmm)
        return (int)(*sda_io & (1u << IN_BIT));
    return gpio_get_sda();
}

static int read_scl(void) {
    if (use_tlmm)
        return (int)(*scl_io & (1u << IN_BIT));
    return gpio_get_scl();
}

static int wait_scl_high(void) {
    for (int i = 0; i < 10000; i++) {
        if (read_scl() > 0)
            return 0;
        i2c_delay();
    }
    snprintf(errbuf, sizeof(errbuf), "SCL 一直为低（缺上拉或被从机拉死）");
    return -1;
}

static void i2c_start(void) {
    sda_rel();
    scl_rel();
    i2c_delay();
    sda_lo();
    i2c_delay();
    scl_lo();
    i2c_delay();
}

static void i2c_stop(void) {
    sda_lo();
    i2c_delay();
    scl_rel();
    wait_scl_high();
    i2c_delay();
    sda_rel();
    i2c_delay();
}

static int write_bit(int bit) {
    if (bit)
        sda_rel();
    else
        sda_lo();
    i2c_delay();
    scl_rel();
    if (wait_scl_high() != 0)
        return -1;
    i2c_delay();
    scl_lo();
    i2c_delay();
    return 0;
}

static int read_bit(void) {
    sda_rel();
    i2c_delay();
    scl_rel();
    if (wait_scl_high() != 0)
        return -1;
    i2c_delay();
    int b = read_sda() > 0;
    scl_lo();
    i2c_delay();
    return b;
}

static int write_byte(uint8_t val) {
    for (int i = 7; i >= 0; i--) {
        if (write_bit((val >> i) & 1) != 0)
            return -1;
    }
    int ack = read_bit();
    if (ack < 0)
        return -1;
    return ack == 0 ? 0 : 1; /* 0=ACK, 1=NACK */
}

static int read_byte(int ack) {
    uint8_t v = 0;
    for (int i = 7; i >= 0; i--) {
        int b = read_bit();
        if (b < 0)
            return -1;
        v = (uint8_t)((v << 1) | (b ? 1 : 0));
    }
    if (write_bit(ack ? 0 : 1) != 0)
        return -1;
    return (int)v;
}

static int request_lines(int chip, int sda, int scl, uint64_t flags) {
    char path[64];
    snprintf(path, sizeof(path), "/dev/gpiochip%d", chip);
    chip_fd = open(path, O_RDONLY);
    if (chip_fd < 0) {
        set_err("open gpiochip");
        return -1;
    }

    struct gpio_v2_line_request req;
    memset(&req, 0, sizeof(req));
    req.offsets[SDA_IDX] = (uint32_t)sda;
    req.offsets[SCL_IDX] = (uint32_t)scl;
    req.num_lines = 2;
    req.config.flags = flags;
    strncpy(req.consumer, "imu-i2c6", sizeof(req.consumer) - 1);

    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        set_err("request GPIO 24/25");
        close(chip_fd);
        chip_fd = -1;
        return -1;
    }
    line_fd = req.fd;
    return 0;
}

static int map_tlmm(int sda, int scl) {
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        set_err("open /dev/mem（内部上拉需要 root）");
        return -1;
    }
    tlmm = mmap(NULL, TLMM_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd,
                (off_t)TLMM_PHYS);
    if (tlmm == MAP_FAILED) {
        set_err("mmap TLMM");
        tlmm = NULL;
        return -1;
    }
    uintptr_t sda_base = (uintptr_t)tlmm + (uintptr_t)sda * GPIO_STRIDE;
    uintptr_t scl_base = (uintptr_t)tlmm + (uintptr_t)scl * GPIO_STRIDE;
    sda_ctl = (volatile uint32_t *)(sda_base + GPIO_CTL);
    sda_io = (volatile uint32_t *)(sda_base + GPIO_IO);
    scl_ctl = (volatile uint32_t *)(scl_base + GPIO_CTL);
    scl_io = (volatile uint32_t *)(scl_base + GPIO_IO);
    tlmm_cfg(sda_ctl);
    tlmm_cfg(scl_ctl);
    use_tlmm = 1;
    return 0;
}

int i2c_gpio_open(int chip, int sda, int scl, int use_internal_pullup) {
    errbuf[0] = 0;
    use_tlmm = 0;
    uint64_t frq;
    asm volatile("mrs %0, cntfrq_el0" : "=r"(frq));
    if (frq == 0)
        frq = 19200000;
    /* ~5 µs 半周期，约 100 kHz 上限，用户态会更慢 */
    half_ticks = (frq * 5ull) / 1000000ull;
    if (half_ticks < 8)
        half_ticks = 8;

    uint64_t flags = GPIO_V2_LINE_FLAG_OUTPUT | GPIO_V2_LINE_FLAG_OPEN_DRAIN |
                     GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
    if (request_lines(chip, sda, scl, flags) != 0) {
        flags = GPIO_V2_LINE_FLAG_OUTPUT | GPIO_V2_LINE_FLAG_OPEN_DRAIN |
                GPIO_V2_LINE_FLAG_BIAS_DISABLED;
        if (request_lines(chip, sda, scl, flags) != 0)
            return -1;
    }

    if (gpio_drive(SDA_IDX, 1) != 0 || gpio_drive(SCL_IDX, 1) != 0)
        return -1;
    /* 有 /dev/mem 时用 TLMM 直接翻转，固件下载会快很多 */
    if (map_tlmm(sda, scl) == 0)
        errbuf[0] = 0;
    else if (use_internal_pullup)
        return -1;
    else
        errbuf[0] = 0;
    i2c_delay();
    if (read_sda() == 0 || read_scl() == 0) {
        snprintf(errbuf, sizeof(errbuf),
                 "SDA/SCL 空闲不为高（SDA=%d SCL=%d）。检查 3.3V、GND 和上拉",
                 read_sda(), read_scl());
        return -1;
    }
    return 0;
}

void i2c_gpio_close(void) {
    if (use_tlmm) {
        if (sda_ctl)
            *sda_ctl &= ~(1u << OE_BIT);
        if (scl_ctl)
            *scl_ctl &= ~(1u << OE_BIT);
    }
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
    sda_ctl = scl_ctl = sda_io = scl_io = NULL;
    use_tlmm = 0;
}

void i2c_gpio_recover(void) {
    sda_rel();
    for (int i = 0; i < 9; i++) {
        scl_lo();
        i2c_delay();
        scl_rel();
        (void)wait_scl_high();
        i2c_delay();
    }
    i2c_stop();
}

static int i2c_gpio_xfer_once(uint8_t addr7, const uint8_t *w, int wlen, uint8_t *r,
                              int rlen) {
    if (wlen < 0 || rlen < 0)
        return -1;
    i2c_start();
    if (wlen > 0) {
        int ack = write_byte((uint8_t)(addr7 << 1));
        if (ack != 0) {
            i2c_stop();
            if (ack > 0)
                snprintf(errbuf, sizeof(errbuf), "地址 0x%02X 无应答", addr7);
            return -1;
        }
        for (int i = 0; i < wlen; i++) {
            ack = write_byte(w[i]);
            if (ack != 0) {
                i2c_stop();
                snprintf(errbuf, sizeof(errbuf), "写 0x%02X 无应答", w[i]);
                return -1;
            }
        }
        if (rlen > 0)
            i2c_start();
    }
    if (rlen > 0) {
        int ack = write_byte((uint8_t)((addr7 << 1) | 1));
        if (ack != 0) {
            i2c_stop();
            if (ack > 0)
                snprintf(errbuf, sizeof(errbuf), "读地址 0x%02X 无应答", addr7);
            return -1;
        }
        for (int i = 0; i < rlen; i++) {
            int v = read_byte(i != rlen - 1);
            if (v < 0) {
                i2c_stop();
                return -1;
            }
            r[i] = (uint8_t)v;
        }
    }
    i2c_stop();
    i2c_delay();
    i2c_delay();
    return 0;
}

int i2c_gpio_xfer(uint8_t addr7, const uint8_t *w, int wlen, uint8_t *r, int rlen) {
    for (int t = 0; t < 5; t++) {
        if (i2c_gpio_xfer_once(addr7, w, wlen, r, rlen) == 0)
            return 0;
        i2c_gpio_recover();
        usleep(500);
    }
    return -1;
}

int i2c_gpio_scan(uint8_t *found, int maxn) {
    int n = 0;
    for (uint8_t a = 0x08; a <= 0x77 && n < maxn; a++) {
        i2c_start();
        int ack = write_byte((uint8_t)(a << 1));
        i2c_stop();
        if (ack == 0)
            found[n++] = a;
    }
    return n;
}
