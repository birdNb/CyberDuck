#ifndef I2C_GPIO_H
#define I2C_GPIO_H

#include <stdint.h>

int i2c_gpio_open(int chip, int sda, int scl, int use_internal_pullup);
void i2c_gpio_close(void);
void i2c_gpio_recover(void);
int i2c_gpio_xfer(uint8_t addr7, const uint8_t *w, int wlen, uint8_t *r, int rlen);
int i2c_gpio_scan(uint8_t *found, int maxn);
const char *i2c_gpio_error(void);

#endif
