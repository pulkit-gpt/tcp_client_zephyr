#include "mlx.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>

/* -------- MLX90614 Definitions -------- */
#define MLX90614_I2C_ADDR   0x5A
#define MLX90614_TEMP_REG  0x07

#define NUM_READINGS       100
#define READ_DELAY_MS      5

static const struct device *mlx_i2c;

/* Initialize MLX90614 */
int mlx_init(const struct device *i2c_dev)
{
    if (!device_is_ready(i2c_dev)) {
        return -ENODEV;
    }

    mlx_i2c = i2c_dev;
    return 0;
}

/* Read averaged temperature */
int mlx_read_avg_temp(float *avg_c, float *avg_f)
{
    if (!mlx_i2c || !avg_c || !avg_f) {
        return -EINVAL;
    }

    uint8_t i2cdata[2];
    float temp_sum = 0.0f;

    for (int i = 0; i < NUM_READINGS; i++) {

        int ret = i2c_burst_read(
            mlx_i2c,
            MLX90614_I2C_ADDR,
            MLX90614_TEMP_REG,
            i2cdata,
            2
        );

        if (ret < 0) {
            return ret;
        }

        float raw_temp = (float)((i2cdata[1] << 8) | i2cdata[0]);
        float temp_c = (raw_temp * 0.02f) - 273.15f;

        temp_sum += temp_c;
        k_msleep(READ_DELAY_MS);
    }

    *avg_c = temp_sum / NUM_READINGS;
    *avg_f = (1.8f * (*avg_c)) + 32.0f + 3.8f;

    return 0;
}
