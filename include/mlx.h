#ifndef MLX_H
#define MLX_H

#include <zephyr/device.h>

/* Initialize MLX90614 (pass I2C device) */
int mlx_init(const struct device *i2c_dev);

/* Read averaged temperature
 * avg_c  -> averaged temperature in Celsius
 * avg_f  -> averaged temperature in Fahrenheit (with +3.8 offset)
 */
int mlx_read_avg_temp(float *avg_c, float *avg_f);

#endif /* MLX_H */
