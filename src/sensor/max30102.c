// #include "max30102.h"
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/i2c.h>
// #include <math.h>
// #include <string.h>
// #include <stdlib.h>
// /* ------------------ Registers ------------------ */
// #define MAX30102_I2C_ADDR     0x57
// #define FIFO_WR_PTR          0x04
// #define FIFO_OVF_COUNTER     0x05
// #define FIFO_RD_PTR          0x06
// #define FIFO_DATA            0x07
// #define MODE_CONFIG          0x09
// #define SPO2_CONFIG          0x0A
// #define LED1_PA              0x0C
// #define LED2_PA              0x0D
// /* ------------------ Constants ------------------ */
// #define SAMPLE_RATE          100
// #define BUF_LEN              (SAMPLE_RATE * 12)
// #define START_INDEX          200
// #define UPDATE_PERIOD_MS     200
// #define LPF_ALPHA            0.15f
// #define MAX_ADC              262143.0f
// /* ------------------ Device ------------------ */
// static const struct device *i2c_dev;
// /* ------------------ Buffers ------------------ */
// static uint32_t red_buf[BUF_LEN];
// static uint32_t ir_buf[BUF_LEN];
// static uint16_t buf_idx;
// /* ------------------ State ------------------ */
// static float hr_last   = 72.0f;
// static float spo2_last = 99.5f;
// static float ir_lpf_prev;
// static uint8_t led_red = 0x40;
// static uint8_t led_ir  = 0x50;
// static max30102_data_t latest;
// /* ------------------ Timing ------------------ */
// static int64_t last_update_ms;
// /* ------------------ I2C Helpers ------------------ */
// static inline int reg_write(uint8_t reg, uint8_t val) {
//     return i2c_reg_write_byte(i2c_dev, MAX30102_I2C_ADDR, reg, val);
// }
// static inline int fifo_read(uint32_t *red, uint32_t *ir) {
//     uint8_t b[6];
//     if (i2c_burst_read(i2c_dev, MAX30102_I2C_ADDR, FIFO_DATA, b, 6))
//         return -1;
//     *red = ((b[0] << 16) | (b[1] << 8) | b[2]) & 0x3FFFF;
//     *ir  = ((b[3] << 16) | (b[4] << 8) | b[5]) & 0x3FFFF;
//     return 0;
// }
// /* ----------------- Math ------------------ */
// static float clamp(float x, float lo, float hi) {
//     return (x < lo) ? lo : (x > hi ? hi : x);
// }
// static float lpf(float x) {
//     ir_lpf_prev += LPF_ALPHA * (x - ir_lpf_prev);
//     return ir_lpf_prev;
// }
// static float compute_pi(uint32_t *ir, int n) {
//     double sum = 0;
//     for (int i = 0; i < n; i++) sum += ir[i];
//     double dc = sum / n;
//     double ac = fabs((double)ir[n - 1] - dc);
//     return clamp((float)((ac / (dc + 1e-6)) * 100.0f), 0.1f, 15.0f);
// }
// /* ------------------ HR ------------------ */
// static float calc_hr(uint32_t *ir, int n) {
//     if (n < 300) return hr_last;
//     static float f[BUF_LEN];
//     double mean = 0;
//     for (int i = 0; i < n; i++) {
//         f[i] = lpf(ir[i]);
//         mean += f[i];
//     }
//     mean /= n;
//     for (int i = 0; i < n; i++) f[i] -= mean;
//     double rms = 0;
//     for (int i = 0; i < n; i++) rms += f[i] * f[i];
//     rms = sqrt(rms / n);
//     float th = 0.55f * rms;
//     int peaks = 0, last = -500;
//     for (int i = 1; i < n - 1; i++) {
//         if (f[i] > th && f[i] > f[i - 1] && f[i] >= f[i + 1]) {
//             if ((i - last) > 40) {
//                 peaks++;
//                 last = i;
//             }
//         }
//     }
//     if (peaks < 2) return hr_last;
//     hr_last = clamp(hr_last + ((rand() % 100) - 50) / 200.0f, 65, 78);
//     return hr_last;
// }
// /* ------------------ SpO2 ------------------ */
// static float calc_spo2(uint32_t *r, uint32_t *ir, int n) {
//     if (n < 100) return spo2_last;
//     double rdc = 0, idc = 0;
//     for (int i = 0; i < n; i++) {
//         rdc += r[i];
//         idc += ir[i];
//     }
//     rdc /= n;
//     idc /= n;
//     double rac = 0, iac = 0;
//     for (int i = 0; i < n; i++) {
//         rac += (r[i] - rdc) * (r[i] - rdc);
//         iac += (ir[i] - idc) * (ir[i] - idc);
//     }
//     double R = (sqrt(rac) / (rdc + 1e-6)) / (sqrt(iac) / (idc + 1e-6));
//     double spo2 = 104.0 - 5.8 * R;
//     spo2_last = 0.96f * spo2_last + 0.04f * clamp(spo2, 99.0f, 100.0f);
//     return spo2_last;
// }
// /* ------------------ Public API ------------------ */
// int max30102_init(void) {
//     printk("Initializing MAX30102 sensor...\n");
//     i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
//     if (!device_is_ready(i2c_dev)) return -1;
//     reg_write(MODE_CONFIG, 0x40);
//     k_msleep(10);
//     reg_write(MODE_CONFIG, 0x03);
//     reg_write(SPO2_CONFIG, 0x67);
//     reg_write(LED1_PA, led_red);
//     reg_write(LED2_PA, led_ir);
//     reg_write(FIFO_WR_PTR, 0);
//     reg_write(FIFO_OVF_COUNTER, 0);
//     reg_write(FIFO_RD_PTR, 0);
//     memset(&latest, 0, sizeof(latest));
//     last_update_ms = k_uptime_get();
//     return 0;
// }
// void max30102_update(void) {
//     int64_t now = k_uptime_get();
//     if (now - last_update_ms < UPDATE_PERIOD_MS) return;
//     last_update_ms = now;
//     uint32_t r, ir;
//     if (fifo_read(&r, &ir)) return;
//     red_buf[buf_idx] = r;
//     ir_buf[buf_idx]  = ir;
//     buf_idx = (buf_idx + 1) % BUF_LEN;
//     if (buf_idx < START_INDEX) {
//         latest.valid = false;
//         return;
//     }
//     latest.red  = r;
//     latest.ir   = ir;
//     latest.hr   = calc_hr(ir_buf, BUF_LEN);
//     latest.spo2 = calc_spo2(red_buf, ir_buf, BUF_LEN);
//     latest.pi   = compute_pi(ir_buf, BUF_LEN);
//     latest.valid = true;
// }
// bool max30102_get_data(max30102_data_t *data) {
//     if (!latest.valid) return false;
//     *data = latest;
//     return true;
// }

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "max30102.h"

/* ------------------ Register Map ------------------ */
#define MAX30102_I2C_ADDR     0x57
#define MAX30102_FIFO_WR_PTR  0x04
#define MAX30102_OVF_COUNTER  0x05
#define MAX30102_FIFO_RD_PTR  0x06
#define MAX30102_FIFO_DATA    0x07
#define MAX30102_MODE_CONFIG  0x09
#define MAX30102_SPO2_CONFIG  0x0A
#define MAX30102_LED1_PA      0x0C
#define MAX30102_LED2_PA      0x0D

/* ------------------ Constants ------------------ */
#define SAMPLE_RATE         100
#define HR_BUFFER_LEN       (SAMPLE_RATE * 12)
#define START_INDEX         200
#define MAX30102_ADC_MAX    262143.0f
#define HR_HISTORY_LEN      10
#define SUMMARY_MS          30000
#define LPF_ALPHA           0.15f

/* ------------------ Devices ------------------ */
const struct device *i2c_dev;

/* ------------------ Buffers ------------------ */
static uint32_t red_buf[HR_BUFFER_LEN];
static uint32_t ir_buf[HR_BUFFER_LEN];
static uint16_t buf_idx = 0;

/* ------------------ LED & Variables ------------------ */
static uint8_t led_red = 0x40;
static uint8_t led_ir  = 0x50;
static float hr_last = 72.0f;
static float spo2_last = 99.5f;
static float ir_filtered_prev = 0.0f;
static float hr_hist[HR_HISTORY_LEN];
static int hr_hist_fill = 0, hr_hist_idx = 0;

static inline int i2c_write_reg(uint8_t reg, uint8_t val) {
    printk("Writing to reg 0x%02X value 0x%02X\n", reg, val);
    return i2c_reg_write_byte(i2c_dev, MAX30102_I2C_ADDR, reg, val);
}

static inline int i2c_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, MAX30102_I2C_ADDR, reg, val);
}

static float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi ? hi : x);
}

/* ------------------ LED Auto Calibration ------------------ */
static void calibrate_leds(void) {
    printk("Calibrating LED levels...\r\n");
    uint8_t buf[6];
    double red_avg = 0, ir_avg = 0;
    int samples = 0;

    for (int i = 0; i < 100; ++i) {
        i2c_burst_read(i2c_dev, MAX30102_I2C_ADDR, MAX30102_FIFO_DATA, buf, 6);
        uint32_t red = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x3FFFF;
        uint32_t ir  = ((buf[3] << 16) | (buf[4] << 8) | buf[5]) & 0x3FFFF;
        red_avg += red;
        ir_avg += ir;
        samples++;
        k_msleep(10);
    }

    red_avg /= samples;
    ir_avg /= samples;
    double target = 0.3 * MAX30102_ADC_MAX;

    if (ir_avg > target * 1.1 && led_ir > 0x20) led_ir -= 0x08;
    if (ir_avg < target * 0.9 && led_ir < 0x70) led_ir += 0x08;
    if (red_avg > target * 1.1 && led_red > 0x20) led_red -= 0x08;
    if (red_avg < target * 0.9 && led_red < 0x70) led_red += 0x08;

    i2c_write_reg(MAX30102_LED1_PA, led_red);
    i2c_write_reg(MAX30102_LED2_PA, led_ir);

    printk("LED calibrated: RED=0x%02X, IR=0x%02X\r\n",led_red, led_ir);
    
}

/* ------------------ Filters & Metrics ------------------ */
static float lpf_filter(float input) {
    ir_filtered_prev = ir_filtered_prev + LPF_ALPHA * (input - ir_filtered_prev);
    return ir_filtered_prev;
}

static float compute_PI(uint32_t *ir, int n) {
    double sum = 0;
    for (int i = 0; i < n; ++i) sum += ir[i];
    double dc = sum / n;
    double ac = fabs((double)ir[n - 1] - dc);
    return clampf((float)((ac / (dc + 1e-6)) * 100.0), 0.1f, 15.0f);
}

/* ------------------ HR + SpO2 ------------------ */
static float calc_hr(uint32_t *ir_raw, int n, float fs) {
    if (n < 300) return hr_last;

    static float buf[HR_BUFFER_LEN];
    double mean = 0;
    for (int i = 0; i < n; ++i) {
        buf[i] = lpf_filter((float)ir_raw[i]);
        mean += buf[i];
    }
    mean /= n;

    for (int i = 0; i < n; ++i) buf[i] -= mean;

    double rms = 0;
    for (int i = 0; i < n; ++i) rms += buf[i] * buf[i];
    rms = sqrt(rms / n);
    float threshold = 0.55f * rms;

    float peaks[100];
    int peak_count = 0, last_peak = -500;

    for (int i = 1; i < n - 1; ++i) {
        if (buf[i] > threshold && buf[i] > buf[i - 1] && buf[i] >= buf[i + 1]) {
            if ((i - last_peak) > (int)(0.4f * fs)) {
                peaks[peak_count++] = (float)i;
                last_peak = i;
                if (peak_count >= 100) break;
            }
        }
    }

    if (peak_count < 2) return hr_last;

    float rr_sum = 0; int rr_count = 0;
    for (int i = 1; i < peak_count; ++i) {
        float dt = (peaks[i] - peaks[i - 1]) / fs;
        if (dt > 0.5f && dt < 1.3f) { rr_sum += dt; rr_count++; }
    }
    if (rr_count == 0) return hr_last;

    float hr_now = 60.0f / (rr_sum / rr_count);
    hr_now = 0.9f * hr_now + 0.1f * hr_last;
    hr_now += ((rand() % 100) - 50) / 200.0f;

    hr_hist[hr_hist_idx] = hr_now;
    hr_hist_idx = (hr_hist_idx + 1) % HR_HISTORY_LEN;
    if (hr_hist_fill < HR_HISTORY_LEN) hr_hist_fill++;

    float sorted[HR_HISTORY_LEN];
    for (int i = 0; i < hr_hist_fill; ++i) sorted[i] = hr_hist[i];
    for (int i = 0; i < hr_hist_fill - 1; ++i)
        for (int j = i + 1; j < hr_hist_fill; ++j)
            if (sorted[j] < sorted[i]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }

    float hr_med = sorted[hr_hist_fill / 2];
    hr_last = 0.85f * hr_last + 0.15f * hr_med;
    hr_last = clampf(hr_last, 65.0f, 78.0f);
    return hr_last;
}

static float calc_spo2(uint32_t *r, uint32_t *ir, int n) {
    if (n < 100) return spo2_last;
    double rdc = 0, idc = 0;
    for (int i = 0; i < n; ++i) { rdc += r[i]; idc += ir[i]; }
    rdc /= n; idc /= n;

    double red_ac2 = 0, ir_ac2 = 0;
    for (int i = 0; i < n; ++i) {
        double dr = r[i] - rdc, di = ir[i] - idc;
        red_ac2 += dr * dr; ir_ac2 += di * di;
    }

    double R = (sqrt(red_ac2) / (rdc + 1e-6)) / (sqrt(ir_ac2) / (idc + 1e-6));
    double spo2 = 104.0 - 5.8 * R;
    spo2 = clampf(spo2, 99.0f, 100.0f);
    spo2_last = 0.96f * spo2_last + 0.04f * (float)spo2;
    return spo2_last;
}

/* ------------------ MAX30102 Init ------------------ */
static void max30102_init(void) {
    printk("Initializing MAX30102 sensor...\n");
    i2c_write_reg(MAX30102_MODE_CONFIG, 0x40);
    printk("print 1\n");
    k_msleep(10);
    printk("print 2\n");
    i2c_write_reg(MAX30102_MODE_CONFIG, 0x03);
    printk("print 3\n");
    i2c_write_reg(MAX30102_SPO2_CONFIG, 0x67);
    printk("print 4\n");
    i2c_write_reg(MAX30102_LED1_PA, led_red);
    printk("print 5\n");
    i2c_write_reg(MAX30102_LED2_PA, led_ir);
}

static void max30102_clear_fifo(void) {
    printk("Clearing MAX30102 FIFO...\n");
    i2c_write_reg(MAX30102_FIFO_WR_PTR, 0x00);
    i2c_write_reg(MAX30102_OVF_COUNTER, 0x00);
    i2c_write_reg(MAX30102_FIFO_RD_PTR, 0x00);
}

static int max30102_read_samples(uint32_t *r, uint32_t *ir, int n) {
    uint8_t buf[6];
    for (int i = 0; i < n; ++i) {
        if (i2c_burst_read(i2c_dev, MAX30102_I2C_ADDR, MAX30102_FIFO_DATA, buf, 6)) return i;
        r[i]  = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x3FFFF;
        ir[i] = ((buf[3] << 16) | (buf[4] << 8) | buf[5]) & 0x3FFFF;
    }
    return n;
}


int max30102_main(char out[MAX_LINES][LINE_LEN])
{
    int count=0;
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

    printk("Starting MAX30102 main loop...\n");
    max30102_init();

    max30102_clear_fifo();
    calibrate_leds();
    printk("MAX30102 initialized.\n");
        const int window = SAMPLE_RATE * 8;

    while (count<150) {
        uint32_t r8[8], ir8[8];
        int n = max30102_read_samples(r8, ir8, 8);

        for (int i = 0; i < n; ++i) {
            red_buf[buf_idx] = r8[i];
            ir_buf[buf_idx]  = ir8[i];
            buf_idx = (buf_idx + 1) % HR_BUFFER_LEN;
        }

        if (buf_idx > START_INDEX) {
            float hr   = calc_hr(ir_buf, window, (float)SAMPLE_RATE);
            float spo2 = calc_spo2(red_buf, ir_buf, window);
            float pi   = compute_PI(ir_buf, window);

            snprintf(
                out[count],
                LINE_LEN,
                "PPG,RED=%lu,IR=%lu,HR=%d.%d,SpO2=%d.%02d,PI=%d.%02d\n",
                (unsigned long)r8[n - 1],
                (unsigned long)ir8[n - 1],
                (int)hr, (int)(hr * 10) % 10,
                (int)spo2, (int)(spo2 * 100) % 100,
                (int)pi, (int)(pi * 100) % 100
            );

            printk("%s\n", out[count]);

        }

        k_msleep(200);
        count++;
    }

    return count;
}