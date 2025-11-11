#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>
#include "sensor_thread.h"

#define BUFFER_SIZE 1
#define SENSITIVITY 8
#define SENSOR_STACK_SZ  1024
#define SENSOR_PRIO      5
K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_STACK_SZ);

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
static struct k_thread sensor_thr;

static struct sensor_msg latest;
static struct k_mutex latest_mtx;

static bool enabled = false;
static struct k_mutex  enable_mtx;
static struct k_condvar enable_cv;

void sensor_thread_set_enabled(bool en)
{
    k_mutex_lock(&enable_mtx, K_FOREVER);
    bool was = enabled;
    enabled = en;
    if (en && !was) {
        k_condvar_signal(&enable_cv);
    }
    k_mutex_unlock(&enable_mtx);
}

bool sensor_thread_try_get(struct sensor_msg *out)
{
    if (!out) return false;
    k_mutex_lock(&latest_mtx, K_FOREVER);
    *out = latest;
    k_mutex_unlock(&latest_mtx);
    return true;
}

static int16_t sample_buffer[BUFFER_SIZE];

static struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 0,
};

int read_adc_raw (int16_t *raw_val)
{
    if (!device_is_ready(adc_dev)) {
        printk("Error: ADC device is not ready\n");
        return 0;
    }

    const struct adc_sequence sequence = {
        .channels = BIT(0),
        .buffer = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = 12,
    };

    int8_t ret = adc_read(adc_dev, &sequence);
    if (ret < 0) {
        printk("Error: ADC read_dt failed: %d\n", ret);
        return ret;
    }

    *raw_val = sample_buffer[0];
    return 0;
}

static void sensor_entry(void *a, void *b, void *c                                                                                                                                                                                                                                                                                                                                                                                                )
{
    int8_t ret = adc_channel_setup(adc_dev, &channel_cfg);
    if (ret < 0) {
        printk("ADC channel setup failed: %d\n", ret);
        return;
    }

    k_mutex_init(&latest_mtx);

    while (1) {
        k_mutex_lock(&enable_mtx, K_FOREVER);
        while (!enabled) {
            k_condvar_wait(&enable_cv, &enable_mtx, K_FOREVER);
        }
        k_mutex_unlock(&enable_mtx);
        
        int16_t raw = 0;
        if (read_adc_raw(&raw) == 0) {
            raw = raw * SENSITIVITY;
            float pct = (raw <= 0) ? 0.0f :
                        (raw >= 4095) ? 100.0f :
                        ((float)raw * 100.0f) / 4095.0f;

            k_mutex_lock(&latest_mtx, K_FOREVER);
            latest.raw = raw;
            latest.pct = pct;
            k_mutex_unlock(&latest_mtx);
        }

        k_msleep(2000);
    }
}

void sensor_thread_start(void)
{
    k_mutex_init(&enable_mtx);
    k_condvar_init(&enable_cv);
    enabled = false;

    k_thread_create(&sensor_thr, sensor_stack, SENSOR_STACK_SZ,
                    sensor_entry, NULL, NULL, NULL,
                    SENSOR_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thr, "sensor_thr");
}
