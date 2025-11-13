#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>

#include "sensor_thread.h"

#define BIT(n) (1UL << (n))
#define BUS_SIZE 3
#define PRESSED 0

#define MODE_NORMAL 1
#define MODE_BLUE   2
#define MODE_OFF    3

#define LIGHT_SENSITIVITY 8

static const struct gpio_dt_spec ledBlue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec ledRed = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec ledGreen = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static volatile bool isr_btn_event = false;
static volatile bool long_press_timeout = false;
static bool short_press = false;

static int8_t mode = MODE_NORMAL;

struct bus_out {
    struct gpio_dt_spec pins[BUS_SIZE];
    size_t pin_count;
};

struct bus_out my_bus_out = {
    .pins = {
        ledBlue,
        ledGreen,
        ledRed,
    },
    .pin_count = BUS_SIZE,
};

static struct gpio_callback button_cb;

void timeout_handler(struct k_timer *timer_id) {
    ARG_UNUSED(timer_id);
    long_press_timeout = true;
}
K_TIMER_DEFINE(my_timeout, timeout_handler, NULL);

static void button_isr (const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    isr_btn_event = true;
}

int bus_out_init (struct bus_out *bus)
{
    for (size_t i = 0; i < bus->pin_count; i++) {
        if(!device_is_ready(bus->pins[i].port)) {
            printk("GPIO device not ready for pin %d\n", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&bus->pins[i], GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            printk("Failed to configure output pin %d\n", i);
            return ret;
        }
    }
    return 0;
}

int bus_out_write (struct bus_out *bus, int8_t value)
{
    for (size_t i = 0; i < bus->pin_count; i++) {
        int8_t bit_val = (value >> i) & 0x1;
        int ret = gpio_pin_set_dt(&bus->pins[i], bit_val);
        if (ret != 0) {
            printk("Failed to write pin %d\n", i);
            return ret;
        }

        //printk("LED %d has bit value: %d\n", i, bit_val);

    }
    return 0;
}

int main(void)
{
    printk("Embedded Platforms and Communications for IoT\n");
    printk("        ETSIST - UPM - MUIoT 2025-2026       \n\n");
    printk("    Board LED toggle (single thread: main)   \n");

    // Ensure the GPIO device is ready
    if (!device_is_ready(ledRed.port) ||!device_is_ready(ledBlue.port) || !device_is_ready(ledGreen.port)) {
        printk("Error: LED device not ready\n");
        return 0;
    }

    // Configure as output and start LOW (off)
    if (gpio_pin_configure_dt(&ledRed, GPIO_OUTPUT_INACTIVE) < 0 || gpio_pin_configure_dt(&ledBlue, GPIO_OUTPUT_INACTIVE) || gpio_pin_configure_dt(&ledGreen, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("Error: configuring LED\n");
        return 0;
    }

    if (bus_out_init(&my_bus_out) != 0) {
        return 0;
    }

    if (!device_is_ready(button.port)) {
        printk("Error: GPIO device %s not ready\n", button.port->name);
        return 0;
    }

    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: Failed to configure pin %d\n", ret, button.pin);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Error %d: Failed to configure pin %d\n", ret, button.pin);
        return 0;
    }

    gpio_init_callback(&button_cb, button_isr, BIT(button.pin));

    ret = gpio_add_callback_dt(&button, &button_cb);
    if (ret != 0) {
        printk("Error %d: gpio_add_callback_dt failed\n", ret);
        return 0;
    }

    sensor_thread_start();

    sensor_thread_set_enabled(true);  


    // LED related variables
    int8_t count = 0;
    while (1) {
        if (long_press_timeout) {
            long_press_timeout = false;
            if (!short_press) {
                mode = MODE_OFF;
                bus_out_write(&my_bus_out, 0);
                sensor_thread_set_enabled(false);
                printk("System OFF\n");
            }
        }

        if (mode == MODE_OFF) {
        k_msleep(10);
        continue;
        }
        
        // User button related code
        if (isr_btn_event) {
            isr_btn_event = false;
            bool state = gpio_pin_get_raw(button.port, button.pin);
            if(state == PRESSED) {
                short_press = false;
                k_timer_start(&my_timeout, K_SECONDS(1), K_NO_WAIT);
            } else {
                short_press = true;
                mode = (mode == MODE_NORMAL) ? MODE_BLUE : MODE_NORMAL;
                if (mode == MODE_NORMAL) {
                    sensor_thread_set_enabled(true);    
                } else {
                    sensor_thread_set_enabled(false);  
                }
            }
        }

        // Photoresistor related code
        if (mode == MODE_NORMAL) {
            struct sensor_msg msg;
            if (sensor_thread_try_get(&msg)) {
            float scaled_raw = msg.light_raw * LIGHT_SENSITIVITY;
            float light_pct = (scaled_raw <= 0)   ? 0.0f :
            (scaled_raw >= 4095)? 100.0f :
            (scaled_raw * 100.0f) / 4095.0f;
            
            if (light_pct < 33.0f) {
                count = 4;        // Red
            } else if (light_pct <= 66.0f) {
                count = 6;        // Yellow (Red+Green)
            } else {
                count = 2;        // Green
            }

            float soil_pct = (msg.soil_raw * 100.0f) / 4095.0f;
            if (soil_pct < 0.0f)   soil_pct = 0.0f; //Wet
            if (soil_pct > 100.0f) soil_pct = 100.0f; //Dry
            
            float sens_g = 4096.0f; /* ±2g default */
            if (msg.accel_range == 1) sens_g = 2048.0f;   /* ±4g */
            else if (msg.accel_range == 2) sens_g = 1024.0f; /* ±8g */
            float ax_g = msg.ax_raw / sens_g;
            float ay_g = msg.ay_raw / sens_g;
            float az_g = msg.az_raw / sens_g;

            float rh = 0.0, tc = 0.0;
            if (msg.rh_raw != 0) {
                rh = (125.0 * msg.rh_raw / 65536.0) - 6.0;
                if (rh < 0.0) { rh = 0.0; }
                if (rh > 100.0) { rh = 100.0; }
            }
            if (msg.temp_raw != 0) {
                tc = (175.72 * msg.temp_raw / 65536.0) - 46.85;
            }

            uint8_t R = 0, G = 0, B = 0;

            if (msg.clr_raw > 0) {
                uint32_t r_scaled = (uint32_t)msg.red_raw * 255u;
                uint32_t g_scaled = (uint32_t)msg.grn_raw * 255u;
                uint32_t b_scaled = (uint32_t)msg.blu_raw * 255u;

                R = (uint8_t)(r_scaled / (uint32_t)msg.clr_raw);
                G = (uint8_t)(g_scaled / (uint32_t)msg.clr_raw);
                B = (uint8_t)(b_scaled / (uint32_t)msg.clr_raw);
            }


            }
        } else { // Always MODE_BLUE here
            count = 1;
        }
        bus_out_write(&my_bus_out, count);
            
        k_msleep(10);
    }
}