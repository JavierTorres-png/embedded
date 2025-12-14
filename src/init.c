/* src/init.c - single init facade implementation
 *
 * - Concrete struct bus_out is defined here (internal).
 * - Provides bus_out_write so main can set RGB pattern.
 */

#include "init.h"
#include "project_config.h"
#include "sensor_processing.h"
#include "sensor_thread.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* ---- concrete bus_out definition used internally and by callers ---- */
struct bus_out {
    struct gpio_dt_spec pins[BUS_SIZE];
    size_t pin_count;
};

/* Device-tree pins (mandatory RGB leds) */
static const struct gpio_dt_spec ledBlue   = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec ledRed    = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec ledGreen  = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* optional onboard leds: use DT_NODE_HAS_STATUS guard */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(blue_led_1), okay)
static const struct gpio_dt_spec onboard_blue_led  = GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_1), gpios);
#else
static const struct gpio_dt_spec onboard_blue_led  = {0};
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(green_led_2), okay)
static const struct gpio_dt_spec onboard_green_led = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios);
#else
static const struct gpio_dt_spec onboard_green_led = {0};
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(red_led_3), okay)
static const struct gpio_dt_spec onboard_red_led = GPIO_DT_SPEC_GET(DT_NODELABEL(red_led_3), gpios);
#else
static const struct gpio_dt_spec onboard_red_led = {0};
#endif

/* button device tree spec */
static const struct gpio_dt_spec button_dt = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

/* default bus instance */
static struct bus_out default_bus = {
    .pins = { ledBlue, ledGreen, ledRed },
    .pin_count = BUS_SIZE,
};

struct bus_out *init_get_default_bus(void)
{
    return &default_bus;
}

/* Button ISR & flags */
static struct gpio_callback button_cb;
static volatile bool button_event = false;

/* Timer flags */
static volatile bool measure_event = false;
static volatile bool read_event = false;
static volatile bool hourly_event = false;

static int current_mode = MODE_TEST;

/* ISR for button - set flag only */
static void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
    button_event = true;
}

/* Ticker callbacks */
static void read_ticker_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    measure_event = true;
    read_event = true;
}

static void hourly_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    hourly_event = true;
}

/* timers */
K_TIMER_DEFINE(read_ticker, read_ticker_handler, NULL);
K_TIMER_DEFINE(hourly_timer, hourly_handler, NULL);

/* Helper: configure a gpio_dt_spec if device ready */
static int configure_dt_pin(const struct gpio_dt_spec *spec, gpio_flags_t flags)
{
    if (!spec || !spec->port) {
        return -ENODEV;
    }
    if (!device_is_ready(spec->port)) {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(spec, flags);
}

/* Bus write implementation (LSB -> pin 0) */
int bus_out_write(struct bus_out *bus, int8_t value)
{
    if (!bus) return -EINVAL;
    for (size_t i = 0; i < bus->pin_count; i++) {
        int bit_val = (value >> i) & 0x1;
        (void)gpio_pin_set_dt(&bus->pins[i], bit_val); /* ignore return for robustness */
    }
    return 0;
}

/* Setup GPIOs, timers, sensor_thread and stats */
int init_system(void)
{
    int rc = 0;

    /* Configure RGB leds */
    rc = configure_dt_pin(&ledBlue, GPIO_OUTPUT_INACTIVE);
    if (rc) return rc;
    rc = configure_dt_pin(&ledRed, GPIO_OUTPUT_INACTIVE);
    if (rc) return rc;
    rc = configure_dt_pin(&ledGreen, GPIO_OUTPUT_INACTIVE);
    if (rc) return rc;

    /* Onboard leds */
    (void)configure_dt_pin(&onboard_blue_led, GPIO_OUTPUT_INACTIVE);
    (void)configure_dt_pin(&onboard_green_led, GPIO_OUTPUT_INACTIVE);
    (void)configure_dt_pin(&onboard_red_led, GPIO_OUTPUT_INACTIVE);

    /* Configure bus pins (idempotent) */
    for (size_t i = 0; i < default_bus.pin_count; i++) {
        rc = configure_dt_pin(&default_bus.pins[i], GPIO_OUTPUT_INACTIVE);
        if (rc) return rc;
    }

    /* Button */
    if (!device_is_ready(button_dt.port)) {
        return -ENODEV;
    }
    rc = gpio_pin_configure_dt(&button_dt, GPIO_INPUT | GPIO_PULL_UP);
    if (rc) return rc;
    rc = gpio_pin_interrupt_configure_dt(&button_dt, GPIO_INT_EDGE_FALLING);
    if (rc) return rc;

    gpio_init_callback(&button_cb, button_isr, BIT(button_dt.pin));
    rc = gpio_add_callback_dt(&button_dt, &button_cb);
    if (rc) return rc;

    /* Start sensor thread and reset stats */
    sensor_thread_start();
    reset_all_stats();

    /* Start in test mode (fast sampling) */
    init_set_test_mode();

    return 0;
}


static void clear_read_flags(void)
{
    measure_event = false;
    read_event = false;
}

/* Mode control */
void init_set_test_mode(void)
{
    current_mode = MODE_TEST;
    k_timer_stop(&hourly_timer);
    k_timer_stop(&read_ticker);
    clear_read_flags();
    k_timer_start(&read_ticker, K_SECONDS(0), K_SECONDS(MODE_TEST_SLEEP_TIME_SEC));

    if (onboard_blue_led.port) (void)gpio_pin_set_dt(&onboard_blue_led, 1);
    if (onboard_green_led.port) (void)gpio_pin_set_dt(&onboard_green_led, 0);
    if (onboard_red_led.port) (void)gpio_pin_set_dt(&onboard_red_led, 0);

    printk("init: TEST mode\n");
}

void init_set_normal_mode(void)
{
    current_mode = MODE_NORMAL;
    reset_all_stats();
    k_timer_stop(&read_ticker);
    clear_read_flags();
    k_timer_start(&read_ticker, K_SECONDS(0), K_SECONDS(MODE_NORMAL_SLEEP_TIME_SEC));
    k_timer_start(&hourly_timer, K_HOURS(HOURLY_STATS_TIME_HOUR), K_HOURS(HOURLY_STATS_TIME_HOUR));

    if (onboard_blue_led.port) (void)gpio_pin_set_dt(&onboard_blue_led, 0);
    if (onboard_green_led.port) (void)gpio_pin_set_dt(&onboard_green_led, 1);
    if (onboard_red_led.port) (void)gpio_pin_set_dt(&onboard_red_led, 0);

    printk("init: NORMAL mode\n");
}

void init_set_advanced_mode(void)
{
    current_mode = MODE_ADVANCED;
    reset_all_stats();
    k_timer_stop(&read_ticker);
    clear_read_flags();
    k_timer_start(&read_ticker, K_SECONDS(0), K_SECONDS(MODE_NORMAL_SLEEP_TIME_SEC));
    k_timer_start(&hourly_timer, K_HOURS(HOURLY_STATS_TIME_HOUR), K_HOURS(HOURLY_STATS_TIME_HOUR));

    if (onboard_blue_led.port) (void)gpio_pin_set_dt(&onboard_blue_led, 0);
    if (onboard_green_led.port) (void)gpio_pin_set_dt(&onboard_green_led, 0);
    if (onboard_red_led.port) (void)gpio_pin_set_dt(&onboard_red_led, 1);

    printk("init: ADVANCED mode\n");
}

int init_toggle_mode(void)
{
    /* Cycle: TEST -> NORMAL -> ADVANCED -> TEST */
    switch (current_mode)
    {
        case MODE_TEST:
            init_set_normal_mode();
            break;
        case MODE_NORMAL:
            init_set_advanced_mode();
            break;
        default:
            init_set_test_mode();
            break;
    }
    return current_mode;
}

int init_get_mode(void)
{
    return current_mode;
}

/* Event consumer functions (polling from main) */
bool init_consume_measure(void)
{
    if (!measure_event) return false;
    measure_event = false;
    return true;
}

bool init_read(void)
{
    if (!read_event) return false;
    return true;
}

void consume_read(void)
{
    read_event = false;
}

bool init_consume_hourly(void)
{
    if (!hourly_event) return false;
    hourly_event = false;
    return true;
}

bool init_consume_button_released(void)
{
    if (!button_event) return false;
    button_event = false;

    int val = gpio_pin_get_raw(button_dt.port, button_dt.pin);
    /* active-low release semantics */
    return (val == 0);
}
