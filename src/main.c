#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>

#include "sensor_thread.h"

#define BIT(n) (1UL << (n))
#define BUS_SIZE 3
#define PRESSED 0

#define MODE_TEST 1
#define MODE_NORMAL 2

#define MODE_TEST_SLEEP_TIME 2000
#define MODE_NORMAL_SLEEP_TIME 30000

#define LIGHT_SENSITIVITY 4

static const struct gpio_dt_spec ledBlue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec ledRed = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec ledGreen = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec onboard_blue_led =GPIO_DT_SPEC_GET(DT_NODELABEL(blue_led_1), gpios);
static const struct gpio_dt_spec onboard_green_led =GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios);

static volatile bool isr_btn_event = false;
static volatile bool long_press_timeout = false;
static volatile bool read_ticker_event = false;
//static bool short_press = false;

static int8_t mode = MODE_TEST;

typedef struct {
    double sum;
    double min;
    double max;
    uint32_t count;
} stats_t;

static stats_t temp_stats;   // Temperature
static stats_t rh_stats;     // Relative humidity
static stats_t light_stats;  // Ambient light
static stats_t soil_stats;   // Soil moisture

static void reset_stat(stats_t *s)
{
    s->sum   = 0.0;
    s->min   = DBL_MAX;
    s->max   = -DBL_MAX;
    s->count = 0;
}

static void add_sample(stats_t *s, double v)
{
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->sum += v;
    s->count++;
}
static uint32_t dom_red_count   = 0;
static uint32_t dom_green_count = 0;
static uint32_t dom_blue_count  = 0;

static void reset_color_counts(void)
{
    dom_red_count   = 0;
    dom_green_count = 0;
    dom_blue_count  = 0;
}
typedef struct {
    double min;
    double max;
} axis_stats_t;

static axis_stats_t ax_stats;
static axis_stats_t ay_stats;
static axis_stats_t az_stats;

static void reset_axis(axis_stats_t *a)
{
    a->min = DBL_MAX;
    a->max = -DBL_MAX;
}

static void add_axis_sample(axis_stats_t *a, double v)
{
    if (v < a->min) a->min = v;
    if (v > a->max) a->max = v;
}

static void reset_all_stats(void)
{
    reset_stat(&temp_stats);
    reset_stat(&rh_stats);
    reset_stat(&light_stats);
    reset_stat(&soil_stats);

    reset_color_counts();

    reset_axis(&ax_stats);
    reset_axis(&ay_stats);
    reset_axis(&az_stats);
}

static volatile bool hourly_stats_event = false;

static void hourly_stats_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    hourly_stats_event = true;
}

K_TIMER_DEFINE(hourly_stats_timer, hourly_stats_handler, NULL);

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

/*void timeout_handler(struct k_timer *timer_id) {
    ARG_UNUSED(timer_id);
    long_press_timeout = true;
}
K_TIMER_DEFINE(my_timeout, timeout_handler, NULL);*/

void ticker_handler(struct k_timer *timer_id) {
    ARG_UNUSED(timer_id);
    read_ticker_event = true;
}
K_TIMER_DEFINE(read_ticker, ticker_handler, NULL);

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

// Convert NMEA (DDMM.MMMM) to decimal degrees
static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4) return 0.0f;

    float value = 0.0f;
    int degrees = 0;
    float minutes = 0.0f;

    // Convert string to number
    for (int i = 0; nmea[i]; i++) {
        if (nmea[i] >= '0' && nmea[i] <= '9') {
            value = value * 10 + (nmea[i] - '0');
        } else if (nmea[i] == '.') {
            float decimal = 0.0f;
            float divisor = 10.0f;
            for (int j = i + 1; nmea[j] >= '0' && nmea[j] <= '9'; j++) {
                decimal += (nmea[j] - '0') / divisor;
                divisor *= 10.0f;
            }
            value += decimal;
            break;
        }
    }

    degrees = (int)(value / 100);
    minutes = value - (degrees * 100);

    float result = degrees + (minutes / 60.0f);

    if (dir == 'S' || dir == 'W') {
        result = -result;
    }
    return result;
}

// Parse & print GPS info from a raw GGA sentence
static void gps_print_from_sentence(const char *sentence)
{
    if (!sentence || sentence[0] == '\0') {
        printk("GPS: no data\n");
        return;
    }

    // Work on a local copy because we will modify it
    char line[GPS_SENTENCE_MAX];
    strncpy(line, sentence, sizeof(line));
    line[sizeof(line) - 1] = '\0';

    char *p = line;
    int field = 0;
    char *fields[15] = {0};

    // Split by commas
    fields[field++] = p;
    while (*p && field < 15) {
        if (*p == ',') {
            *p = '\0';
            fields[field++] = p + 1;
        }
        p++;
    }

    // GGA: $GPGGA, time, lat, N/S, lon, E/W, fix, sats, HDOP, alt, ...
    if (fields[1] && fields[2] && fields[3] && fields[4] && fields[5] && fields[9]) {
        float lat = nmea_to_degrees(fields[2], fields[3][0]);
        float lon = nmea_to_degrees(fields[4], fields[5][0]);

        printk("GPS:\n");
        printk("\tTime: %c%c:%c%c:%c%c\n",
               fields[1][0], fields[1][1],
               fields[1][2], fields[1][3],
               fields[1][4], fields[1][5]);

        printk("\tLat: %.6f° %c\n", (lat >= 0 ? lat : -lat), fields[3][0]);
        printk("\tLon: %.6f° %c\n", (lon >= 0 ? lon : -lon), fields[5][0]);
        printk("\tAlt: %s m\n", fields[9]);
        if (fields[7]) {
            printk("\tSatellites: %s\n", fields[7]);
        }
    } else {
        printk("GPS: incomplete GGA sentence\n");
    }
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

    if (!device_is_ready(onboard_blue_led.port) || !device_is_ready(onboard_green_led.port)) {
        printk("Onboard LED not ready!\n");
    }

    if (gpio_pin_configure_dt(&onboard_blue_led, GPIO_OUTPUT_INACTIVE) < 0 || gpio_pin_configure_dt(&onboard_green_led, GPIO_OUTPUT_INACTIVE) < 0) {
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

    sensor_thread_set_sleep_time(MODE_TEST_SLEEP_TIME);  

    k_timer_start(&read_ticker, K_SECONDS(0.1), K_SECONDS(2));

    gpio_pin_set_dt(&onboard_blue_led, 1);

    reset_all_stats();

    // LED related variables
    int8_t count = 0;
    while (1) {
        // ------------------------------
        // User button related code
        // ------------------------------
        if (isr_btn_event) {
            isr_btn_event = false;
            bool state = gpio_pin_get_raw(button.port, button.pin);

            if (state == PRESSED) {
                /* Button pressed: nothing special for now */
            } else {
                /* Button released: toggle between TEST and NORMAL */
                if (mode == MODE_TEST) {
                    mode = MODE_NORMAL;
                    reset_all_stats();
                    printk("Mode Normal\n");
                    gpio_pin_set_dt(&onboard_blue_led, 0);
                    gpio_pin_set_dt(&onboard_green_led, 1);

                    k_timer_start(&read_ticker,
                                  K_SECONDS(0),
                                  K_SECONDS(MODE_NORMAL_SLEEP_TIME / 1000));
                    /* For real hour use K_HOURS(1); for fast testing, K_MINUTES(1) */
                    k_timer_start(&hourly_stats_timer,
                                  K_MINUTES(1),
                                  K_MINUTES(1));

                    sensor_thread_set_sleep_time(MODE_NORMAL_SLEEP_TIME);
                } else {
                    mode = MODE_TEST;
                    printk("Mode Test\n");
                    gpio_pin_set_dt(&onboard_blue_led, 1);
                    gpio_pin_set_dt(&onboard_green_led, 0);

                    k_timer_start(&read_ticker,
                                  K_SECONDS(0),
                                  K_SECONDS(MODE_TEST_SLEEP_TIME / 1000));
                    sensor_thread_set_sleep_time(MODE_TEST_SLEEP_TIME);
                }
            }
        }

        // ------------------------------
        // Read sensor only when ticker fires
        // ------------------------------
        if (read_ticker_event) {
            read_ticker_event = false;

            struct sensor_msg msg;
            if (sensor_thread_try_get(&msg)) {
                /* ---- Compute all values from this sample ---- */

                float scaled_raw = msg.light_raw * LIGHT_SENSITIVITY;
                float light_pct = (scaled_raw <= 0)   ? 0.0f :
                                  (scaled_raw >= 4095)? 100.0f :
                                  (scaled_raw * 100.0f) / 4095.0f;

                float sens_g = 4096.0f; /* ±2g default */
                if (msg.accel_range == 1) sens_g = 2048.0f;   /* ±4g */
                else if (msg.accel_range == 2) sens_g = 1024.0f; /* ±8g */

                float ax_g = 9.807f * msg.ax_raw / sens_g;
                float ay_g = 9.807f * msg.ay_raw / sens_g;
                float az_g = 9.807f * msg.az_raw / sens_g;

                float rh = 0.0f, tc = 0.0f;
                if (msg.rh_raw != 0) {
                    rh = (125.0f * msg.rh_raw / 65536.0f) - 6.0f;
                    if (rh < 0.0f)   rh = 0.0f;
                    if (rh > 100.0f) rh = 100.0f;
                }
                bool rh_in_range = (rh >= 25.0f) && (rh <= 75.0f);

                if (msg.temp_raw != 0) {
                    tc = (175.72f * msg.temp_raw / 65536.0f) - 46.85f;
                }
                bool temp_in_range = (tc >= -10.0f) && (tc <= 50.0f);

                float soil_pct = (msg.soil_raw * 100.0f) / 4095.0f;
                if (soil_pct < 0.0f)   soil_pct = 0.0f;  // Wet
                if (soil_pct > 100.0f) soil_pct = 100.0f; // Dry

                /* ---- COLOR: dominant + LED pattern ---- */
                uint8_t R = 0, G = 0, B = 0;
                const char *dominant = "";

                if (msg.clr_raw > 0) {
                    uint32_t r_scaled = (uint32_t)msg.red_raw * 255u;
                    uint32_t g_scaled = (uint32_t)msg.grn_raw * 255u;
                    uint32_t b_scaled = (uint32_t)msg.blu_raw * 255u;

                    R = (uint8_t)(r_scaled / (uint32_t)msg.clr_raw);
                    G = (uint8_t)(g_scaled / (uint32_t)msg.clr_raw);
                    B = (uint8_t)(b_scaled / (uint32_t)msg.clr_raw);

                    if (R >= G && R >= B) {
                        count    = 4;   // Red dominant
                        dominant = "RED";
                        dom_red_count++;     /* NM4: count this printed sample */
                    } else if (G >= R && G >= B) {
                        count    = 2;   // Green dominant
                        dominant = "GREEN";
                        dom_green_count++;
                    } else {
                        count    = 1;   // Blue dominant
                        dominant = "BLUE";
                        dom_blue_count++;
                    }
                }

                /* ---- NM3 / NM5: update stats ONLY for printed samples ---- */

                if (msg.temp_raw != 0) {
                    add_sample(&temp_stats, (double)tc);
                }
                if (msg.rh_raw != 0) {
                    add_sample(&rh_stats, (double)rh);
                }
                add_sample(&light_stats, (double)light_pct);
                add_sample(&soil_stats, (double)soil_pct);

                add_axis_sample(&ax_stats, (double)ax_g);
                add_axis_sample(&ay_stats, (double)ay_g);
                add_axis_sample(&az_stats, (double)az_g);

                /* ---- LED bus output ---- */
                bus_out_write(&my_bus_out, count);

                /* ---- Print this sample ---- */
                printk("SOIL MOISTURE: %.1f%%\n", (double)soil_pct);
                printk("LIGHT: %.2f%%\n", (double)light_pct);
                gps_print_from_sentence(msg.gps_sentence);
                printk("COLOR SENSOR: Clear: %d Red: %d Green: %d Blue: %d -- Dominant color: %s\n",
                       msg.clr_raw, msg.red_raw, msg.grn_raw, msg.blu_raw, dominant);
                printk("ACCELEROMETERS:\n\tX_axis: %.2f m/s²\n\tY_axis: %.2f m/s²\n\tZ_axis: %.2f m/s²\n",
                       (double)ax_g, (double)ay_g, (double)az_g);

                if (!temp_in_range || !rh_in_range) {
                    printk("WARNING: ENVIRONMENT OUT OF SPEC RANGE!\n");
                    if (!temp_in_range) {
                        printk("\tTemperature %.1f ºC is outside [-10, 50] ºC\n", (double)tc);
                    }
                    if (!rh_in_range) {
                        printk("\tRelative Humidity %.1f%% is outside [25, 75]%%\n", (double)rh);
                    }
                    printk("TEMP/HUM:\n\tTemperature: %.1f ºC\n\tRelative Humidity: %.1f%%\n",
                           (double)tc, (double)rh);
                } else {
                    printk("TEMP/HUM:\n\tTemperature: %.1f ºC\n\tRelative Humidity: %.1f%%\n",
                           (double)tc, (double)rh);
                }
            } // end if(sensor_thread_try_get)
        } // end if(read_ticker_event)

        // ------------------------------
        // Hourly statistics
        // ------------------------------
        if (hourly_stats_event) {
            hourly_stats_event = false;

            if (mode == MODE_NORMAL) {
                if (temp_stats.count == 0) {
                    printk("[NM3] Hourly statistics: no samples collected in last hour.\n");
                } else {
                    double mean_temp  = temp_stats.sum  / temp_stats.count;
                    double mean_rh    = rh_stats.sum    / rh_stats.count;
                    double mean_light = light_stats.sum / light_stats.count;
                    double mean_soil  = soil_stats.sum  / soil_stats.count;

                    printk("\n[NM3] Hourly statistics (N = %u samples)\n", temp_stats.count);
                    printk("  Temperature [ºC]: mean = %.2f  min = %.2f  max = %.2f\n",
                           mean_temp, temp_stats.min, temp_stats.max);
                    printk("  Rel. Humidity [%%]: mean = %.2f  min = %.2f  max = %.2f\n",
                           mean_rh, rh_stats.min, rh_stats.max);
                    printk("  Ambient Light [%%]: mean = %.2f  min = %.2f  max = %.2f\n",
                           mean_light, light_stats.min, light_stats.max);
                    printk("  Soil Moisture [%%]: mean = %.2f  min = %.2f  max = %.2f\n",
                           mean_soil, soil_stats.min, soil_stats.max);
                }

                /* NM4: hourly dominant colour */
                {
                    const char *hourly_dom = "NONE";
                    uint32_t max_c = 0;

                    if (dom_red_count > max_c) {
                        max_c = dom_red_count;
                        hourly_dom = "RED";
                    }
                    if (dom_green_count > max_c) {
                        max_c = dom_green_count;
                        hourly_dom = "GREEN";
                    }
                    if (dom_blue_count > max_c) {
                        max_c = dom_blue_count;
                        hourly_dom = "BLUE";
                    }

                    printk("[NM4] Hourly dominant leaf colour: %s (R=%u, G=%u, B=%u)\n",
                           hourly_dom, dom_red_count, dom_green_count, dom_blue_count);
                }

                /* NM5: hourly accelerometer stats */
                if (ax_stats.min == DBL_MAX) {
                    printk("[NM5] No accelerometer samples collected in last hour.\n");
                } else {
                    printk("[NM5] Hourly accelerometer min/max (m/s^2)\n");
                    printk("  X: min = %.2f  max = %.2f\n", ax_stats.min, ax_stats.max);
                    printk("  Y: min = %.2f  max = %.2f\n", ay_stats.min, ay_stats.max);
                    printk("  Z: min = %.2f  max = %.2f\n\n", az_stats.min, az_stats.max);
                }

                /* Prepare for next hour */
                reset_all_stats();
            }
        }

        k_msleep(10);
    }
}