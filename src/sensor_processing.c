/* src/sensor_processing.c
 *
 * Implements sensor processing, statistics aggregation and hourly printing.
 * Uses float for reduced footprint on embedded targets.
 */

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>
#include <string.h>

#include "sensor_processing.h"
#include "project_config.h"

/* ---- internal stats types ---- */
typedef float stat_scalar_t_local; // local float type for aggregations

typedef struct {
    stat_scalar_t_local sum;    // accumulate sum
    stat_scalar_t_local min;    // minimum seen
    stat_scalar_t_local max;    // maximum seen
    uint32_t count;             // sample count
} stats_t;

static stats_t temp_stats;  // temperature aggregator
static stats_t rh_stats;
static stats_t light_stats;
static stats_t soil_stats;

static uint32_t dom_red_count   = 0;
static uint32_t dom_green_count = 0;
static uint32_t dom_blue_count  = 0;

typedef struct { stat_scalar_t_local min, max; } axis_stats_t; // accel axis min/max
static axis_stats_t ax_stats;
static axis_stats_t ay_stats;
static axis_stats_t az_stats;

/* Initialize/reset helper */
void reset_all_stats(void)
{
    temp_stats.sum = 0.0f;
    temp_stats.min =  FLT_MAX; // set min to max sentinel
    temp_stats.max = -FLT_MAX;
    temp_stats.count = 0;

    rh_stats.sum = 0.0f;
    rh_stats.min =  FLT_MAX;
    rh_stats.max = -FLT_MAX;
    rh_stats.count = 0;

    light_stats.sum = 0.0f;
    light_stats.min =  FLT_MAX;
    light_stats.max = -FLT_MAX;
    light_stats.count = 0;

    soil_stats.sum = 0.0f;
    soil_stats.min =  FLT_MAX;
    soil_stats.max = -FLT_MAX;
    soil_stats.count = 0;

    dom_red_count = dom_green_count = dom_blue_count = 0; // reset color counters

    ax_stats.min = ay_stats.min = az_stats.min = FLT_MAX;   // reset accel stats
    ax_stats.max = ay_stats.max = az_stats.max = -FLT_MAX;
}

/* local helpers */
static inline void stats_add(stats_t *s, stat_scalar_t_local v)
{
    if (v < s->min) s->min = v; // update min
    if (v > s->max) s->max = v; // update max
    s->sum += v;                // add to sum
    s->count++;                 // increment count
}

static inline void axis_add(axis_stats_t *a, stat_scalar_t_local v)
{
    if (v < a->min) a->min = v;
    if (v > a->max) a->max = v;
}

static inline float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Public: process one sensor message and return LED pattern */
uint8_t process_sensor_sample(const struct sensor_msg *msg, int mode)
{
    if (!msg) return LED_PATTERN_OFF;

    /* accel sensitivity selection */
    float sens_g = 4096.0f;
    if (msg->accel_range == 1) sens_g = 2048.0f;
    else if (msg->accel_range == 2) sens_g = 1024.0f;

    float ax_g = 9.807f * (float)msg->ax_raw / sens_g;
    float ay_g = 9.807f * (float)msg->ay_raw / sens_g;
    float az_g = 9.807f * (float)msg->az_raw / sens_g;

    /* color processing */
    uint8_t count = 0;
    const char *dominant = "NONE";
    uint8_t R = 0, G = 0, B = 0;

    if (msg->clr_raw > 0) {
        /* scale to 0..255 per component */
        uint32_t r_scaled = (uint32_t)msg->red_raw * 255u;
        uint32_t g_scaled = (uint32_t)msg->grn_raw * 255u;
        uint32_t b_scaled = (uint32_t)msg->blu_raw * 255u;

        R = (uint8_t)(r_scaled / (uint32_t)msg->clr_raw);
        G = (uint8_t)(g_scaled / (uint32_t)msg->clr_raw);
        B = (uint8_t)(b_scaled / (uint32_t)msg->clr_raw);

        if (R >= G && R >= B) { count = 4; dominant = "RED"; if (mode == MODE_NORMAL) dom_red_count++; }
        else if (G >= R && G >= B) { count = 2; dominant = "GREEN"; if (mode == MODE_NORMAL) dom_green_count++; }
        else { count = 1; dominant = "BLUE"; if (mode == MODE_NORMAL) dom_blue_count++; }
    }

    /* soil percentage */
    float soil_pct = (msg->soil_raw * 100.0f) / 4095.0f;
    soil_pct = clampf_local(soil_pct, 0.0f, 100.0f);

    /* ambient light percentage (scale and clamp) */
    float scaled_raw = (float)msg->light_raw * LIGHT_SENSITIVITY;
    float light_pct;
    if (scaled_raw <= 0.0f) light_pct = 0.0f;
    else if (scaled_raw >= 4095.0f) light_pct = 100.0f;
    else light_pct = (scaled_raw * 100.0f) / 4095.0f;

    /* humidity & temperature conversions */
    float rh = 0.0f, tc = 0.0f;
    if (msg->rh_raw != 0) {
        rh = (125.0f * (float)msg->rh_raw / 65536.0f) - 6.0f;
        rh = clampf_local(rh, 0.0f, 100.0f);
    }
    if (msg->temp_raw != 0) {
        tc = (175.72f * (float)msg->temp_raw / 65536.0f) - 46.85f;
    }

    /* Update stats only in normal mode */
    if (mode == MODE_NORMAL) {
        if (msg->temp_raw != 0) stats_add(&temp_stats, tc);
        if (msg->rh_raw   != 0) stats_add(&rh_stats, rh);
        stats_add(&light_stats, light_pct);
        stats_add(&soil_stats, soil_pct);
        axis_add(&ax_stats, ax_g);
        axis_add(&ay_stats, ay_g);
        axis_add(&az_stats, az_g);
    }

    /* check ranges */
    bool temp_in_range = (tc > TEMP_MIN_C) && (tc < TEMP_MAX_C);
    bool rh_in_range   = (rh > RH_MIN_PCT) && (rh < RH_MAX_PCT);
    bool light_in_range = (light_pct > LIGHT_MIN_PCT && light_pct < LIGHT_MAX_PCT);
    bool soil_in_range  = (soil_pct > SOIL_MIN_PCT && soil_pct < SOIL_MAX_PCT);
    bool color_ok = (msg->clr_raw > COLOR_CLEAR_MIN);
    bool accel_in_range = (ax_g < ACC_ABS_MAX && ax_g > -ACC_ABS_MAX &&
                           ay_g < ACC_ABS_MAX && ay_g > -ACC_ABS_MAX &&
                           az_g < ACC_ABS_MAX && az_g > -ACC_ABS_MAX);

    uint8_t led_pattern = LED_PATTERN_OFF;
    if (mode == MODE_TEST) {
        led_pattern = count; /* show color index in test mode */
    } else {
        if (!temp_in_range) led_pattern = LED_PATTERN_TEMP_ERR;
        else if (!rh_in_range) led_pattern = LED_PATTERN_RH_ERR;
        else if (!light_in_range) led_pattern = LED_PATTERN_LIGHT_ERR;
        else if (!soil_in_range) led_pattern = LED_PATTERN_SOIL_ERR;
        else if (!color_ok) led_pattern = LED_PATTERN_COLOR_ERR;
        else if (!accel_in_range) led_pattern = LED_PATTERN_ACCEL_ERR;
        else led_pattern = LED_PATTERN_OFF;
    }

    /* Print per-sample summary (kept concise) */
    printk("SAMPLE: soil=%.1f%% light=%.1f%% temp=%.1fC rh=%.1f%% clr=%u R=%u G=%u B=%u dom=%s\n",
           (double)soil_pct, (double)light_pct, (double)tc, (double)rh,
           msg->clr_raw, R, G, B, dominant);
    printk("ACCEL (m/s^2): X=%.2f Y=%.2f Z=%.2f\n", (double)ax_g, (double)ay_g, (double)az_g);

    if (!temp_in_range || !rh_in_range) {
        printk("ALERT: environmental parameter out of range (T: %.2f C, RH: %.2f%%)\n", (double)tc, (double)rh);
    }

    return led_pattern;
}

/* Public: compute and print hourly aggregated statistics, then reset stats */
void print_hourly_stats(void)
{
    if (temp_stats.count == 0) {
        printk("[STATS] No samples available for last hour.\n");
    } else {
        float mean_temp  = temp_stats.sum  / (float)temp_stats.count;
        float mean_rh    = rh_stats.sum    / (float)rh_stats.count;
        float mean_light = light_stats.sum / (float)light_stats.count;
        float mean_soil  = soil_stats.sum  / (float)soil_stats.count;

        printk("\n[STATS] Hourly summary - Samples: %u\n", temp_stats.count);
        printk("  Temperature [C]: mean=%.2f min=%.2f max=%.2f\n",
               (double)mean_temp, (double)temp_stats.min, (double)temp_stats.max);
        printk("  Relative Humidity [%%]: mean=%.2f min=%.2f max=%.2f\n",
               (double)mean_rh, (double)rh_stats.min, (double)rh_stats.max);
        printk("  Ambient Light [%%]: mean=%.2f min=%.2f max=%.2f\n",
               (double)mean_light, (double)light_stats.min, (double)light_stats.max);
        printk("  Soil Moisture [%%]: mean=%.2f min=%.2f max=%.2f\n",
               (double)mean_soil, (double)soil_stats.min, (double)soil_stats.max);
    }

    /* Dominant color */
    const char *hourly_dom = "NONE";
    uint32_t max_c = 0;
    if (dom_red_count > max_c) { max_c = dom_red_count; hourly_dom = "RED"; }
    if (dom_green_count > max_c) { max_c = dom_green_count; hourly_dom = "GREEN"; }
    if (dom_blue_count > max_c) { max_c = dom_blue_count; hourly_dom = "BLUE"; }

    printk("[STATS] Hourly dominant color: %s (R=%u G=%u B=%u)\n",
           hourly_dom, dom_red_count, dom_green_count, dom_blue_count);

    /* Accelerometer summary */
    if (ax_stats.min == FLT_MAX) {
        printk("[STATS] No accelerometer samples collected in last hour.\n");
    } else {
        printk("[STATS] Accelerometer min/max (m/s^2):\n");
        printk("  X: min=%.2f max=%.2f\n", (double)ax_stats.min, (double)ax_stats.max);
        printk("  Y: min=%.2f max=%.2f\n", (double)ay_stats.min, (double)ay_stats.max);
        printk("  Z: min=%.2f max=%.2f\n\n", (double)az_stats.min, (double)az_stats.max);
    }

    /* Reset for next period */
    reset_all_stats();
}
