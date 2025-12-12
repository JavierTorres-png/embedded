/* project_config.h */
#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <float.h>
#include <stdint.h>

/* Use float everywhere for statistics to reduce footprint on embedded targets */
typedef float stat_scalar_t;

/* sleep times (seconds) */
#define MODE_TEST_SLEEP_TIME_SEC   2U
#define MODE_NORMAL_SLEEP_TIME_SEC 30U
#define MODE_ADVANCED_SLEEP_TIME_SEC 30U

/* hourly stats (hours) */
#define HOURLY_STATS_TIME_HOUR 1U

/* Modes */
#define MODE_TEST   1
#define MODE_NORMAL 2
#define MODE_ADVANCED 3

/* Bus */
#define BUS_SIZE 3

/* Light sensitivity multiplier */
#define LIGHT_SENSITIVITY 2.0f

/* Sensor/limits */
#define TEMP_MIN_C      (-10.0f) //Parameters to receive alert
#define TEMP_MAX_C      50.0f
#define RH_MIN_PCT      25.0f
#define RH_MAX_PCT      75.0f
#define LIGHT_MIN_PCT   20.0f
#define LIGHT_MAX_PCT   90.0f
#define SOIL_MIN_PCT    10.0f
#define SOIL_MAX_PCT    20.0f
#define COLOR_CLEAR_MIN 1u

/* accelerometer limit (m/s^2) */
#define ACC_ABS_MAX     15.0f

/* LED patterns */
enum {
    LED_PATTERN_OFF = 0x0,
    LED_PATTERN_TEMP_ERR = 0x4,
    LED_PATTERN_RH_ERR   = 0x1,
    LED_PATTERN_LIGHT_ERR= 0x2,
    LED_PATTERN_SOIL_ERR = 0x6,
    LED_PATTERN_COLOR_ERR= 0x5,
    LED_PATTERN_ACCEL_ERR= 0x3
};

#endif /* PROJECT_CONFIG_H */
