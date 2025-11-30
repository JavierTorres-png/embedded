/* src/sensor_processing.h */
#ifndef SENSOR_PROCESSING_H
#define SENSOR_PROCESSING_H

#include <stdint.h>
#include "sensor_thread.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Process one sensor_msg and return the LED pattern to display.
 * mode: MODE_TEST or MODE_NORMAL.
 */
uint8_t process_sensor_sample(const struct sensor_msg *msg, int mode);

/* Print + reset hourly statistics (call when hourly event fires). */
void print_hourly_stats(void);

/* Reset internal stats (used when switching to normal and at startup). */
void reset_all_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_PROCESSING_H */
