/* src/init.h - single init facade (header)
 *
 * Provides system initialization and simple event-consume APIs used by main.
 * Note: 'struct bus_out' is an opaque forward declaration here.
 */

#ifndef INIT_H
#define INIT_H

#include <stdbool.h>
#include <stdint.h>
#include "project_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* opaque bus_out type (concrete layout is defined in init.c) */
struct bus_out;

/* System init / mode control */
int init_system(void);
int init_toggle_mode(void);
void init_set_test_mode(void);
void init_set_normal_mode(void);
int init_get_mode(void);

/* Access to default bus_out (opaque pointer) */
struct bus_out *init_get_default_bus(void);

/* Event consumers (polling-style) */
bool init_consume_measure(void);
bool init_consume_read(void);
bool init_consume_hourly(void);
bool init_consume_button_released(void);

/* Bus write helper (writes LSB->pin0). Implemented in init.c */
int bus_out_write(struct bus_out *bus, int8_t value);

#ifdef __cplusplus
}
#endif

#endif /* INIT_H */
