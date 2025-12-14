/* src/main.c
 *
 * Minimal top-level orchestrator.
 * Uses the init facade (init.c / init.h) to initialize hardware and provide event flags.
 * Delegates sensor processing to sensor_processing.c and acquisition to sensor_thread.c.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include "init.h"                /* init_system(), init_consume_*, bus_out_write(), init_get_mode() */
#include "sensor_thread.h"       /* struct sensor_msg, sensor_thread_try_get(), sensor_thread_measure() */
#include "sensor_processing.h"   /* process_sensor_sample(), print_hourly_stats() */
#include "project_config.h"      /* MODE_TEST / MODE_NORMAL, constants */

/* --- main --- */
int main(void)
{
    printk("\nPlant monitor starting...\n");

    if (init_system() != 0) {
        printk("Initialization failed\n");
        return 0;
    }

    struct bus_out *bus = init_get_default_bus();
    if (!bus) {
        printk("No bus available from init\n");
        return 0;
    }

    /* Main event loop: poll event flags provided by init facade */
    while (1) {
        
        int mode = init_get_mode();

        /* Timer asked to measure: request sensor acquisition */
        if (init_consume_measure()) {
            sensor_thread_measure();
        }

        /* Button released -> toggle mode */
        if (init_consume_button_released()) {
            init_toggle_mode();
            mode = init_get_mode();
        }

        /* Read event: when ready, get sample and process it */
        if (init_read()) {
            struct sensor_msg msg;
            if (sensor_thread_try_get(&msg)) {
                consume_read();
                uint8_t pattern = process_sensor_sample(&msg, init_get_mode());
                (void)bus_out_write(bus, pattern);
                gps_print_from_sentence(msg.gps_sentence);
                if(msg.tcs_triggered && (mode == MODE_ADVANCED)) {
                    init_set_test_mode();
                }
            }
        }

        /* Hourly aggregated stats */
        if (init_consume_hourly()) {
            if (init_get_mode() == MODE_NORMAL) {
                print_hourly_stats();
            }
        }

        k_msleep(50);
        
    }

    return 0;
}
