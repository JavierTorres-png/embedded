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

#define GPS_SENTENCE_MAX_LEN GPS_SENTENCE_MAX

/* --- NMEA (GGA) helpers for GPS printing --- */
static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4) return 0.0f;

    float value = 0.0f;
    for (int i = 0; nmea[i]; i++) {
        if (nmea[i] >= '0' && nmea[i] <= '9') {
            value = value * 10.0f + (nmea[i] - '0');
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

    int degrees = (int)(value / 100.0f);
    float minutes = value - (degrees * 100.0f);
    float result = degrees + (minutes / 60.0f);

    if (dir == 'S' || dir == 'W') result = -result;
    return result;
}

static void gps_print_from_sentence(const char *sentence)
{
    if (!sentence || sentence[0] == '\0') {
        printk("GPS: no data\n");
        return;
    }

    char line[GPS_SENTENCE_MAX_LEN];
    strncpy(line, sentence, sizeof(line));
    line[sizeof(line) - 1] = '\0';

    char *p = line;
    int field = 0;
    char *fields[15] = { 0 };

    fields[field++] = p;
    while (*p && field < (int)sizeof(fields)/sizeof(fields[0])) {
        if (*p == ',') {
            *p = '\0';
            fields[field++] = p + 1;
        }
        p++;
    }

    /* GGA fields: 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W, 7=sat, 9=alt */
    if (fields[1] && fields[2] && fields[3] && fields[4] && fields[5] && fields[9]) {
        float lat = nmea_to_degrees(fields[2], fields[3][0]);
        float lon = nmea_to_degrees(fields[4], fields[5][0]);

        int hour = (fields[1][0] - '0') * 10 + (fields[1][1] - '0');
        int min  = (fields[1][2] - '0') * 10 + (fields[1][3] - '0');
        int sec  = (fields[1][4] - '0') * 10 + (fields[1][5] - '0');

        /* timezone adjustment used previously (+1) */
        hour = (hour + 1) % 24;

        printk("GPS:\n");
        printk("\tTime: %02d:%02d:%02d\n", hour, min, sec);
        printk("\tLat: %.6f° %c\n", (lat >= 0 ? lat : -lat), fields[3][0]);
        printk("\tLon: %.6f° %c\n", (lon >= 0 ? lon : -lon), fields[5][0]);
        printk("\tAlt: %s m\n", fields[9]);
        if (fields[7]) printk("\tSatellites: %s\n", fields[7]);
    } else {
        printk("GPS: incomplete GGA sentence\n");
    }
}

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

        bool did_work = false;
        int mode = init_get_mode();

        /* Timer asked to measure: request sensor acquisition */
        if (init_consume_measure()) {
            sensor_thread_measure();
            did_work = true;
        }

        /* Button released -> toggle mode */
        if (init_consume_button_released()) {
            init_toggle_mode();
            did_work = true;
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
                did_work = true;
            }
        }

        /* Hourly aggregated stats */
        if (init_consume_hourly()) {
            if (init_get_mode() == MODE_NORMAL) {
                print_hourly_stats();
            }
            did_work = true;
        }

        /* ECO sleep: only when in ADVANCED mode we sleep indefinitely if idle.
         * In other modes keep polling with 10 ms to preserve responsiveness/behaviour.
         */
        if (!did_work) {
            if (mode == MODE_ADVANCED) {
                /* Enter deep sleep until an interrupt (timer/GPIO) occurs */
                k_sleep(K_FOREVER);
            } else {
                /* keep responsive in TEST/NORMAL */
                k_msleep(10);
            }
        }
    }

    return 0;
}
