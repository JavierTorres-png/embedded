#ifndef SENSOR_THREAD_H
#define SENSOR_THREAD_H

#define GPS_SENTENCE_MAX 128 

/* Latest sensor sample published by the sensor thread */
struct sensor_msg {
    int16_t light_raw;                  /* 0..4095 */
    int16_t ax_raw, ay_raw, az_raw;     /* Signed 14-bit in int16 */
    uint8_t accel_range;                /* 0: ±2g, 1: ±4g, 2: ±8g */
    uint16_t rh_raw;                    /* Raw humidity */
    uint16_t temp_raw;                  /* Raw temperature */
    uint16_t clr_raw;                   /* Clear */
    uint16_t red_raw;                   /* Raw red */
    uint16_t grn_raw;                   /* Raw green */
    uint16_t blu_raw;                   /* Raw blue */
    int16_t soil_raw;                   /* Raw soil*/
    char gps_sentence[GPS_SENTENCE_MAX];/* Raw GPS measure*/
};

/* Start/stop + read of latest value */
void sensor_thread_start(void);                     // Start thread (for the first and only time)
void sensor_thread_measure();                       // Wake up sensor_thread
bool sensor_thread_try_get(struct sensor_msg *out); // Getter for sensor_msg struct

#endif /* SENSOR_THREAD_H */
