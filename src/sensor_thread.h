#ifndef SENSOR_THREAD_H
#define SENSOR_THREAD_H

#define GPS_SENTENCE_MAX 128 

/* Latest sensor sample published by the sensor thread */
struct sensor_msg {
    int16_t light_raw;         /* 0..4095 */
    int16_t ax_raw, ay_raw, az_raw;  /* signed 14-bit in int16 */
    uint8_t accel_range;             /* 0: ±2g, 1: ±4g, 2: ±8g */
    uint16_t rh_raw;     /* raw humidity */
    uint16_t temp_raw;   /* raw temperature */
    uint16_t clr_raw;                   /* Clear */
    uint16_t red_raw;
    uint16_t grn_raw;
    uint16_t blu_raw;
    int16_t soil_raw;
    char gps_sentence[GPS_SENTENCE_MAX];
};

/* Start/stop + read of latest value */
void sensor_thread_start(void);
void sensor_thread_measure();
bool sensor_thread_try_get(struct sensor_msg *out);

extern bool finished;

#endif /* SENSOR_THREAD_H */
