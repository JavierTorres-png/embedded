#ifndef SENSOR_THREAD_H
#define SENSOR_THREAD_H

/* Latest sensor sample published by the sensor thread */
struct sensor_msg {
    int16_t raw;   /* 0..4095 */
    float   pct;   /* 0..100 */
};

/* Start/stop + read of latest value */
void sensor_thread_start(void);
void sensor_thread_set_enabled(bool en);
bool sensor_thread_try_get(struct sensor_msg *out);

#endif /* SENSOR_THREAD_H */
