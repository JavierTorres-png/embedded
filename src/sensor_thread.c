#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include "sensor_thread.h"

#define BUFFER_SIZE 1
#define SENSOR_STACK_SZ  1024
#define SENSOR_PRIO      5

K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_STACK_SZ);
static struct k_thread sensor_thr;

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

#define REG_OUT_X_MSB     0x01
#define REG_WHO_AM_I      0x0D
#define REG_XYZ_DATA_CFG  0x0E
#define REG_CTRL_REG1     0x2A

#define FS_2G             0x00
#define FS_4G             0x01
#define FS_8G             0x02

#define CTRL_REG1_ACTIVE  (1 << 0)
#define WHO_AM_I_ID       0x1A

uint8_t accel_range = FS_2G;

static const struct i2c_dt_spec mma = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = 0x1D,
};

#define SI7021_ADDR          0x40
#define SI7021_CMD_MEAS_RH   0xF5  /* Measure Relative Humidity, No-Hold Master */
#define SI7021_CMD_MEAS_TEMP 0xF3  /* Measure Temperature, No-Hold Master */
#define SI7021_CMD_RESET     0xFE
#define SI7021_T_CONV_MS     20    /* conservative wait (datasheet ~12-14 ms max) */

static const struct i2c_dt_spec si7021 = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = SI7021_ADDR,
};

#define UART1_NODE DT_NODELABEL(usart1)
#define BUF_SIZE 128

static const struct device *uart_dev;
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;

static struct sensor_msg latest;
static struct k_mutex latest_mtx;

static bool enabled = false;
static struct k_mutex  enable_mtx;
static struct k_condvar enable_cv;

void sensor_thread_set_enabled(bool en)
{
    k_mutex_lock(&enable_mtx, K_FOREVER);
    bool was = enabled;
    enabled = en;
    if (en && !was) {
        k_condvar_signal(&enable_cv);
    }
    k_mutex_unlock(&enable_mtx);
}

bool sensor_thread_try_get(struct sensor_msg *out)
{
    if (!out) return false;
    k_mutex_lock(&latest_mtx, K_FOREVER);
    *out = latest;
    k_mutex_unlock(&latest_mtx);
    return true;
}

static int16_t sample_buffer[BUFFER_SIZE];

static struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 0,
};

int read_adc_raw (int16_t *raw_val)
{
    if (!device_is_ready(adc_dev)) {
        printk("Error: ADC device is not ready\n");
        return 0;
    }

    const struct adc_sequence sequence = {
        .channels = BIT(0),
        .buffer = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = 12,
    };

    int8_t ret = adc_read(adc_dev, &sequence);
    if (ret < 0) {
        printk("Error: ADC read_dt failed: %d\n", ret);
        return ret;
    }

    *raw_val = sample_buffer[0];
    return 0;
}

static inline int mma_read(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_write_read_dt(&mma, &reg, 1, buf, len);
}
static inline int mma_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg, val};
    return i2c_write_dt(&mma, tx, sizeof(tx));
}
static int mma_read_xyz14(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t b[6];
    int r = mma_read(REG_OUT_X_MSB, b, sizeof(b));
    if (r < 0) return r;
    *x = (int16_t)((uint16_t)b[0] << 8 | b[1]);
    *y = (int16_t)((uint16_t)b[2] << 8 | b[3]);
    *z = (int16_t)((uint16_t)b[4] << 8 | b[5]);
    *x = *x  >> 2;
    *y = *y  >> 2;
    *z = *z  >> 2;
    
    return 0;
}
static int mma_init(uint8_t *range_out) {
    if (!device_is_ready(mma.bus)) return -ENODEV;

    uint8_t id = 0;
    if (mma_read(REG_WHO_AM_I, &id, 1) < 0 || id != WHO_AM_I_ID) return -EIO;

    uint8_t c1 = 0;
    if (mma_read(REG_CTRL_REG1, &c1, 1) < 0) return -EIO;

    /* Standby to configure */
    c1 &= ~CTRL_REG1_ACTIVE;
    if (mma_write(REG_CTRL_REG1, c1) < 0) return -EIO;

    if (mma_write(REG_XYZ_DATA_CFG, accel_range) < 0) return -EIO;

    /* Activate */
    c1 |= CTRL_REG1_ACTIVE;
    if (mma_write(REG_CTRL_REG1, c1) < 0) return -EIO;

    *range_out = accel_range;
    return 0;
}

static inline int si7021_cmd(uint8_t cmd)
{
    return i2c_write_dt(&si7021, &cmd, 1);
}

static int si7021_read16(uint16_t *out)
{
    /* Read exactly 2 bytes (ignore CRC) */
    uint8_t buf[2] = {0};
    int ret = i2c_read_dt(&si7021, buf, 2);
    if (ret < 0) return ret;
    *out = (uint16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

static int si7021_soft_reset(void)
{
    int ret = si7021_cmd(SI7021_CMD_RESET);
    if (ret < 0) return ret;
    k_msleep(20); /* allow reset time */
    return 0;
}

static int si7021_init(void)
{
    if (!device_is_ready(si7021.bus)) {
        printk("Si7021: I2C bus not ready\n");
        return -ENODEV;
    }
    return si7021_soft_reset();
}

static int si7021_read_rh_raw(uint16_t *raw)
{
    int ret = si7021_cmd(SI7021_CMD_MEAS_RH);
    if (ret < 0) return ret;
    k_msleep(SI7021_T_CONV_MS);
    return si7021_read16(raw);
}

static int si7021_read_temp_raw(uint16_t *raw)
{
    int ret = si7021_cmd(SI7021_CMD_MEAS_TEMP);
    if (ret < 0) return ret;
    k_msleep(SI7021_T_CONV_MS);
    return si7021_read16(raw);
}

// ------------------------------- START GPS CODE

// Convertir NMEA (DDMM.MMMM) en degrés décimaux
static float nmea_to_degrees(const char *nmea, char dir)
{
    if (!nmea || strlen(nmea) < 4) return 0.0f;
    
    float value = 0.0f;
    int degrees = 0;
    float minutes = 0.0f;
    
    // Convertir la chaîne en nombre
    for (int i = 0; nmea[i]; i++) {
        if (nmea[i] >= '0' && nmea[i] <= '9') {
            value = value * 10 + (nmea[i] - '0');
        } else if (nmea[i] == '.') {
            // Lire les décimales
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
    
    // Séparer degrés et minutes
    if (dir == 'N' || dir == 'S') {
        degrees = (int)(value / 100);
        minutes = value - (degrees * 100);
    } else {
        degrees = (int)(value / 100);
        minutes = value - (degrees * 100);
    }
    
    float result = degrees + (minutes / 60.0f);
    
    // Négatif si Sud ou Ouest
    if (dir == 'S' || dir == 'W') {
        result = -result;
    }
    
    return result;
}

// Fonction simple pour afficher les infos importantes
static void print_gps_info(char *line)
{
    char *p = line;
    int field = 0;
    char *fields[15] = {0};
    
    // Découper la ligne en champs
    while (*p && field < 15) {
        if (*p == ',') {
            *p = '\0';
            fields[field++] = line;
            line = p + 1;
        }
        p++;
    }
    
    // Afficher : Heure | Position en degrés | Altitude | Satellites | HDOP
    if (fields[1] && fields[2] && fields[4] && fields[9]) {
        float lat = nmea_to_degrees(fields[2], fields[3][0]);
        float lon = nmea_to_degrees(fields[4], fields[5][0]);
        
        printk("%c%c:%c%c:%c%c | %.6f° %c, %.6f° %c | Alt: %s m | Sats: %s\n",
               fields[1][0], fields[1][1], fields[1][2], 
               fields[1][3], fields[1][4], fields[1][5],
               lat >= 0 ? lat : -lat, fields[3][0],  // Latitude
               lon >= 0 ? lon : -lon, fields[5][0],  // Longitude
               fields[9],             // Altitude
               fields[7]);             // Satellites
    }
}

static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;
    
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {
            if (c == '$') {
                line_pos = 0;
            }
            
            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;
                
                if (c == '\n') {
                    nmea_line[line_pos] = '\0';
                    
                    // Affichage direct des trames GPGGA
                    if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA")) {
                        print_gps_info(nmea_line);
                    }
                    
                    line_pos = 0;
                }
            }
        }
    }
}

// ------------------------------- END GPS FUNCTIONS

static void sensor_entry(void *a, void *b, void *c                                                                                                                                                                                                                                                                                                                                                                                                )
{
    int8_t ret = adc_channel_setup(adc_dev, &channel_cfg);
    if (ret < 0) {
        printk("ADC channel setup failed: %d\n", ret);
        return;
    }
    
    uart_dev = DEVICE_DT_GET(UART1_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("UART not ready\n");
        return -1;
    }

    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);

    k_mutex_init(&latest_mtx);

    bool accel_ok = (mma_init(&accel_range) == 0);

    bool si_ok = (si7021_init() == 0);

    while (1) {
        k_mutex_lock(&enable_mtx, K_FOREVER);
        while (!enabled) {
            k_condvar_wait(&enable_cv, &enable_mtx, K_FOREVER);
        }
        k_mutex_unlock(&enable_mtx);
        
        int16_t light_raw = 0;
        (void)read_adc_raw(&light_raw);

        int16_t ax = 0, ay = 0, az = 0;
        if (accel_ok) {
            (void)mma_read_xyz14(&ax, &ay, &az);
        }

        uint16_t rh_raw = 0, temp_raw = 0;
        if (si_ok) {
            if (si7021_read_rh_raw(&rh_raw) < 0) { rh_raw = 0; }
            if (si7021_read_temp_raw(&temp_raw) < 0) { temp_raw = 0; }
        }

        k_mutex_lock(&latest_mtx, K_FOREVER);
        latest.light_raw   = light_raw;
        latest.ax_raw      = ax;
        latest.ay_raw      = ay;
        latest.az_raw      = az;
        latest.accel_range = accel_range;
        latest.rh_raw      = rh_raw;
        latest.temp_raw    = temp_raw;
        k_mutex_unlock(&latest_mtx);

        k_msleep(2000);
    }
}

void sensor_thread_start(void)
{
    k_mutex_init(&enable_mtx);
    k_condvar_init(&enable_cv);
    enabled = false;

    k_thread_create(&sensor_thr, sensor_stack, SENSOR_STACK_SZ,
                    sensor_entry, NULL, NULL, NULL,
                    SENSOR_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thr, "sensor_thr");
}
