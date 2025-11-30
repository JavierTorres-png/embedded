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

/* ---- Device address ---- */
#define TCS_ADDR           0x29
#define TCS_CMD_BIT        0x80  /* command bit for register access */

/* ---- Essential registers ---- */
#define TCS_ENABLE         0x00
#define TCS_EN_PON        0x01  /* power on        */
#define TCS_EN_AEN        0x02  /* ADC enable      */

#define TCS_ATIME          0x01  /* integration time */
#define TCS_CONTROL        0x0F  /* gain control     */

/* ---- 16-bit output registers (low,high) ---- */
#define TCS_CDATAL         0x14
#define TCS_CDATAH         0x15
#define TCS_RDATAL         0x16
#define TCS_RDATAH         0x17
#define TCS_GDATAL         0x18
#define TCS_GDATAH         0x19
#define TCS_BDATAL         0x1A
#define TCS_BDATAH         0x1B

/* ---- Basic config options ---- */
#define TCS_ATIME_154MS    0xC0  /* ~154 ms integration time */
#define TCS_GAIN_4X        0x01  /* 4x gain (good default)   */

bool finished = false;

static const struct gpio_dt_spec rgb_led = GPIO_DT_SPEC_GET(DT_ALIAS(rgbled), gpios);

static const struct i2c_dt_spec tcs = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = TCS_ADDR,
};

#define UART1_NODE DT_NODELABEL(usart1)
#define BUF_SIZE 128

static const struct device *uart_dev;
static char nmea_line[BUF_SIZE];
static uint8_t line_pos = 0;

static char latest_gga[BUF_SIZE];
static volatile bool latest_gga_valid = false;


static struct sensor_msg latest;
static struct k_mutex latest_mtx;

static struct k_mutex enable_mtx;
static struct k_mutex  time_mtx;
static struct k_condvar enable_cv;

static const struct gpio_dt_spec soilGpio = GPIO_DT_SPEC_GET(DT_ALIAS(soil), gpios);

void sensor_thread_measure()
{
    k_mutex_lock(&enable_mtx, K_FOREVER);
    k_condvar_signal(&enable_cv);
    k_mutex_unlock(&enable_mtx);
}

bool sensor_thread_try_get(struct sensor_msg *out)
{
    if (!out) return false;
    k_mutex_lock(&latest_mtx, K_FOREVER);
    if(!finished) {
        k_mutex_unlock(&latest_mtx);
        return false;
    }
    *out = latest;
    finished = false;
    k_mutex_unlock(&latest_mtx);
    return true;
}

static int16_t sample_buffer[BUFFER_SIZE];

static struct adc_channel_cfg channel_cfg_light = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 0,
};

int read_adc_raw (uint8_t channel_id, int16_t *raw_val)
{
    if (!device_is_ready(adc_dev)) {
        printk("Error: ADC device is not ready\n");
        return 0;
    }

    const struct adc_sequence sequence = {
        .channels = BIT(channel_id),
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


static struct adc_channel_cfg channel_cfg_soil = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 1,   // PA1 – Soil moisture
};

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

static inline int tcs_write8(uint8_t reg, uint8_t val)
{
    uint8_t w[2] = { (uint8_t)(TCS_CMD_BIT | reg), val };
    return i2c_write_dt(&tcs, w, sizeof(w));
}

static inline int tcs_read16(uint8_t reg_low, uint16_t *out)
{
    uint8_t r = (uint8_t)(TCS_CMD_BIT | reg_low);
    uint8_t b[2];
    int ret = i2c_write_read_dt(&tcs, &r, 1, b, 2);
    if (ret < 0) return ret;
    *out = (uint16_t)(b[0] | (b[1] << 8));
    return 0;
}

static inline void tcs_led_on(void)
{
    if (device_is_ready(rgb_led.port)) {
        gpio_pin_set_dt(&rgb_led, 1);
    }
}
static inline void tcs_led_off(void)
{
    if (device_is_ready(rgb_led.port)) {
        gpio_pin_set_dt(&rgb_led, 0);
    }
}

static int tcs_init(void)
{

    if (!device_is_ready(rgb_led.port)) {
        printk("TCS LED GPIO not ready\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&rgb_led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure TCS LED GPIO (%d)\n", ret);
        return ret;
    }

    if (!device_is_ready(tcs.bus)) return -ENODEV;

    /* Power on, then enable ADC */
    if (tcs_write8(TCS_ENABLE, TCS_EN_PON) < 0) return -EIO;
    k_msleep(3);
    if (tcs_write8(TCS_ENABLE, TCS_EN_PON | TCS_EN_AEN) < 0) return -EIO;

    /* Integration time + gain */
    if (tcs_write8(TCS_ATIME,   TCS_ATIME_154MS) < 0) return -EIO;
    if (tcs_write8(TCS_CONTROL, TCS_GAIN_4X)     < 0) return -EIO;

    /* Wait at least 1 integration period before first read */
    k_msleep(160);
    return 0;
}

static int tcs_read_crgb_led(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b)
{
    tcs_led_on();
    //k_msleep(10);  /* short settle time for LED */

    int ret_c = tcs_read16(TCS_CDATAL, c);
    int ret_r = tcs_read16(TCS_RDATAL, r);
    int ret_g = tcs_read16(TCS_GDATAL, g);
    int ret_b = tcs_read16(TCS_BDATAL, b);

    if (ret_c < 0 || ret_r < 0 || ret_g < 0 || ret_b < 0) {
        tcs_led_off();
        return -EIO;
    }

    //tcs_led_off();

    return 0;
}

static void gps_uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) == 1) {

            /* Start of a new NMEA sentence */
            if (c == '$') {
                line_pos = 0;
            }

            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;

                if (c == '\n') {
                    nmea_line[line_pos] = '\0';

                    /* Only keep if line has $GPGGA or $GNGGA*/
                    if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA")) {
                        size_t len = strlen(nmea_line);
                        if (len >= BUF_SIZE) {
                            len = BUF_SIZE - 1;
                        }
                        memcpy(latest_gga, nmea_line, len);
                        latest_gga[len] = '\0';
                        latest_gga_valid = true;
                    }

                    line_pos = 0;
                }
            }
        }
    }
}


static void sensor_entry(void *a, void *b, void *c)
{
    int8_t ret = adc_channel_setup(adc_dev, &channel_cfg_light);
    if (ret < 0) {
        printk("ADC channel setup failed: %d\n", ret);
        return;
    }
    
    uart_dev = DEVICE_DT_GET(UART1_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("UART not ready\n");
        return;
    }

    uart_irq_callback_set(uart_dev, gps_uart_isr);
    uart_irq_rx_enable(uart_dev);

    ret = adc_channel_setup(adc_dev, &channel_cfg_soil);
    if (ret < 0) {
        printk("ADC soil channel setup failed: %d\n", ret);
        return;
    }

    if (!device_is_ready(soilGpio.port)) {
        printk("Error: Soil GPIO device not ready\n");
        return;
    }

    if (gpio_pin_configure_dt(&soilGpio, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("Error: configuring Soil GPIO\n");
        return;
    }

    k_mutex_init(&latest_mtx);

    bool accel_ok = (mma_init(&accel_range) == 0);

    bool si_ok = (si7021_init() == 0);

    bool tcs_ok   = (tcs_init() == 0);

    while (1) {
        gpio_pin_set_dt(&soilGpio, 1);
        int16_t light_raw = 0;
        int16_t soil_raw  = 0;

        (void)read_adc_raw(0, &light_raw); // Channel 0 – LDR
        (void)read_adc_raw(1, &soil_raw);  // Channel 1 – Soil moisture
        gpio_pin_set_dt(&soilGpio, 0);

        int16_t ax = 0, ay = 0, az = 0;
        if (accel_ok) {
            (void)mma_read_xyz14(&ax, &ay, &az);
        }

        uint16_t rh_raw = 0, temp_raw = 0;
        if (si_ok) {
            if (si7021_read_rh_raw(&rh_raw) < 0) { rh_raw = 0; }
            if (si7021_read_temp_raw(&temp_raw) < 0) { temp_raw = 0; }
        }

        uint16_t c_raw = 0, r_raw = 0, g_raw = 0, b_raw = 0;
        if (tcs_ok) {
            (void)tcs_read_crgb_led(&c_raw, &r_raw, &g_raw, &b_raw);
        }

        k_mutex_lock(&latest_mtx, K_FOREVER);
        latest.light_raw   = light_raw;
        latest.ax_raw      = ax;
        latest.ay_raw      = ay;
        latest.az_raw      = az;
        latest.accel_range = accel_range;
        latest.rh_raw      = rh_raw;
        latest.temp_raw    = temp_raw;
        latest.clr_raw     = c_raw;
        latest.red_raw     = r_raw;
        latest.grn_raw     = g_raw;
        latest.blu_raw     = b_raw;
        latest.soil_raw    = soil_raw;

        if (latest_gga_valid) {
            strncpy(latest.gps_sentence, latest_gga, sizeof(latest.gps_sentence));
            latest.gps_sentence[sizeof(latest.gps_sentence) - 1] = '\0';
        } else {
            latest.gps_sentence[0] = '\0';
        }

        finished = true;
        k_mutex_unlock(&latest_mtx);

        k_mutex_lock(&enable_mtx, K_FOREVER);
        k_condvar_wait(&enable_cv, &enable_mtx, K_FOREVER);
        k_mutex_unlock(&enable_mtx);

    }
}

void sensor_thread_start(void)
{
    k_mutex_init(&time_mtx);
    k_mutex_init(&enable_mtx);
    k_condvar_init(&enable_cv);

    k_thread_create(&sensor_thr, sensor_stack, SENSOR_STACK_SZ,
                    sensor_entry, NULL, NULL, NULL,
                    SENSOR_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thr, "sensor_thr");
}