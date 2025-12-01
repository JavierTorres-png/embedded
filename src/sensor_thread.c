#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include "sensor_thread.h"

#define BUFFER_SIZE 1  /* ADC buffer: one sample */   
#define SENSOR_STACK_SZ  1024 /* thread stack size */
#define SENSOR_PRIO      5  /* thread priority */ 

K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_STACK_SZ); /* allocate stack memory */ 
static struct k_thread sensor_thr; /* Zephyr thread control block */
static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1)); /* ADC device */ 

#define REG_OUT_X_MSB     0x01  /* first output register for X MSB */
#define REG_WHO_AM_I      0x0D  /* device ID register */
#define REG_XYZ_DATA_CFG  0x0E  /* full-scale range config */
#define REG_CTRL_REG1     0x2A  /* control register (ACTIVE bit) */

#define FS_2G             0x00  
#define FS_4G             0x01
#define FS_8G             0x02

#define CTRL_REG1_ACTIVE  (1 << 0) /* ACTIVE bit mask for CTRL_REG1 */
#define WHO_AM_I_ID       0x1A  /* expected WHO_AM_I value */

uint8_t accel_range = FS_2G;

static const struct i2c_dt_spec mma = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = 0x1D,
};

#define SI7021_ADDR          0x40
#define SI7021_CMD_MEAS_RH   0xF5  /* request RH (no-hold) */
#define SI7021_CMD_MEAS_TEMP 0xF3  /* request temperature (no-hold) */
#define SI7021_CMD_RESET     0xFE  /* soft reset */
#define SI7021_T_CONV_MS     20    /* conservative wait (datasheet ~12-14 ms max) */

static const struct i2c_dt_spec si7021 = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = SI7021_ADDR,
};

/* ---- Device address ---- */
#define TCS_ADDR           0x29
#define TCS_CMD_BIT        0x80  /* command bit for TCS operations */

/* ---- Essential registers ---- */
#define TCS_ENABLE         0x00 /* ENABLE register */
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
#define TCS_ATIME_154MS    0xC0  /* ~154 ms integration time to measure RGB */
#define TCS_GAIN_4X        0x01  /* 4x ADC gain */

/* threshold registers */
#define TCS_AILTL   0x04U /* clear low threshold low byte */
#define TCS_AILTH   0x05U /* clear low threshold high byte */
#define TCS_AIHTL   0x06U /* clear high threshold low byte */
#define TCS_AIHTH   0x07U /* clear high threshold high byte */
#define TCS_PERS    0x0CU /* interrupt persistence register */

/* ENABLE/AIEN bit */
#define TCS_ENABLE_AIEN (1 << 4) /* AIEN bit in ENABLE to enable interrupts */
/* special clear interrupt command */
#define TCS_CLEAR_INT_CMD 0x66U

#define TCS_FIXED_LOW   300U    /* low threshold chosen by app */
#define TCS_FIXED_HIGH  30000U   /* high threshold chosen by app */

int sensor_set_clear_thresholds(uint16_t low, uint16_t high);
int sensor_enable_interrupts(bool enable);
int sensor_clear_interrupt(void);

bool finished = false;

static const struct gpio_dt_spec rgb_led = GPIO_DT_SPEC_GET(DT_ALIAS(rgbled), gpios);
static const struct gpio_dt_spec tcswake_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(tcswake), gpios);
static const struct gpio_dt_spec soilGpio = GPIO_DT_SPEC_GET(DT_ALIAS(soil), gpios);
static const struct gpio_dt_spec lightGpio = GPIO_DT_SPEC_GET(DT_ALIAS(light), gpios);
static struct gpio_callback tcswake_cb_data;
static atomic_t tcswake_flag = ATOMIC_INIT(0);

/* i2c spec for tcs */
static const struct i2c_dt_spec tcs = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = TCS_ADDR,
};

#define UART1_NODE DT_NODELABEL(usart1)
#define BUF_SIZE 128 /* NMEA Stanrd buffer size */

static const struct device *uart_dev; /* UART device pointer */
static char nmea_line[BUF_SIZE];  /* UART device pointer */
static uint8_t line_pos = 0;  /* position in NMEA buffer */

static char latest_gga[BUF_SIZE];  /* store latest GGA GPS Fix Data sentence */
static volatile bool latest_gga_valid = false;


static struct sensor_msg latest;
static struct k_mutex latest_mtx;

static struct k_mutex enable_mtx;
static struct k_mutex  time_mtx;
static struct k_condvar enable_cv;

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
        return -ENODEV;
    }

    const struct adc_sequence sequence = {
        .channels = BIT(channel_id),
        .buffer = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = 12,
    };

    int ret = adc_read(adc_dev, &sequence);
    if (ret < 0) {
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
    return i2c_write_read_dt(&mma, &reg, 1, buf, len); /* write reg, then read len bytes */
}
static inline int mma_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg, val};
    return i2c_write_dt(&mma, tx, sizeof(tx)); /* write reg,value */
}
static int mma_read_xyz14(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t b[6];
    int r = mma_read(REG_OUT_X_MSB, b, sizeof(b)); /* read X/Y/Z MSB/LSB */
    if (r < 0) return r;
    *x = (int16_t)((uint16_t)b[0] << 8 | b[1]); /* combine X MSB/LSB */
    *y = (int16_t)((uint16_t)b[2] << 8 | b[3]);
    *z = (int16_t)((uint16_t)b[4] << 8 | b[5]);
    *x = *x  >> 2;
    *y = *y  >> 2;
    *z = *z  >> 2;
    
    return 0;
}
static int mma_init(uint8_t *range_out) {
    if (!device_is_ready(mma.bus)) return -ENODEV; /* check I2C master */

    uint8_t id = 0;
    if (mma_read(REG_WHO_AM_I, &id, 1) < 0 || id != WHO_AM_I_ID) return -EIO; /* verify ID */

    uint8_t c1 = 0;
    if (mma_read(REG_CTRL_REG1, &c1, 1) < 0) return -EIO; /* read CTRL_REG1 */

    /* Standby to configure */
    c1 &= ~CTRL_REG1_ACTIVE; /* clear ACTIVE bit to enter standby */
    if (mma_write(REG_CTRL_REG1, c1) < 0) return -EIO; /* write CTRL_REG1 */

    if (mma_write(REG_XYZ_DATA_CFG, accel_range) < 0) return -EIO; /* set full-scale range */

    /* Activate */
    c1 |= CTRL_REG1_ACTIVE;  /* set ACTIVE bit to resume measurements */
    if (mma_write(REG_CTRL_REG1, c1) < 0) return -EIO; /* write CTRL_REG1 back after*/

    *range_out = accel_range;  /* return range */
    return 0;
}

static inline int si7021_cmd(uint8_t cmd)
{
    return i2c_write_dt(&si7021, &cmd, 1); /* send single-byte command */
}

static int si7021_read16(uint16_t *out)
{
    /* Read exactly 2 bytes (ignore CRC) */
    uint8_t buf[2] = {0};
    int ret = i2c_read_dt(&si7021, buf, 2); /* read 2 bytes MSB,LSB */
    if (ret < 0) return ret;
    *out = (uint16_t)((buf[0] << 8) | buf[1]); /* combine */
    return 0;
}

static int si7021_soft_reset(void)
{
    int ret = si7021_cmd(SI7021_CMD_RESET); /* issue reset */
    if (ret < 0) return ret;
    k_msleep(20); /* allow reset time */
    return 0;
}

static int si7021_init(void)
{
    if (!device_is_ready(si7021.bus)) {
        return -ENODEV;
    }
    return si7021_soft_reset();
}

static int si7021_read_rh_raw(uint16_t *raw)
{
    int ret = si7021_cmd(SI7021_CMD_MEAS_RH); /* request RH measurement */
    if (ret < 0) return ret;
    k_msleep(SI7021_T_CONV_MS); /* wait for conversion */
    return si7021_read16(raw);  /* read 2-byte result */
}

static int si7021_read_temp_raw(uint16_t *raw)
{
    int ret = si7021_cmd(SI7021_CMD_MEAS_TEMP);  /* request temp measurement */
    if (ret < 0) return ret;
    k_msleep(SI7021_T_CONV_MS);
    return si7021_read16(raw);
}

static inline int tcs_write8(uint8_t reg, uint8_t val)
{
    uint8_t w[2] = { (uint8_t)(TCS_CMD_BIT | reg), val };  /* command byte + value */
    return i2c_write_dt(&tcs, w, sizeof(w));  /* write register */
}

static inline int tcs_read16(uint8_t reg_low, uint16_t *out)
{
    uint8_t r = (uint8_t)(TCS_CMD_BIT | reg_low);  /* command to read starting reg_low */
    uint8_t b[2];
    int ret = i2c_write_read_dt(&tcs, &r, 1, b, 2);  /* read low,high */
    if (ret < 0) return ret;
    *out = (uint16_t)(b[0] | (b[1] << 8));  /* combine low|high */
    return 0;
}

static inline void tcs_led_on(void)
{
    if (device_is_ready(rgb_led.port)) {
        gpio_pin_set_dt(&rgb_led, 1); /* turn LED on */
    }
}
static inline void tcs_led_off(void)
{
    if (device_is_ready(rgb_led.port)) {
        gpio_pin_set_dt(&rgb_led, 0);
    }
}

static void tcswake_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* Signal that the INT pin fired; sensor thread will clear the latch by I2C. */
    atomic_set(&tcswake_flag, 1); /* ISR: set flag to indicate INT fired */
}

void sensor_interrupt_init(void)
{
    int rc;

    /* If no wake GPIO defined or device not ready, skip GPIO setup but keep thresholds. */
    if (!tcswake_gpio.port || !device_is_ready(tcswake_gpio.port)) {
        /* no gpio available — still program thresholds */
    } else {
        /* configure wake GPIO input */
        rc = gpio_pin_configure_dt(&tcswake_gpio, GPIO_INPUT | GPIO_PULL_UP);
        if (rc) {
            /* continue anyway — thresholds still useful */
        } else {
            (void)gpio_pin_interrupt_configure_dt(&tcswake_gpio, GPIO_INT_EDGE_FALLING); /* enable edge */

            /* install minimal callback (ISR only sets atomic flag) */
            gpio_init_callback(&tcswake_cb_data, tcswake_gpio_cb, BIT(tcswake_gpio.pin)); /* set callback */
            (void)gpio_add_callback(tcswake_gpio.port, &tcswake_cb_data); /* add callback */
        }
    }

    (void)sensor_set_clear_thresholds(TCS_FIXED_LOW, TCS_FIXED_HIGH); /* program thresholds */

    (void)sensor_clear_interrupt(); /* clear any pending INT */
    k_msleep(5);

    /* enable AIEN in ENABLE register (PON/AEN must already be set in tcs_init) */
    (void)sensor_enable_interrupts(true);
}

static int tcs_init(void)
{
    if (!device_is_ready(rgb_led.port)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&rgb_led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    if (!device_is_ready(tcs.bus)) return -ENODEV;

    tcs_led_on();

    /* Power on, then enable ADC */
    if (tcs_write8(TCS_ENABLE, TCS_EN_PON) < 0) return -EIO;  /* write PON */
    k_msleep(3);
    if (tcs_write8(TCS_ENABLE, TCS_EN_PON | TCS_EN_AEN) < 0) return -EIO;  /* write PON|AEN */

    /* Integration time + gain */
    if (tcs_write8(TCS_ATIME,   TCS_ATIME_154MS) < 0) return -EIO;  /* set integration time */
    if (tcs_write8(TCS_CONTROL, TCS_GAIN_4X)     < 0) return -EIO;  /* set gain */

    /* Wait at least 1 integration period before first read */
    k_msleep(160);


    sensor_interrupt_init();

    return 0;
}

static int tcs_read_crgb_led(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b)
{
    int rc;

    //tcs_led_on();
    k_msleep(10);  /* short settle time for LED */

    rc = tcs_read16(TCS_CDATAL, c);
    if (rc < 0) { return rc; }
    rc = tcs_read16(TCS_RDATAL, r);
    if (rc < 0) { return rc; }
    rc = tcs_read16(TCS_GDATAL, g);
    if (rc < 0) { return rc; }
    rc = tcs_read16(TCS_BDATAL, b);
    if (rc < 0) { return rc; }

    //tcs_led_off();
    return 0;
}

static int tcs_read8(uint8_t reg, uint8_t *out)
{
    uint8_t cmd = (uint8_t)(TCS_CMD_BIT | reg); /* command to read one byte */
    if (!device_is_ready(tcs.bus)) {
        return -ENODEV;
    }
    return i2c_write_read_dt(&tcs, &cmd, 1, out, 1); /* read single byte */
} 

static int tcs_write_16(uint8_t reg_low, uint16_t value)
{
    int rc;
    rc = tcs_write8(reg_low, (uint8_t)(value & 0xFF)); /* write low byte */
    if (rc) return rc;
    rc = tcs_write8(reg_low + 1, (uint8_t)((value >> 8) & 0xFF)); /* write high byte */
    return rc;
}

static int tcs_write_cmd_byte(uint8_t code)
{
    uint8_t buf = (uint8_t)(TCS_CMD_BIT | code);  /* special command (e.g., clear INT) */
    if (!device_is_ready(tcs.bus)) {
        return -ENODEV;
    }
    return i2c_write_dt(&tcs, &buf, 1);  /* send command */
}

/* Set clear-channel thresholds (low/high). low <= high normally.
 * Returns 0 on success.
 */
int sensor_set_clear_thresholds(uint16_t low, uint16_t high)
{
    int rc;

    rc = tcs_write_16(TCS_AILTL, low);  /* set low threshold (low+high bytes) */
    if (rc) {
        return rc;
    }

    rc = tcs_write_16(TCS_AIHTL, high);  /* set high threshold (low+high bytes) */
    if (rc) {
        return rc;
    }

    /* default persistence: 1 (one consecutive out-of-range cycle) */
    rc = tcs_write8(TCS_PERS, 0x01);
    if (rc) {
        return rc;
    }

    return 0;
}

/* Enable or disable RGBC interrupt via ENABLE.AIEN bit */
int sensor_enable_interrupts(bool enable)
{
    int rc;
    uint8_t en;

    rc = tcs_read8(TCS_ENABLE, &en);  /* read ENABLE reg */
    if (rc) {
        return rc;
    }

    if (enable) {
        en |= TCS_ENABLE_AIEN;  /* set AIEN to enable interrupts */
    } else {
        en &= (uint8_t)(~TCS_ENABLE_AIEN);  /* clear AIEN */
    }

    rc = tcs_write8(TCS_ENABLE, en);  /* write ENABLE reg back */
    return rc;
}

/* Clear sensor interrupt latch using special command (thread context only) */
int sensor_clear_interrupt(void)
{
    int rc = tcs_write_cmd_byte(TCS_CLEAR_INT_CMD);  /* send clear-int command */
    if (rc == 0) {
        /* small delay to let INT pin release */
        k_msleep(2);
    }
    return rc;
}

static void gps_uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {  /* read while data available */
        if (uart_fifo_read(dev, &c, 1) == 1) {

            /* Start of a new NMEA sentence */
            if (c == '$') {
                line_pos = 0;
            }

            if (line_pos < BUF_SIZE - 1) {
                nmea_line[line_pos++] = c;  /* store char */
                if (c == '\n') {  /* end of sentence */
                    nmea_line[line_pos] = '\0';

                    /* keep only GGA sentences */
                    if (strstr(nmea_line, "$GPGGA") || strstr(nmea_line, "$GNGGA")) {
                        size_t len = strlen(nmea_line);
                        if (len >= BUF_SIZE) {
                            len = BUF_SIZE - 1;  /* clamp */
                        }
                        memcpy(latest_gga, nmea_line, len);  /* copy */
                        latest_gga[len] = '\0';
                        latest_gga_valid = true;  /* mark valid */
                    }

                    line_pos = 0;  /* reset for next sentence */
                }
            }
        }
    }
}


static void sensor_entry(void *a, void *b, void *c)
{
    int8_t ret = adc_channel_setup(adc_dev, &channel_cfg_light);
    if (ret < 0) {
        return;
    }
    
    uart_dev = DEVICE_DT_GET(UART1_NODE);
    if (!device_is_ready(uart_dev)) {
        return;
    }

    uart_irq_callback_set(uart_dev, gps_uart_isr);  /* attach UART ISR */
    uart_irq_rx_enable(uart_dev);  /* enable RX interrupts */

    ret = adc_channel_setup(adc_dev, &channel_cfg_soil);
    if (ret < 0) {
        return;
    }

    if (!device_is_ready(soilGpio.port)) {
        return;
    }

    if (gpio_pin_configure_dt(&soilGpio, GPIO_OUTPUT_INACTIVE) < 0) {
        return;
    }

    if (!device_is_ready(lightGpio.port)) {
        return;
    }

    if (gpio_pin_configure_dt(&lightGpio, GPIO_OUTPUT_INACTIVE) < 0) {
        return;
    }

    k_mutex_init(&latest_mtx);

    bool accel_ok = (mma_init(&accel_range) == 0);

    bool si_ok = (si7021_init() == 0);

    bool tcs_ok   = (tcs_init() == 0);

    while (1) {
        int16_t light_raw = 0;
        int16_t soil_raw  = 0;

        gpio_pin_set_dt(&soilGpio, 1);
        gpio_pin_set_dt(&lightGpio, 1);

        (void)read_adc_raw(0, &light_raw); // Channel 0 – LDR
        (void)read_adc_raw(1, &soil_raw);  // Channel 1 – Soil moisture

        gpio_pin_set_dt(&soilGpio, 0);
        gpio_pin_set_dt(&lightGpio, 0);

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
        bool tcs_trig = false;

        if (tcs_ok) {
            (void)tcs_read_crgb_led(&c_raw, &r_raw, &g_raw, &b_raw);

            /*    If ISR indicated INT happened, consume it and mark trigger.
             *    ISR only sets atomic flag; here we clear the sensor latch via I2C.
             */
            if (atomic_cas(&tcswake_flag, 1, 0)) {
                tcs_trig = true;
                (void)sensor_clear_interrupt();
            }

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

        if (latest_gga_valid) {  /* copy GPS sentence if valid */
            strncpy(latest.gps_sentence, latest_gga, sizeof(latest.gps_sentence));
            latest.gps_sentence[sizeof(latest.gps_sentence) - 1] = '\0';
        } else {
            latest.gps_sentence[0] = '\0';
        }

        /* store the TCS trigger boolean in the message */
        latest.tcs_triggered = tcs_trig;

        finished = true;
        k_mutex_unlock(&latest_mtx);

        k_mutex_lock(&enable_mtx, K_FOREVER); /* wait for next measure signal */
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
