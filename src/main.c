/* Fixed main.c — updated for NCS / Zephyr mcumgr callback API + bug fixes */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <math.h>

#include <zephyr/logging/log.h>
#define LOG_LEVEL LOG_LEVEL_INF
LOG_MODULE_REGISTER(main);

/* mcumgr / smp includes (Zephyr/NCS) */
/* MCUmgr / mgmt includes */
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>   /* mgmt_cb typedef & mgmt_callback_register() */
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>        /* mgmt helper APIs (if needed) */
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h> /* smp_bt_register() */

#include "imu_service.h"

/* --- Sensor and Application Logic --- */
/* NOTE: previous IMU_FRAME_SIZE (20) was too small for seq(2)+ts(8)+6*2 accel+6*2 gyro = 22 bytes.
 * Use 24 for alignment and safety.
 */
#define IMU_FRAME_SIZE 24
#define BATCH_SIZE 10
#define PRE_TRIGGER_SAMPLES 26    // 1 s at 26 Hz
#define POST_TRIGGER_SAMPLES 2000 // 4 s at 500 Hz
#define BURST_DURATION_MS 20000   // 20 s
#define BURST_MAX_MS 60000        // 60 s
#define COOLDOWN_MS 10000         // 10 s
#define DAILY_BUDGET_S 1200       // 20 min
#define STEP_THRESHOLD_G 1.5      // Accel magnitude threshold
#define STEP_MIN_MS 285           // 210 spm (3.5 Hz)
#define STEP_MAX_MS 1250          // 48 spm (0.8 Hz)
#define STEP_COUNT_TRIGGER 8

static uint8_t batch_buffer[BATCH_SIZE * IMU_FRAME_SIZE];
static int batch_index = 0;
static uint16_t seq_id = 0;
static uint8_t pre_trigger_buffer[PRE_TRIGGER_SAMPLES * IMU_FRAME_SIZE];
static struct ring_buf pre_trigger_rb;
static uint32_t total_burst_ms = 0;
static int64_t last_step_time = 0;
static int step_count = 0;
static int64_t burst_start_time = 0;
static int64_t cooldown_start_time = 0;
static int post_trigger_samples_left = 0;

/* --- Devicetree Aliases --- */
static const struct device *const imu_dev = DEVICE_DT_GET(DT_INST(0, st_ism330dhcx));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/* --- Advertising Data --- */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab)), // Motion Service
    // BT_DATA_BYTES(BT_DATA_UUID128_ALL,
    //               BT_UUID_128_ENCODE(0x8d53dc1d, 0x1db7, 0x4cd3, 0x868b, 0x8a9514604e95)), // SMP Service
};

/* --- Application State --- */
enum app_state
{
    APP_STATE_IDLE,
    APP_STATE_SESSION,
    APP_STATE_ALLDAY,
    APP_STATE_BURST,
    APP_STATE_COOLDOWN,
    APP_STATE_DFU,
};
static volatile enum app_state current_state = APP_STATE_IDLE;

/* --- Button Handling --- */
static struct gpio_callback button_cb_data;
static int64_t button_press_time = 0;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int64_t now = k_uptime_get();
    if (button_press_time == 0)
    {
        button_press_time = now;
        return;
    }

    int64_t duration_ms = now - button_press_time;
    button_press_time = 0;

    if (duration_ms >= 8000) // 8 s for DFU or reset
    {
        current_state = APP_STATE_DFU;
        gpio_pin_set_dt(&led, (k_uptime_get() % 200) < 100 ? 1 : 0); // Fast blink
        LOG_INF("State -> DFU (8s press)");
    }
    else if (duration_ms >= 3000) // 3 s for All-Day Mode toggle
    {
        if (current_state == APP_STATE_IDLE)
        {
            current_state = APP_STATE_ALLDAY;
            gpio_pin_set_dt(&led, 1); // Slow pulse in main loop
            LOG_INF("State -> ALLDAY (Low-power sampling)");
        }
        else if (current_state == APP_STATE_ALLDAY || current_state == APP_STATE_BURST || current_state == APP_STATE_COOLDOWN)
        {
            current_state = APP_STATE_IDLE;
            gpio_pin_set_dt(&led, 0);
            LOG_INF("State -> IDLE (Sampling OFF)");
            if (batch_index > 0)
            {
                imu_service_notify_motion(batch_buffer, batch_index * IMU_FRAME_SIZE);
                batch_index = 0;
            }
        }
    }
    else // Short press for Session Mode toggle
    {
        if (current_state == APP_STATE_IDLE || current_state == APP_STATE_ALLDAY)
        {
            current_state = APP_STATE_SESSION;
            gpio_pin_set_dt(&led, 1);
            LOG_INF("State -> SESSION (Streaming ON)");
        }
        else if (current_state == APP_STATE_SESSION)
        {
            current_state = APP_STATE_IDLE;
            gpio_pin_set_dt(&led, 0);
            LOG_INF("State -> IDLE (Streaming OFF)");
            if (batch_index > 0)
            {
                imu_service_notify_motion(batch_buffer, batch_index * IMU_FRAME_SIZE);
                batch_index = 0;
            }
        }
    }
}

static int button_init(void)
{
    if (!device_is_ready(button.port))
    {
        LOG_ERR("Button device not ready");
        return -ENODEV;
    }
    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret)
        return ret;

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    if (ret)
        return ret;

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    LOG_INF("Button initialized");
    return 0;
}

static int16_t scale_accel_to_int16(const struct sensor_value *val)
{
    double scaled_val = sensor_value_to_double(val) * 1671.0;
    if (scaled_val > INT16_MAX)
        return INT16_MAX;
    if (scaled_val < INT16_MIN)
        return INT16_MIN;
    return (int16_t)scaled_val;
}

static int16_t scale_gyro_to_int16(const struct sensor_value *val)
{
    double scaled_val = sensor_value_to_double(val) * 7500.0;
    if (scaled_val > INT16_MAX)
        return INT16_MAX;
    if (scaled_val < INT16_MIN)
        return INT16_MIN;
    return (int16_t)scaled_val;
}

/* --- MCUmgr DFU Callback using modern mgmt callback API --- */
/* Callback signature required by Zephyr's mgmt subsystem. */
static enum mgmt_cb_return mcumgr_event_cb(uint32_t event,
                                           enum mgmt_cb_return prev_status,
                                           int32_t *rc,
                                           uint16_t *group,
                                           bool *abort_more,
                                           void *data,
                                           size_t data_size)
{
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);

    switch (event)
    {
    case MGMT_EVT_OP_IMG_MGMT_DFU_STARTED:
        current_state = APP_STATE_DFU;
        LOG_INF("DFU started");
        break;
    case MGMT_EVT_OP_IMG_MGMT_DFU_PENDING:
        LOG_INF("DFU upload finished (pending confirm)");
        break;
    case MGMT_EVT_OP_IMG_MGMT_DFU_CONFIRMED:
        current_state = APP_STATE_IDLE;
        gpio_pin_set_dt(&led, 0);
        LOG_INF("DFU confirmed, state -> IDLE");
        break;
    default:
        break;
    }

    return MGMT_CB_OK;
}

/* --- IMU read and processing --- */
static void read_and_process_imu(bool high_freq)
{
    uint8_t frame[IMU_FRAME_SIZE];
    memset(frame, 0, sizeof(frame));
    struct sensor_value accel[3], gyro[3];

    if (sensor_sample_fetch(imu_dev) < 0)
    {
        LOG_ERR("Sensor sample fetch failed");
        return;
    }

    if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel) < 0)
    {
        LOG_ERR("Accel channel read failed");
        return;
    }
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro) < 0)
    {
        LOG_ERR("Gyro channel read failed");
        return;
    }

    uint16_t current_seq_id = seq_id++;
    /* timestamp in microseconds */
    uint64_t timestamp_us = (uint64_t)k_uptime_get() * 1000ULL;

    memcpy(&frame[0], &current_seq_id, sizeof(current_seq_id)); /* 2 bytes */
    memcpy(&frame[2], &timestamp_us, sizeof(timestamp_us));     /* 8 bytes -> covers frame[2..9] */

    int16_t ax = scale_accel_to_int16(&accel[0]);
    int16_t ay = scale_accel_to_int16(&accel[1]);
    int16_t az = scale_accel_to_int16(&accel[2]);
    int16_t gx = scale_gyro_to_int16(&gyro[0]);
    int16_t gy = scale_gyro_to_int16(&gyro[1]);
    int16_t gz = scale_gyro_to_int16(&gyro[2]);

    /* offsets chosen so that fields don't overlap (seq(0..1), ts(2..9), ax@10..11, ay@12..13, az@14..15, gx@16..17, gy@18..19, gz@20..21) */
    memcpy(&frame[10], &ax, sizeof(ax));
    memcpy(&frame[12], &ay, sizeof(ay));
    memcpy(&frame[14], &az, sizeof(az));
    memcpy(&frame[16], &gx, sizeof(gx));
    memcpy(&frame[18], &gy, sizeof(gy));
    memcpy(&frame[20], &gz, sizeof(gz));

    if (high_freq)
    {
        memcpy(&batch_buffer[batch_index * IMU_FRAME_SIZE], frame, IMU_FRAME_SIZE);
        batch_index++;
        if (batch_index == BATCH_SIZE)
        {
            imu_service_notify_motion(batch_buffer, BATCH_SIZE * IMU_FRAME_SIZE);
            batch_index = 0;
        }
    }
    else
    {
        /* write into pre-trigger ring buffer (bytes) */
        ring_buf_put(&pre_trigger_rb, frame, IMU_FRAME_SIZE);
        imu_service_notify_motion(frame, IMU_FRAME_SIZE);
    }

    if (current_state == APP_STATE_ALLDAY)
    {
        double ax_g = sensor_value_to_double(&accel[0]);
        double ay_g = sensor_value_to_double(&accel[1]);
        double az_g = sensor_value_to_double(&accel[2]);
        double mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

        int64_t now = k_uptime_get();
        int64_t delta_ms = now - last_step_time;

        if (mag > STEP_THRESHOLD_G && delta_ms >= STEP_MIN_MS && delta_ms <= STEP_MAX_MS)
        {
            step_count++;
            last_step_time = now;
            LOG_INF("Step detected, count: %d", step_count);
            if (step_count >= STEP_COUNT_TRIGGER && total_burst_ms < DAILY_BUDGET_S * 1000)
            {
                current_state = APP_STATE_BURST;
                burst_start_time = now;
                uint32_t bytes = ring_buf_get(&pre_trigger_rb, batch_buffer, PRE_TRIGGER_SAMPLES * IMU_FRAME_SIZE);
                batch_index = bytes / IMU_FRAME_SIZE;
                LOG_INF("State -> BURST (Steps detected) pretrigger samples: %d", batch_index);
            }
        }
        else if (delta_ms > STEP_MAX_MS)
        {
            step_count = 0;
        }
    }
}

static int sensor_imu_init(void)
{
    if (!device_is_ready(imu_dev))
    {
        LOG_ERR("IMU not ready");
        return -ENODEV;
    }
    struct sensor_value odr_attr = {.val1 = 26, .val2 = 0};
    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0)
    {
        LOG_ERR("Failed to set accel ODR");
        return -EIO;
    }
    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0)
    {
        LOG_ERR("Failed to set gyro ODR");
        return -EIO;
    }
    LOG_INF("IMU sensor configured for 26 Hz");
    return 0;
}

static int set_imu_odr(int freq_hz)
{
    struct sensor_value odr_attr = {.val1 = freq_hz, .val2 = 0};
    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0)
    {
        LOG_ERR("Failed to set accel ODR to %d Hz", freq_hz);
        return -EIO;
    }
    if (sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0)
    {
        LOG_ERR("Failed to set gyro ODR to %d Hz", freq_hz);
        return -EIO;
    }
    LOG_INF("IMU ODR set to %d Hz", freq_hz);
    return 0;
}

void main(void)
{
    int err;
    LOG_INF("IMU BLE Auto-Burst Example Starting...");

    if (!device_is_ready(led.port))
    {
        LOG_ERR("LED device not ready");
        return;
    }
    err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        LOG_ERR("Failed to configure LED");
        return;
    }

    err = button_init();
    if (err)
    {
        LOG_ERR("Failed to initialize button");
        return;
    }

    err = sensor_imu_init();
    if (err)
    {
        LOG_ERR("Failed to initialize sensor");
        return;
    }

    /* Initialize pre-trigger ring buffer (byte oriented) */
    ring_buf_init(&pre_trigger_rb, sizeof(pre_trigger_buffer), pre_trigger_buffer);

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    imu_service_init();

    /* Register MCUmgr callbacks for image/DFU notifications */
    static struct mgmt_callback dfu_cb;
    dfu_cb.callback = mcumgr_event_cb;
    dfu_cb.event_id = (MGMT_EVT_OP_IMG_MGMT_DFU_STARTED |
                       MGMT_EVT_OP_IMG_MGMT_DFU_PENDING |
                       MGMT_EVT_OP_IMG_MGMT_DFU_CONFIRMED);
    mgmt_callback_register(&dfu_cb);

    /* Start advertising */
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising started. Short press for session, 3s press for all-day mode, 8s press for DFU.");

    int64_t last_budget_reset = k_uptime_get();
    while (1)
    {
        if (current_state == APP_STATE_IDLE && imu_service_is_start_commanded())
        {
            current_state = APP_STATE_SESSION;
            gpio_pin_set_dt(&led, 1);
            LOG_INF("State -> SESSION (Host start command)");
        }

        if (current_state == APP_STATE_SESSION)
        {
            set_imu_odr(520);
            read_and_process_imu(true);
            k_sleep(K_MSEC(2));
            gpio_pin_set_dt(&led, 1);
        }
        else if (current_state == APP_STATE_ALLDAY)
        {
            set_imu_odr(26);
            read_and_process_imu(false);
            k_sleep(K_MSEC(38));
            gpio_pin_set_dt(&led, (k_uptime_get() % 1000) < 500 ? 1 : 0);
        }
        else if (current_state == APP_STATE_BURST)
        {
            set_imu_odr(520);
            read_and_process_imu(true);
            int64_t now = k_uptime_get();
            if (step_count == 0 && post_trigger_samples_left == 0)
            {
                post_trigger_samples_left = POST_TRIGGER_SAMPLES;
            }
            if (post_trigger_samples_left > 0)
            {
                post_trigger_samples_left--;
                if (post_trigger_samples_left == 0)
                {
                    current_state = APP_STATE_COOLDOWN;
                    cooldown_start_time = now;
                    LOG_INF("State -> COOLDOWN (Post-trigger complete)");
                }
            }
            else if (now - burst_start_time >= BURST_MAX_MS || total_burst_ms >= DAILY_BUDGET_S * 1000)
            {
                current_state = APP_STATE_COOLDOWN;
                cooldown_start_time = now;
                LOG_INF("State -> COOLDOWN (Burst timeout or budget exceeded)");
            }
            total_burst_ms += 2;
            k_sleep(K_MSEC(2));
            gpio_pin_set_dt(&led, (k_uptime_get() % 200) < 100 ? 1 : 0);
        }
        else if (current_state == APP_STATE_COOLDOWN)
        {
            if (batch_index > 0)
            {
                imu_service_notify_motion(batch_buffer, batch_index * IMU_FRAME_SIZE);
                batch_index = 0;
            }
            int64_t now = k_uptime_get();
            if (now - cooldown_start_time >= COOLDOWN_MS)
            {
                current_state = APP_STATE_ALLDAY;
                step_count = 0;
                LOG_INF("State -> ALLDAY (Cooldown complete)");
            }
            k_sleep(K_MSEC(38));
            gpio_pin_set_dt(&led, 0);
        }
        else if (current_state == APP_STATE_DFU)
        {
            k_sleep(K_MSEC(100));
            gpio_pin_set_dt(&led, (k_uptime_get() % 200) < 100 ? 1 : 0);
        }
        else
        {
            k_sleep(K_MSEC(100));
            gpio_pin_set_dt(&led, 0);
        }

        if (k_uptime_get() - last_budget_reset >= 24 * 60 * 60 * 1000)
        {
            total_burst_ms = 0;
            last_budget_reset = k_uptime_get();
            LOG_INF("Daily burst budget reset");
        }
    }
}
