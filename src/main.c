#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/ring_buffer.h>
#include <math.h>
#include "imu_service.h"

#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/* --- Sensor and Application Logic --- */
#define IMU_FRAME_SIZE 20
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
                  BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab)),
};

/* --- Application State --- */
enum app_state
{
    APP_STATE_IDLE,
    APP_STATE_SESSION,
    APP_STATE_ALLDAY,
    APP_STATE_BURST,
    APP_STATE_COOLDOWN,
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

    if (duration_ms >= 3000) // 3 s for All-Day Mode toggle
    {
        if (current_state == APP_STATE_IDLE)
        {
            current_state = APP_STATE_ALLDAY;
            gpio_pin_set_dt(&led, 1); // Slow pulse handled in main loop
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
    return CLAMP(scaled_val, INT16_MIN, INT16_MAX);
}

static int16_t scale_gyro_to_int16(const struct sensor_value *val)
{
    double scaled_val = sensor_value_to_double(val) * 7500.0;
    return CLAMP(scaled_val, INT16_MIN, INT16_MAX);
}

static void read_and_process_imu(bool high_freq)
{
    uint8_t frame[IMU_FRAME_SIZE] = {0};
    struct sensor_value accel[3], gyro[3];

    if (sensor_sample_fetch(imu_dev) < 0)
    {
        LOG_ERR("Sensor sample fetch failed");
        return;
    }

    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);

    uint16_t current_seq_id = seq_id++;
    uint64_t timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
    memcpy(&frame[0], &current_seq_id, sizeof(current_seq_id));
    memcpy(&frame[2], &timestamp_us, sizeof(timestamp_us));

    int16_t ax = scale_accel_to_int16(&accel[0]);
    int16_t ay = scale_accel_to_int16(&accel[1]);
    int16_t az = scale_accel_to_int16(&accel[2]);
    int16_t gx = scale_gyro_to_int16(&gyro[0]);
    int16_t gy = scale_gyro_to_int16(&gyro[1]);
    int16_t gz = scale_gyro_to_int16(&gyro[2]);

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
        ring_buf_put(&pre_trigger_rb, frame, IMU_FRAME_SIZE);
        if (ring_buf_space_get(&pre_trigger_rb) == 0)
        {
            uint8_t temp[IMU_FRAME_SIZE];
            ring_buf_get(&pre_trigger_rb, temp, IMU_FRAME_SIZE);
        }
        imu_service_notify_motion(frame, IMU_FRAME_SIZE); // Single sample for low freq
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
                // Copy pre-trigger buffer to batch
                uint32_t bytes = ring_buf_get(&pre_trigger_rb, batch_buffer, PRE_TRIGGER_SAMPLES * IMU_FRAME_SIZE);
                batch_index = bytes / IMU_FRAME_SIZE;
                LOG_INF("State -> BURST (Steps detected)");
            }
        }
        else if (delta_ms > STEP_MAX_MS)
        {
            step_count = 0; // Reset if too long between steps
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
    struct sensor_value odr_attr = {.val1 = 26, .val2 = 0}; // Default to 26 Hz
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

    ring_buf_init(&pre_trigger_rb, PRE_TRIGGER_SAMPLES * IMU_FRAME_SIZE, pre_trigger_buffer);

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    imu_service_init();

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising started. Short press for session, 3s press for all-day mode, or write to Control char.");

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
            k_sleep(K_MSEC(2));       // 500 Hz
            gpio_pin_set_dt(&led, 1); // Solid for session
        }
        else if (current_state == APP_STATE_ALLDAY)
        {
            set_imu_odr(26);
            read_and_process_imu(false);
            k_sleep(K_MSEC(38)); // ~26 Hz
            // Slow pulse for ALLDAY
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
            k_sleep(K_MSEC(2)); // 500 Hz
            // Fast blink for BURST
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
            k_sleep(K_MSEC(38));      // ~26 Hz
            gpio_pin_set_dt(&led, 0); // Off for cooldown
        }
        else
        {
            k_sleep(K_MSEC(100)); // Idle
            gpio_pin_set_dt(&led, 0);
        }

        // Reset daily budget every 24 hours
        if (k_uptime_get() - last_budget_reset >= 24 * 60 * 60 * 1000)
        {
            total_burst_ms = 0;
            last_budget_reset = k_uptime_get();
            LOG_INF("Daily burst budget reset");
        }
    }
}