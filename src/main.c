#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "imu_service.h"

#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

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
// We only need two states now: IDLE and SESSION (streaming)
enum app_state
{
    APP_STATE_IDLE,
    APP_STATE_SESSION,
};
static volatile enum app_state current_state = APP_STATE_IDLE;

/* --- Button Handling --- */
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Toggle the state on each button press
    if (current_state == APP_STATE_IDLE)
    {
        current_state = APP_STATE_SESSION;
        gpio_pin_set_dt(&led, 1); // Turn LED ON
        LOG_INF("State -> SESSION (Streaming ON)");
    }
    else
    {
        current_state = APP_STATE_IDLE;
        gpio_pin_set_dt(&led, 0); // Turn LED OFF
        LOG_INF("State -> IDLE (Streaming OFF)");
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

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret)
        return ret;

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    LOG_INF("Button initialized");
    return 0;
}

/* --- Sensor and Application Logic (mostly unchanged) --- */
// (Helper functions scale_accel_to_int16, scale_gyro_to_int16, read_and_notify_imu, sensor_imu_init remain the same)
#define IMU_FRAME_SIZE 22
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
static void read_and_notify_imu(void)
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

    // --- FIX: Get and pack a 64-bit microsecond timestamp ---
    uint64_t timestamp_us = k_uptime_get_32();
    memcpy(&frame[0], &timestamp_us, sizeof(timestamp_us));

    // Pack sensor data into the rest of the frame
    int16_t ax = scale_accel_to_int16(&accel[0]);
    int16_t ay = scale_accel_to_int16(&accel[1]);
    int16_t az = scale_accel_to_int16(&accel[2]);
    int16_t gx = scale_gyro_to_int16(&gyro[0]);
    int16_t gy = scale_gyro_to_int16(&gyro[1]);
    int16_t gz = scale_gyro_to_int16(&gyro[2]);

    printf(" Accel X (m/s^2): %d\n", ax);
    printf(" Accel Y (m/s^2): %d\n", ay);
    printf(" Accel Z (m/s^2): %d\n", az);
    printf(" Gyro  X (rad/s): %d\n", gx);
    printf(" Gyro  Y (rad/s): %d\n", gy);
    printf(" Gyro  Z (rad/s): %d\n", gz);
    memcpy(&frame[8], &ax, sizeof(ax));
    memcpy(&frame[10], &ay, sizeof(ay));
    memcpy(&frame[12], &az, sizeof(az));
    memcpy(&frame[14], &gx, sizeof(gx));
    memcpy(&frame[16], &gy, sizeof(gy));
    memcpy(&frame[18], &gz, sizeof(gz));

    imu_service_notify_motion(frame, sizeof(frame));
}
static int sensor_imu_init(void)
{
    if (!device_is_ready(imu_dev))
    {
        LOG_ERR("IMU not ready");
        return -ENODEV;
    }
    struct sensor_value odr_attr = {.val1 = 520, .val2 = 0};
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
    LOG_INF("IMU sensor configured for 520 Hz");
    return 0;
}

void main(void)
{
    int err;
    LOG_INF("IMU BLE Simple UX Example Starting...");

    // --- GPIO and Hardware Init ---
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

    // --- Bluetooth Init ---
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
    LOG_INF("Advertising started. Press button to start/stop streaming.");

    // --- Main Loop ---
    while (1)
    {
        // Target a 500 Hz polling rate
        k_sleep(K_MSEC(2));

        if (current_state == APP_STATE_SESSION)
        {
            read_and_notify_imu();
        }
    }
}