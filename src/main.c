// main.c (updated)
// Advertising runs at all times (starts after BT init).
// IMU notifications are sent only while app_state == UX_STATE_SESSION.

// #include <zephyr/kernel.h>
// #include <zephyr/sys/byteorder.h>
// #include <zephyr/random/random.h>
// #include <zephyr/sys/util.h>
// #include <zephyr/sys/printk.h>

// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/hci.h>
// #include <zephyr/bluetooth/conn.h>
// #include <zephyr/bluetooth/gatt.h>

// #include "imu_service.h"
// #include "ux.h"

// #define IMU_FRAME_SIZE 22 /* as defined in imu_service.h */

// /* Advertising service UUID: match service defined in imu_service.c */
// static const struct bt_uuid_128 adv_uuid = BT_UUID_INIT_128(
//     BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab));

// /* Manufacturer data buffer used in advertising */
// static uint8_t mfg_data_buf[4];

// static const struct bt_data ad[] = {
//     BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     BT_DATA_BYTES(BT_DATA_UUID128_ALL,
//                   BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab)),
//     BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_buf, sizeof(mfg_data_buf)),
// };

// /* Application UX state (tracked locally) */
// static enum ux_state app_state = UX_STATE_BOOT;

// /* Advertising state flag */
// static atomic_t adv_active = ATOMIC_INIT(0);

// /* Work items to safely start/stop advertising from UX handler (or elsewhere) */
// static struct k_work adv_start_work;
// static struct k_work adv_stop_work;

// /* Forward declarations */
// static void start_advertising_impl(void);
// static void stop_advertising_impl(void);

// /* Work handlers */
// static void adv_start_work_handler(struct k_work *work)
// {
//     ARG_UNUSED(work);
//     start_advertising_impl();
// }

// static void adv_stop_work_handler(struct k_work *work)
// {
//     ARG_UNUSED(work);
//     stop_advertising_impl();
// }

// /* Start advertising implementation (thread/work context) */
// static void start_advertising_impl(void)
// {
//     /* If already advertising, return */
//     if (atomic_cas(&adv_active, 0, 1) == false)
//     {
//         return;
//     }

//     /* Prepare random manufacturer bytes for advertising (quick test payload) */
//     sys_rand_get(mfg_data_buf, sizeof(mfg_data_buf));

//     int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
//     if (err)
//     {
//         printk("Advertising failed to start (err %d)\n", err);
//         atomic_set(&adv_active, 0);
//     }
//     else
//     {
//         printk("Advertising started (mfg %02x%02x%02x%02x)\n",
//                mfg_data_buf[0], mfg_data_buf[1], mfg_data_buf[2], mfg_data_buf[3]);
//     }
// }

// /* Stop advertising implementation (thread/work context) */
// static void stop_advertising_impl(void)
// {
//     if (atomic_get(&adv_active) == 0)
//     {
//         return;
//     }

//     int err = bt_le_adv_stop();
//     if (err)
//     {
//         printk("Advertising stop failed (err %d)\n", err);
//     }
//     else
//     {
//         printk("Advertising stopped\n");
//     }
//     atomic_set(&adv_active, 0);
// }

// /* UX event handler: only SHORT_PRESS toggles SESSION <-> IDLE.
//  * Advertising is unaffected by these toggles (advertising runs regardless).
//  */
// static void ux_event_handler(enum ux_event ev)
// {
//     switch (ev)
//     {
//     case UX_EVENT_SHORT_PRESS:
//         printk("UX: Short press detected\n");
//         if (app_state == UX_STATE_IDLE)
//         {
//             app_state = UX_STATE_SESSION;
//             ux_set_state(UX_STATE_SESSION);
//         }
//         else if (app_state == UX_STATE_SESSION)
//         {
//             app_state = UX_STATE_IDLE;
//             ux_set_state(UX_STATE_IDLE);
//         }
//         break;

//     case UX_EVENT_CALIBRATE:
//         printk("UX: Calibrate press (3s) detected\n");
//         app_state = UX_STATE_CALIBRATE;
//         ux_set_state(UX_STATE_CALIBRATE);
//         k_sleep(K_MSEC(2000)); /* simulate calibration duration */
//         app_state = UX_STATE_IDLE;
//         ux_set_state(UX_STATE_IDLE);
//         break;

//     case UX_EVENT_HARD_RESET:
//         printk("UX: Hard reset press (8s) detected\n");
//         app_state = UX_STATE_ERROR;
//         ux_trigger_error(); /* show error pattern */
//         k_sleep(K_MSEC(3000));
//         app_state = UX_STATE_BOOT;
//         ux_set_state(UX_STATE_BOOT); /* simulate reset */
//         break;

//     default:
//         printk("UX: Unknown event %d\n", ev);
//         break;
//     }
// }

// /* helper to fill an IMU frame with synthetic/random data */
// static void fill_random_imu_frame(uint8_t *buf, size_t len)
// {
//     if (len < IMU_FRAME_SIZE)
//     {
//         return;
//     }
//     uint64_t t_us = sys_rand64_get(); /* non-crypto random timestamp substitute */
//     memcpy(buf, &t_us, sizeof(t_us));

//     /* ax ay az gx gy gz temp -> int16_t each */
//     for (int i = 0; i < 7; i++)
//     {
//         int16_t v = (int16_t)(sys_rand32_get() & 0x7FFF) - 16000; /* pseudo-range +/-16k */
//         memcpy(&buf[8 + i * 2], &v, sizeof(v));
//     }
// }

// void main(void)
// {
//     int err;

//     printk("IMU BLE example (nRF5340, NCS 2.9.1 compatible) starting...\n");

//     /* Initialize work items before UX init so handler can submit work safely */
//     k_work_init(&adv_start_work, adv_start_work_handler);
//     k_work_init(&adv_stop_work, adv_stop_work_handler);

//     /* Initialize UX module (passes ux_event_handler) */
//     err = ux_init(ux_event_handler);
//     if (err != 0)
//     {
//         printk("Failed to initialize UX module: %d\n", err);
//         ux_trigger_error();
//         app_state = UX_STATE_ERROR;
//         return;
//     }

//     /* Transition to IDLE after boot (UX shows IDLE pattern) */
//     k_sleep(K_MSEC(2000)); /* simulate boot duration */
//     app_state = UX_STATE_IDLE;
//     ux_set_state(UX_STATE_IDLE);

//     /* Initialize Bluetooth */
//     err = bt_enable(NULL);
//     if (err)
//     {
//         printk("Bluetooth init failed (err %d)\n", err);
//         return;
//     }
//     printk("Bluetooth initialized\n");

//     /* Initialize IMU GATT service (register callbacks etc) */
//     imu_service_init();

//     /* Start advertising immediately and keep it running (clients can connect anytime).
//      * Notifications will still only be sent while app_state == UX_STATE_SESSION.
//      */
//     start_advertising_impl();

//     /* Periodically generate a sample IMU frame and attempt to notify subscribed clients,
//      * but only while we're in SESSION state — advertising continues regardless.
//      */
//     uint8_t frame[IMU_FRAME_SIZE];
//     for (;;)
//     {
//         k_sleep(K_MSEC(200)); /* 200 ms example interval; adjust as needed */

//         if (app_state != UX_STATE_SESSION)
//         {
//             /* advertising still running, but we do not update/send IMU samples */
//             continue;
//         }

//         fill_random_imu_frame(frame, sizeof(frame));

//         int rc = imu_service_notify_motion(frame, sizeof(frame));
//         if (rc == -EACCES)
//         {
//             /* no subscriber; expected until client subscribes */
//         }
//         else if (rc < 0)
//         {
//             printk("Notify error: %d\n", rc);
//         }
//         else
//         {
//             printk("IMU: notified %u bytes\n", (unsigned)sizeof(frame));
//         }
//     }
// }

// #include <zephyr/device.h>
// #include <zephyr/drivers/sensor.h>
// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>

// void main(void)
// {
//     const struct device *imu = DEVICE_DT_GET_ONE(st_ism330dhcx);

//     while (!device_is_ready(imu))
//     {
//         printk("ISM330DHCX not ready!\n");
//         k_sleep(K_MSEC(2));
//     }

//     while (1)
//     {
//         struct sensor_value accel[3], gyro[3];

//         if (sensor_sample_fetch(imu) < 0)
//         {
//             printk("Failed to fetch sample\n");
//             continue;
//         }

//         sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
//         sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);

//         printk("Accel: %d.%06d, %d.%06d, %d.%06d m/s²\n",
//                accel[0].val1, accel[0].val2,
//                accel[1].val1, accel[1].val2,
//                accel[2].val1, accel[2].val2);

//         printk("Gyro: %d.%06d, %d.%06d, %d.%06d dps\n",
//                gyro[0].val1, gyro[0].val2,
//                gyro[1].val1, gyro[1].val2,
//                gyro[2].val1, gyro[2].val2);

//         k_sleep(K_MSEC(2)); /* 500 Hz target */
//     }
// }

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

/* Get the sensor device from the devicetree */
static const struct device *const ism330dhcx_dev = DEVICE_DT_GET(DT_INST(0, st_ism330dhcx));

static void print_sensor_value(const char *name, struct sensor_value val)
{
    printk("%s: %.6f\n", name, sensor_value_to_double(&val));
}

void main(void)
{
    // First, check if the sensor was found and is ready
    if (!device_is_ready(ism330dhcx_dev))
    {
        printk("Sensor device not ready: %s\n", ism330dhcx_dev->name);
        return;
    }

    printk("Found sensor: %s. Reading data...\n", ism330dhcx_dev->name);

    // --- NEW: Configure the sensor ---
    // This section activates the accelerometer and gyroscope by setting their
    // Output Data Rate (ODR) or sampling frequency.

    struct sensor_value odr_attr;

    // Set accelerometer ODR to 52 Hz
    odr_attr.val1 = 52;
    odr_attr.val2 = 0;

    if (sensor_attr_set(ism330dhcx_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0)
    {
        printk("Failed to set accelerometer ODR\n");
        return;
    }

    // Set gyroscope ODR to 52 Hz
    if (sensor_attr_set(ism330dhcx_dev, SENSOR_CHAN_GYRO_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) != 0)
    {
        printk("Failed to set gyroscope ODR\n");
        return;
    }

    printk("Sensor configured. Starting measurements...\n");

    // Loop forever, reading and printing data
    while (1)
    {
        if (sensor_sample_fetch(ism330dhcx_dev) != 0)
        {
            printk("Failed to fetch sensor sample\n");
            return;
        }

        struct sensor_value accel_x, accel_y, accel_z;
        struct sensor_value gyro_x, gyro_y, gyro_z;

        // Get Accelerometer Data
        sensor_channel_get(ism330dhcx_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(ism330dhcx_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(ism330dhcx_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

        // Get Gyroscope Data
        // NOTE: The correct enums for gyroscope axes are GYRO_DX, GYRO_DY, GYRO_DZ
        sensor_channel_get(ism330dhcx_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
        sensor_channel_get(ism330dhcx_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
        sensor_channel_get(ism330dhcx_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

        // Print the results
        printk("--- New Reading ---\n");
        print_sensor_value(" Accel X (m/s^2)", accel_x);
        print_sensor_value(" Accel Y (m/s^2)", accel_y);
        print_sensor_value(" Accel Z (m/s^2)", accel_z);
        print_sensor_value(" Gyro  X (rad/s)", gyro_x);
        print_sensor_value(" Gyro  Y (rad/s)", gyro_y);
        print_sensor_value(" Gyro  Z (rad/s)", gyro_z);

        // A shorter delay is better for seeing changes in sensor data
        k_sleep(K_MSEC(10));
    }
}