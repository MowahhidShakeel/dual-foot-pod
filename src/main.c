// #include <zephyr/kernel.h>
// #include <zephyr/logging/log.h>
// #include "ux.h"

// LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

// // Track current state locally
// static enum ux_state app_state = UX_STATE_BOOT;

// // Callback for button events
// static void ux_event_handler(enum ux_event ev)
// {
//     switch (ev)
//     {
//     case UX_EVENT_SHORT_PRESS:
//         LOG_INF("Short press detected");
//         // Toggle between IDLE and SESSION
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
//         LOG_INF("Calibrate press (3s) detected");
//         app_state = UX_STATE_CALIBRATE;
//         ux_set_state(UX_STATE_CALIBRATE);
//         k_sleep(K_MSEC(2000)); // Simulate calibration duration
//         app_state = UX_STATE_IDLE;
//         ux_set_state(UX_STATE_IDLE);
//         break;
//     case UX_EVENT_HARD_RESET:
//         LOG_INF("Hard reset press (8s) detected");
//         app_state = UX_STATE_ERROR;
//         ux_trigger_error(); // Show error pattern
//         k_sleep(K_MSEC(3000));
//         app_state = UX_STATE_BOOT;
//         ux_set_state(UX_STATE_BOOT); // Simulate reset
//         break;
//     default:
//         LOG_WRN("Unknown UX event: %d", ev);
//         break;
//     }
// }

// void main(void)
// {
//     LOG_INF("Dual Foot-Pod Firmware: Booted on nRF5340");

//     // Initialize UX module
//     int rc = ux_init(ux_event_handler);
//     if (rc != 0)
//     {
//         LOG_ERR("Failed to initialize UX module: %d", rc);
//         ux_trigger_error();
//         app_state = UX_STATE_ERROR;
//         return;
//     }

//     // Transition to IDLE after boot
//     k_sleep(K_MSEC(2000)); // Simulate boot duration
//     app_state = UX_STATE_IDLE;
//     ux_set_state(UX_STATE_IDLE);

//     // Main loop (UX module handles events asynchronously)
//     while (1)
//     {
//         k_sleep(K_MSEC(1000));
//         LOG_DBG("Main loop running...");
//     }
// }
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include "imu_service.h"

/* Advertising service UUID: match service defined in imu_service.c */
static const struct bt_uuid_128 adv_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab));

/* Build advertising data with flags, 128-bit service UUID, and random manufacturer data */
static uint8_t mfg_data_buf[4];

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab)),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_buf, sizeof(mfg_data_buf)),
};

#define IMU_FRAME_SIZE 22 /* as defined in imu_service.h */

/* helper to fill an IMU frame with synthetic/random data */
static void fill_random_imu_frame(uint8_t *buf, size_t len)
{
    if (len < IMU_FRAME_SIZE)
    {
        return;
    }
    uint64_t t_us = sys_rand64_get(); /* non-crypto random timestamp substitute */
    memcpy(buf, &t_us, sizeof(t_us));

    /* ax ay az gx gy gz temp -> int16_t each */
    for (int i = 0; i < 7; i++)
    {
        int16_t v = (int16_t)(sys_rand32_get() & 0x7FFF) - 16000; /* pseudo-range +/-16k */
        memcpy(&buf[8 + i * 2], &v, sizeof(v));
    }
}

void main(void)
{
    int err;

    printk("IMU BLE example (nRF5340, NCS 2.9.1 compatible) starting...\n");

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    /* Initialize our IMU service (register callbacks etc) */
    imu_service_init();

    /* Prepare random manufacturer bytes for advertising (quick test payload) */
    sys_rand_get(mfg_data_buf, sizeof(mfg_data_buf));

    /* Start advertising: connectable advertising with name + service UUID + manufacturer data */
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
    }
    else
    {
        printk("Advertising started (manufacturer random bytes %02x%02x%02x%02x)\n",
               mfg_data_buf[0], mfg_data_buf[1], mfg_data_buf[2], mfg_data_buf[3]);
    }

    /* Periodically generate a sample IMU frame and attempt to notify subscribed clients */
    uint8_t frame[IMU_FRAME_SIZE];
    for (;;)
    {
        k_sleep(K_MSEC(200)); /* 200 ms example interval; reduce/increase as needed */

        fill_random_imu_frame(frame, sizeof(frame));

        int rc = imu_service_notify_motion(frame, sizeof(frame));
        if (rc == -EACCES)
        {
            /* no subscriber; that's expected until client subscribes */
            /* Do nothing */
        }
        else if (rc < 0)
        {
            printk("Notify error: %d\n", rc);
        }
        else
        {
            /* success */
            /* optionally print a short log: */
            printk("IMU: notified %u bytes\n", (unsigned)sizeof(frame));
        }
    }
}
