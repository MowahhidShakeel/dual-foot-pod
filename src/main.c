// main.c (updated)
// Advertising runs at all times (starts after BT init).
// IMU notifications are sent only while app_state == UX_STATE_SESSION.

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include "imu_service.h"
#include "ux.h"

#define IMU_FRAME_SIZE 22 /* as defined in imu_service.h */

/* Advertising service UUID: match service defined in imu_service.c */
static const struct bt_uuid_128 adv_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab));

/* Manufacturer data buffer used in advertising */
static uint8_t mfg_data_buf[4];

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab)),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_buf, sizeof(mfg_data_buf)),
};

/* Application UX state (tracked locally) */
static enum ux_state app_state = UX_STATE_BOOT;

/* Advertising state flag */
static atomic_t adv_active = ATOMIC_INIT(0);

/* Work items to safely start/stop advertising from UX handler (or elsewhere) */
static struct k_work adv_start_work;
static struct k_work adv_stop_work;

/* Forward declarations */
static void start_advertising_impl(void);
static void stop_advertising_impl(void);

/* Work handlers */
static void adv_start_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    start_advertising_impl();
}

static void adv_stop_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    stop_advertising_impl();
}

/* Start advertising implementation (thread/work context) */
static void start_advertising_impl(void)
{
    /* If already advertising, return */
    if (atomic_cas(&adv_active, 0, 1) == false)
    {
        return;
    }

    /* Prepare random manufacturer bytes for advertising (quick test payload) */
    sys_rand_get(mfg_data_buf, sizeof(mfg_data_buf));

    int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        atomic_set(&adv_active, 0);
    }
    else
    {
        printk("Advertising started (mfg %02x%02x%02x%02x)\n",
               mfg_data_buf[0], mfg_data_buf[1], mfg_data_buf[2], mfg_data_buf[3]);
    }
}

/* Stop advertising implementation (thread/work context) */
static void stop_advertising_impl(void)
{
    if (atomic_get(&adv_active) == 0)
    {
        return;
    }

    int err = bt_le_adv_stop();
    if (err)
    {
        printk("Advertising stop failed (err %d)\n", err);
    }
    else
    {
        printk("Advertising stopped\n");
    }
    atomic_set(&adv_active, 0);
}

/* UX event handler: only SHORT_PRESS toggles SESSION <-> IDLE.
 * Advertising is unaffected by these toggles (advertising runs regardless).
 */
static void ux_event_handler(enum ux_event ev)
{
    switch (ev)
    {
    case UX_EVENT_SHORT_PRESS:
        printk("UX: Short press detected\n");
        if (app_state == UX_STATE_IDLE)
        {
            app_state = UX_STATE_SESSION;
            ux_set_state(UX_STATE_SESSION);
        }
        else if (app_state == UX_STATE_SESSION)
        {
            app_state = UX_STATE_IDLE;
            ux_set_state(UX_STATE_IDLE);
        }
        break;

    case UX_EVENT_CALIBRATE:
        printk("UX: Calibrate press (3s) detected\n");
        app_state = UX_STATE_CALIBRATE;
        ux_set_state(UX_STATE_CALIBRATE);
        k_sleep(K_MSEC(2000)); /* simulate calibration duration */
        app_state = UX_STATE_IDLE;
        ux_set_state(UX_STATE_IDLE);
        break;

    case UX_EVENT_HARD_RESET:
        printk("UX: Hard reset press (8s) detected\n");
        app_state = UX_STATE_ERROR;
        ux_trigger_error(); /* show error pattern */
        k_sleep(K_MSEC(3000));
        app_state = UX_STATE_BOOT;
        ux_set_state(UX_STATE_BOOT); /* simulate reset */
        break;

    default:
        printk("UX: Unknown event %d\n", ev);
        break;
    }
}

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

    /* Initialize work items before UX init so handler can submit work safely */
    k_work_init(&adv_start_work, adv_start_work_handler);
    k_work_init(&adv_stop_work, adv_stop_work_handler);

    /* Initialize UX module (passes ux_event_handler) */
    err = ux_init(ux_event_handler);
    if (err != 0)
    {
        printk("Failed to initialize UX module: %d\n", err);
        ux_trigger_error();
        app_state = UX_STATE_ERROR;
        return;
    }

    /* Transition to IDLE after boot (UX shows IDLE pattern) */
    k_sleep(K_MSEC(2000)); /* simulate boot duration */
    app_state = UX_STATE_IDLE;
    ux_set_state(UX_STATE_IDLE);

    /* Initialize Bluetooth */
    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    /* Initialize IMU GATT service (register callbacks etc) */
    imu_service_init();

    /* Start advertising immediately and keep it running (clients can connect anytime).
     * Notifications will still only be sent while app_state == UX_STATE_SESSION.
     */
    start_advertising_impl();

    /* Periodically generate a sample IMU frame and attempt to notify subscribed clients,
     * but only while we're in SESSION state — advertising continues regardless.
     */
    uint8_t frame[IMU_FRAME_SIZE];
    for (;;)
    {
        k_sleep(K_MSEC(200)); /* 200 ms example interval; adjust as needed */

        if (app_state != UX_STATE_SESSION)
        {
            /* advertising still running, but we do not update/send IMU samples */
            continue;
        }

        fill_random_imu_frame(frame, sizeof(frame));

        int rc = imu_service_notify_motion(frame, sizeof(frame));
        if (rc == -EACCES)
        {
            /* no subscriber; expected until client subscribes */
        }
        else if (rc < 0)
        {
            printk("Notify error: %d\n", rc);
        }
        else
        {
            printk("IMU: notified %u bytes\n", (unsigned)sizeof(frame));
        }
    }
}
