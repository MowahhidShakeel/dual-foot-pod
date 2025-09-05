#include "imu_service.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include <string.h>
#include <errno.h>
#include <stdint.h>

/* --- UUIDs (custom 128-bit) --- */
static struct bt_uuid_128 imu_service_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab));

static struct bt_uuid_128 imu_motion_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x1234, 0x1234567890ab));

/* Track whether notifications enabled (by CCCD) */
static bool motion_notify_enabled;

/* Track single reference to the current connection (if any) */
static struct bt_conn *default_conn;

/* Attribute indices:
 * BT_GATT_SERVICE_DEFINE expands attributes in this order:
 * 0: Primary Service
 * 1: Characteristic Declaration
 * 2: Characteristic Value (the attribute we must use to notify)
 * 3: CCC descriptor
 */
enum
{
    ATTR_IDX_PRIMARY = 0,
    ATTR_IDX_MOTION_CHRC,
    ATTR_IDX_MOTION_VAL,
    ATTR_IDX_MOTION_CCCD,
    ATTR_COUNT
};

/* CCC config changed callback */
static void imu_motion_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    motion_notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? true : false;
    printk("IMU: motion notify %s\n", motion_notify_enabled ? "ENABLED" : "DISABLED");
}

/* Optional simple read handler to expose a short string or version */
static ssize_t imu_motion_read(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    const char *payload = "IMUv1";
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, strlen(payload));
}

/* GATT service definition */
BT_GATT_SERVICE_DEFINE(imu_svc,
                       BT_GATT_PRIMARY_SERVICE(&imu_service_uuid),

                       /* Characteristic declaration + value (notify-only characteristic) */
                       BT_GATT_CHARACTERISTIC(&imu_motion_char_uuid.uuid,
                                              BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              imu_motion_read, /* read callback so clients can read a small token */
                                              NULL,
                                              NULL),

                       /* CCC descriptor for notifications */
                       BT_GATT_CCC(imu_motion_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

/* Connection callbacks */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        printk("IMU: connection failed (err %u)\n", err);
        return;
    }

    /* Keep a reference to the connection so we can notify on it.
     * Replace previous ref if any.
     */
    if (default_conn)
    {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    default_conn = bt_conn_ref(conn);
    printk("IMU: connected\n");
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    if (default_conn)
    {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    motion_notify_enabled = false;
    printk("IMU: disconnected (reason %u)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};

int imu_service_init(void)
{
    bt_conn_cb_register(&conn_callbacks);
    printk("IMU service: initialized\n");
    return 0;
}

/* Helper to obtain negotiated ATT MTU for the active conn.
 * If unknown, return default 23.
 */
static uint16_t get_conn_att_mtu(struct bt_conn *conn)
{
    uint16_t mtu = 0;

    if (conn)
    {
        /* bt_gatt_get_mtu() is available in Zephyr/NCS; returns negotiated ATT MTU */
        mtu = bt_gatt_get_mtu(conn);
    }

    if (mtu == 0)
    {
        mtu = 23; /* default ATT MTU */
    }

    return mtu;
}

/* Notify implementation that splits frames into chunks of (MTU - 3)
 * and does brief retries when -ENOMEM is returned.
 */
int imu_service_notify_motion(const void *data, uint16_t len)
{
    if (!motion_notify_enabled)
    {
        return -EACCES;
    }
    if (!default_conn)
    {
        return -ENOTCONN;
    }
    if (!data || len == 0)
    {
        return -EINVAL;
    }

    /* Locate value attribute correctly (see enum above) */
    const struct bt_gatt_attr *attr = &imu_svc.attrs[ATTR_IDX_MOTION_VAL];

    uint16_t mtu = get_conn_att_mtu(default_conn);
    /* payload allowed per ATT packet is (MTU - 3) */
    size_t max_payload = (mtu > 3) ? (mtu - 3) : 20; /* fallback to 20 */

    const uint8_t *ptr = data;
    size_t remaining = len;
    int rc = 0;

    while (remaining > 0)
    {
        size_t chunk = MIN(remaining, max_payload);

        /* Try sending; if -ENOMEM (no buffer) then retry a few times */
        int attempts = 0;
        const int max_attempts = 6;
        while (attempts < max_attempts)
        {
            rc = bt_gatt_notify(default_conn, attr, ptr, chunk);
            if (rc == -ENOMEM)
            {
                /* buffer transiently not available; wait a bit and retry */
                attempts++;
                /* small sleep to let pending transmissions drain */
                k_msleep(5 + attempts * 2);
                continue;
            }
            break;
        }

        if (rc < 0)
        {
            /* non-recoverable or retry exhausted */
            printk("IMU: notify chunk failed (rc=%d) after %d attempts\n", rc, attempts);
            return rc;
        }

        ptr += chunk;
        remaining -= chunk;
    }

    return 0;
}