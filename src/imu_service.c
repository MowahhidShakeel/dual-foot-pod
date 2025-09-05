#include "imu_service.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/random/random.h> /* sys_rand_get / sys_rand64_get */

#include <string.h>
#include <errno.h>
#include <stdint.h>

/* --- UUIDs (custom 128-bit) ---
 * Change these if you want to use your own registered UUIDs.
 */
static struct bt_uuid_128 imu_service_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab));

static struct bt_uuid_128 imu_motion_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x1234, 0x1234567890ab));

/* Notification enabled flag */
static bool motion_notify_enabled;

/* Keep pointer to connection (optional, useful for per-connection decisions) */
static struct bt_conn *default_conn;

/* Forward declaration: attribute array index to the *value attribute*
 * We'll rely on the fact that the value attribute is placed right after the
 * BT_GATT_CHARACTERISTIC macro in this attributes array. Index below matches that.
 */
enum
{
    ATTR_IDX_PRIMARY = 0,
    ATTR_IDX_MOTION_VAL, /* characteristic value attribute -> used in bt_gatt_notify */
    ATTR_IDX_MOTION_CCCD,
    ATTR_COUNT
};

/* CCC change handler */
static void imu_motion_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    motion_notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? true : false;
    printk("IMU: motion notify %s\n", motion_notify_enabled ? "ENABLED" : "DISABLED");
}

/* GATT attribute table (primary service, motion char, CCCD) */
BT_GATT_SERVICE_DEFINE(imu_svc,
                       BT_GATT_PRIMARY_SERVICE(&imu_service_uuid),

                       /* Motion characteristic: notify-only (read not provided here). Value attribute is next. */
                       BT_GATT_CHARACTERISTIC(&imu_motion_char_uuid.uuid,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE,
                                              NULL, /* read */
                                              NULL, /* write */
                                              NULL /* user_data (none) */),

                       /* CCC descriptor for notifications */
                       BT_GATT_CCC(imu_motion_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

/* Connection callbacks to track connections (optional but helpful) */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        printk("IMU: connection failed (err %u)\n", err);
        return;
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
    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    /* nothing else to 'register' — BT_GATT_SERVICE_DEFINE created the service */
    printk("IMU service: initialized\n");
    return 0;
}

int imu_service_notify_motion(const void *data, uint16_t len)
{
    if (!motion_notify_enabled)
    {
        return -EACCES; /* client hasn't enabled notifications */
    }

    /* The value attribute for the motion characteristic is the second attribute
     * inside imu_svc (index ATTR_IDX_MOTION_VAL). Use its address for notify.
     */
    const struct bt_gatt_attr *attr = &imu_svc.attrs[ATTR_IDX_MOTION_VAL];

    int rc = bt_gatt_notify(default_conn, attr, data, len);
    if (rc < 0)
    {
        printk("IMU: bt_gatt_notify failed: %d\n", rc);
    }
    return rc;
}
