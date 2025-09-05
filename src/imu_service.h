#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include <zephyr/types.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** Initialize the IMU GATT service. Call after bt_enable() success. */
    int imu_service_init(void);

    /**
     * Notify connected subscribed clients with a motion frame.
     * - data: pointer to byte buffer (frame format below)
     * - len: length in bytes
     *
     * Returns 0 on success or a negative errno on failure.
     *
     * Frame format used by this example (packed, little-endian):
     * [t_us:uint64_t][ax:int16][ay:int16][az:int16][gx:int16][gy:int16][gz:int16][temp:int16]
     * => 8 + 2*7 = 22 bytes
     *
     * The notifier will:
     *  - check that notifications are enabled by the client,
     *  - use the negotiated ATT MTU to send chunks of at most (MTU - 3),
     *  - retry briefly on transient -ENOMEM buffer shortage.
     */
    int imu_service_notify_motion(const void *data, uint16_t len);

#endif /* IMU_SERVICE_H */
