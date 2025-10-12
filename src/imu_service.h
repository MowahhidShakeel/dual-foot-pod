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
     * [seq_id:uint16][t_us:uint64][ax:int16][ay:int16][az:int16][gx:int16][gy:int16][gz:int16][temp:int16]
     * => 2 + 8 + 2*7 = 24 bytes
     *
     * The notifier will:
     *  - check that notifications are enabled by the client,
     *  - use the negotiated ATT MTU to send chunks of at most (MTU - 3),
     *  - retry briefly on transient -ENOMEM buffer shortage.
     */
    int imu_service_notify_motion(const void *data, uint16_t len);

    /**
     * Check if the host has sent a start command.
     * Returns true if start command received, false otherwise.
     */
    bool imu_service_is_start_commanded(void);

#endif /* IMU_SERVICE_H */