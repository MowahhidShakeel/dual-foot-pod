#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ux.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

// Track current state locally
static enum ux_state app_state = UX_STATE_BOOT;

// Callback for button events
static void ux_event_handler(enum ux_event ev)
{
    switch (ev)
    {
    case UX_EVENT_SHORT_PRESS:
        LOG_INF("Short press detected");
        // Toggle between IDLE and SESSION
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
        LOG_INF("Calibrate press (3s) detected");
        app_state = UX_STATE_CALIBRATE;
        ux_set_state(UX_STATE_CALIBRATE);
        k_sleep(K_MSEC(2000)); // Simulate calibration duration
        app_state = UX_STATE_IDLE;
        ux_set_state(UX_STATE_IDLE);
        break;
    case UX_EVENT_HARD_RESET:
        LOG_INF("Hard reset press (8s) detected");
        app_state = UX_STATE_ERROR;
        ux_trigger_error(); // Show error pattern
        k_sleep(K_MSEC(3000));
        app_state = UX_STATE_BOOT;
        ux_set_state(UX_STATE_BOOT); // Simulate reset
        break;
    default:
        LOG_WRN("Unknown UX event: %d", ev);
        break;
    }
}

void main(void)
{
    LOG_INF("Dual Foot-Pod Firmware: Booted on nRF5340");

    // Initialize UX module
    int rc = ux_init(ux_event_handler);
    if (rc != 0)
    {
        LOG_ERR("Failed to initialize UX module: %d", rc);
        ux_trigger_error();
        app_state = UX_STATE_ERROR;
        return;
    }

    // Transition to IDLE after boot
    k_sleep(K_MSEC(2000)); // Simulate boot duration
    app_state = UX_STATE_IDLE;
    ux_set_state(UX_STATE_IDLE);

    // Main loop (UX module handles events asynchronously)
    while (1)
    {
        k_sleep(K_MSEC(1000));
        LOG_DBG("Main loop running...");
    }
}