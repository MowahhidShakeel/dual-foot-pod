#ifndef UX_H_
#define UX_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ux_state {
    UX_STATE_BOOT,
    UX_STATE_IDLE,
    UX_STATE_CALIBRATE,
    UX_STATE_SESSION,
    UX_STATE_ERROR,
};

enum ux_event {
    UX_EVENT_NONE = 0,
    UX_EVENT_SHORT_PRESS,   /* short press (start/stop) */
    UX_EVENT_CALIBRATE,     /* 3s press */
    UX_EVENT_HARD_RESET,    /* 8s press */
    UX_EVENT_BUTTON_HELD,   /* internal: button held threshold */
};

/* signature for app callback */
typedef void (*ux_event_cb_t)(enum ux_event ev);

/* Initialize the UX module. Must be called once at boot (before usage). */
int ux_init(ux_event_cb_t cb);

/* Set the logical state; module will apply the matching LED pattern */
void ux_set_state(enum ux_state state);

/* Convenience: trigger an error pattern (e.g., pass a code if you want) */
void ux_trigger_error(void);

/* Change LED pattern directly (advanced use). Useful for diagnostics */
void ux_force_blink_once(void);

#ifdef __cplusplus
}
#endif

#endif /* UX_H_ */
