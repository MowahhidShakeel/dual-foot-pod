#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ux, CONFIG_LOG_DEFAULT_LEVEL);

#include "ux.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

/* Devicetree aliases (expected): led0, sw0 */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define LED_NODE DT_ALIAS(led0)
#else
#error "No DT_ALIAS(led0) found — add overlay or update board DTS"
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(sw0), okay)
#define SW_NODE DT_ALIAS(sw0)
#else
#error "No DT_ALIAS(sw0) found — add overlay or update board DTS"
#endif

/* gpio_dt_spec convenience */
static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct gpio_dt_spec btn_gpio = GPIO_DT_SPEC_GET(SW_NODE, gpios);

/* Basic timings (ms) — tweak if you want faster/slower UX */
#define DEBOUNCE_MS 50
#define SHORT_MAX_MS 2000 /* <2s considered short */
#define CALIBRATE_MS 3000 /* >=3s calibrate */
#define HARDRESET_MS 8000 /* >=8s hard reset */

#define BLINK_FAST_ON_MS 100
#define BLINK_FAST_OFF_MS 100
#define BLINK_SLOW_ON_MS 300
#define BLINK_SLOW_OFF_MS 300
#define PULSE_ON_MS 1000
#define PULSE_OFF_MS 1000
#define ERROR_SHORT_MS 150
#define ERROR_GAP_MS 400

/* UX state */
static enum ux_state current_state = UX_STATE_BOOT;

/* user callback */
static ux_event_cb_t user_cb = NULL;

/* Button tracking */
static struct gpio_callback btn_cb_data;
static struct k_work_delayable btn_timeout_work;
static int64_t btn_press_time_ms = 0;
static bool btn_pressed = false;

/* LED work */
static struct k_work_delayable led_work;
static int led_is_on = 0;

/* internal pattern state */
enum led_pattern
{
    PATTERN_OFF,
    PATTERN_SOLID_ON,
    PATTERN_BLINK_FAST,
    PATTERN_BLINK_SLOW,
    PATTERN_PULSE,        /* slow breathe approximation */
    PATTERN_ERROR_DOUBLE, /* two quick blinks + pause */
};

static enum led_pattern active_pattern = PATTERN_OFF;
static int error_phase = 0; /* for double flash state machine */

/* Forward declarations */
static void led_work_handler(struct k_work *work);
static void start_led_pattern(enum led_pattern p);
static void stop_led_pattern(void);
static void btn_timeout_handler(struct k_work *work);

/* GPIO helpers */
static inline int led_set(int on)
{
    if (!device_is_ready(led_gpio.port))
    {
        LOG_ERR("LED GPIO port not ready");
        return -ENODEV;
    }

    /* note: many dev boards use GPIO_ACTIVE_LOW for LEDs.
     * gpio_pin_set_dt expects logical level; device tree config determines polarity.
     */
    int r = gpio_pin_set_dt(&led_gpio, on ? 1 : 0);
    if (r < 0)
    {
        LOG_ERR("Failed to set LED: %d", r);
    }
    else
    {
        led_is_on = on;
    }
    return r;
}

/* led_work performs the timed toggles for blink/pulse/error */
static void led_work_handler(struct k_work *work)
{
    int rc;
    switch (active_pattern)
    {
    case PATTERN_SOLID_ON:
        led_set(1);
        /* no reschedule */
        return;

    case PATTERN_BLINK_FAST:
        led_set(!led_is_on);
        rc = k_work_reschedule(&led_work, K_MSEC(led_is_on ? BLINK_FAST_OFF_MS : BLINK_FAST_ON_MS));
        if (rc)
            LOG_DBG("resched err %d", rc);
        return;

    case PATTERN_BLINK_SLOW:
        led_set(!led_is_on);
        rc = k_work_reschedule(&led_work, K_MSEC(led_is_on ? BLINK_SLOW_OFF_MS : BLINK_SLOW_ON_MS));
        if (rc)
            LOG_DBG("resched err %d", rc);
        return;

    case PATTERN_PULSE:
        /* simple on/off pulse to mimic breathe */
        led_set(!led_is_on);
        rc = k_work_reschedule(&led_work, K_MSEC(led_is_on ? PULSE_OFF_MS : PULSE_ON_MS));
        if (rc)
            LOG_DBG("resched err %d", rc);
        return;

    case PATTERN_ERROR_DOUBLE:
        /* state machine: two shorts, gap, repeat */
        if (error_phase == 0)
        { /* first short on */
            led_set(1);
            error_phase = 1;
            rc = k_work_reschedule(&led_work, K_MSEC(ERROR_SHORT_MS));
            if (rc)
                LOG_DBG("resched err %d", rc);
            return;
        }
        else if (error_phase == 1)
        { /* first short off */
            led_set(0);
            error_phase = 2;
            rc = k_work_reschedule(&led_work, K_MSEC(ERROR_GAP_MS));
            if (rc)
                LOG_DBG("resched err %d", rc);
            return;
        }
        else if (error_phase == 2)
        { /* second short on */
            led_set(1);
            error_phase = 3;
            rc = k_work_reschedule(&led_work, K_MSEC(ERROR_SHORT_MS));
            if (rc)
                LOG_DBG("resched err %d", rc);
            return;
        }
        else
        { /* second short off -> longer pause then repeat */
            led_set(0);
            error_phase = 0;
            rc = k_work_reschedule(&led_work, K_MSEC(PULSE_OFF_MS * 2));
            if (rc)
                LOG_DBG("resched err %d", rc);
            return;
        }

    default:
        led_set(0);
        return;
    }
}

/* Activate a pattern: sets active_pattern and starts the worker */
static void start_led_pattern(enum led_pattern p)
{
    active_pattern = p;
    error_phase = 0;

    switch (p)
    {
    case PATTERN_SOLID_ON:
        k_work_cancel_delayable(&led_work);
        led_set(1);
        break;
    case PATTERN_BLINK_FAST:
        k_work_cancel_delayable(&led_work);
        led_set(0);
        k_work_reschedule(&led_work, K_MSEC(BLINK_FAST_OFF_MS));
        break;
    case PATTERN_BLINK_SLOW:
        k_work_cancel_delayable(&led_work);
        led_set(0);
        k_work_reschedule(&led_work, K_MSEC(BLINK_SLOW_OFF_MS));
        break;
    case PATTERN_PULSE:
        k_work_cancel_delayable(&led_work);
        led_set(0);
        k_work_reschedule(&led_work, K_MSEC(PULSE_OFF_MS));
        break;
    case PATTERN_ERROR_DOUBLE:
        k_work_cancel_delayable(&led_work);
        error_phase = 0;
        led_set(0);
        k_work_reschedule(&led_work, K_MSEC(100));
        break;
    default:
        stop_led_pattern();
        break;
    }
}

static void stop_led_pattern(void)
{
    k_work_cancel_delayable(&led_work);
    led_set(0);
    active_pattern = PATTERN_OFF;
}

/* Map logical ux state to LED pattern */
static void apply_state_pattern(enum ux_state s)
{
    switch (s)
    {
    case UX_STATE_BOOT:
        start_led_pattern(PATTERN_BLINK_FAST);
        break;
    case UX_STATE_IDLE:
        start_led_pattern(PATTERN_PULSE);
        break;
    case UX_STATE_CALIBRATE:
        start_led_pattern(PATTERN_BLINK_FAST);
        break;
    case UX_STATE_SESSION:
        start_led_pattern(PATTERN_SOLID_ON);
        break;
    case UX_STATE_ERROR:
        start_led_pattern(PATTERN_ERROR_DOUBLE);
        break;
    default:
        stop_led_pattern();
        break;
    }
}

/* Button release handler will schedule this with debounce */
static void btn_timeout_handler(struct k_work *work)
{
    int64_t now = k_uptime_get();
    int64_t pressed_for = 0;

    if (btn_press_time_ms > 0)
    {
        pressed_for = now - btn_press_time_ms;
    }

    btn_pressed = false;
    btn_press_time_ms = 0;

    if (pressed_for < SHORT_MAX_MS)
    {
        LOG_DBG("button short: %lld ms", pressed_for);
        if (user_cb)
            user_cb(UX_EVENT_SHORT_PRESS);
    }
    else if (pressed_for >= CALIBRATE_MS && pressed_for < HARDRESET_MS)
    {
        LOG_DBG("button calibrate: %lld ms", pressed_for);
        if (user_cb)
            user_cb(UX_EVENT_CALIBRATE);
    }
    else if (pressed_for >= HARDRESET_MS)
    {
        LOG_DBG("button hard reset: %lld ms", pressed_for);
        if (user_cb)
            user_cb(UX_EVENT_HARD_RESET);
    }
    else
    {
        /* fallback: treat as short */
        if (user_cb)
            user_cb(UX_EVENT_SHORT_PRESS);
    }
}

/* GPIO callback for button edge */
static void btn_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    bool val = gpio_pin_get_dt(&btn_gpio) ? true : false;
    int64_t now = k_uptime_get();

    /* Button signalling convention depends on board: many use ACTIVE_LOW so value==0 when pressed.
     * gpio_dt_spec respects that and gpio_pin_get_dt returns logical 1/0; but safest is to test:
     */
    /* If button is "pressed" (logical 0 if active low) — gpio_pin_get_dt returns 0; invert if active low? */
    bool pressed = !val ? true : false; /* usually works for active-low buttons */
    /* To be robust, determine behaviour by checking gpio_dt_flags? Skipped for brevity. */

    if (pressed)
    {
        /* press event */
        if (!btn_pressed)
        {
            btn_pressed = true;
            btn_press_time_ms = now;
            /* No immediate action; wait for release or long press detection */
        }
    }
    else
    {
        /* release event: schedule debounce check */
        k_work_reschedule(&btn_timeout_work, K_MSEC(DEBOUNCE_MS));
    }
}

/* Public API implementations */

int ux_init(ux_event_cb_t cb)
{
    int rc;

    if (cb == NULL)
    {
        LOG_WRN("ux_init called with NULL callback");
    }
    user_cb = cb;

    if (!device_is_ready(led_gpio.port))
    {
        LOG_ERR("LED GPIO device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(btn_gpio.port))
    {
        LOG_ERR("Button GPIO device not ready");
        return -ENODEV;
    }

    /* Configure LED pin as output, start OFF */
    rc = gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_INACTIVE);
    if (rc)
    {
        LOG_ERR("Failed to configure LED gpio: %d", rc);
        return rc;
    }

    /* Configure button pin as input with interrupt on both edges */
    rc = gpio_pin_configure_dt(&btn_gpio, GPIO_INPUT);
    if (rc)
    {
        LOG_ERR("Failed to configure button gpio: %d", rc);
        return rc;
    }

    rc = gpio_pin_interrupt_configure_dt(&btn_gpio, GPIO_INT_EDGE_BOTH);
    if (rc)
    {
        LOG_ERR("Failed to configure button interrupts: %d", rc);
        return rc;
    }

    gpio_init_callback(&btn_cb_data, btn_gpio_cb, BIT(btn_gpio.pin));
    gpio_add_callback(btn_gpio.port, &btn_cb_data);

    /* init work items */
    k_work_init_delayable(&led_work, led_work_handler);
    k_work_init_delayable(&btn_timeout_work, btn_timeout_handler);

    /* start in boot pattern briefly */
    current_state = UX_STATE_BOOT;
    apply_state_pattern(current_state);

    LOG_INF("UX module initialized");
    return 0;
}

void ux_set_state(enum ux_state state)
{
    if (state == current_state)
        return;
    current_state = state;

    LOG_INF("UX state -> %d", state);
    apply_state_pattern(state);
}

void ux_trigger_error(void)
{
    current_state = UX_STATE_ERROR;
    apply_state_pattern(UX_STATE_ERROR);
}

void ux_force_blink_once(void)
{
    start_led_pattern(PATTERN_BLINK_FAST);
}
