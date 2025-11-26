from machine import Pin
from utime import ticks_ms
import rp2

# Use the same yellow LED pin as in `yellow led blink.py`
LED_PIN = 25
BUTTON_PIN = 21
DEBOUNCE_MS = 200

led = Pin(LED_PIN, Pin.OUT)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)

_is_running = True  # bot starts in running state
_last_press_ms = 0
_on_start = None
_on_pause = None
_sm = None  # PIO state machine for blinking the yellow LED


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_1hz():
    # Same blink pattern as in `yellow led blink.py`
    set(pins, 1)           # LED ON
    set(x, 31)             [6]
    label("delay_high")
    nop()                  [29]
    jmp(x_dec, "delay_high")

    set(pins, 0)           # LED OFF
    set(x, 31)             [6]
    label("delay_low")
    nop()                  [29]
    jmp(x_dec, "delay_low")


def _ensure_blink_sm():
    """Create the PIO state machine for the yellow LED if it doesn't exist."""
    global _sm
    if _sm is None:
        _sm = rp2.StateMachine(
            1,              # use SM1 to avoid clashing with other code
            blink_1hz,
            freq=2000,
            set_base=Pin(LED_PIN),
        )
        _sm.active(0)


def register_callbacks(on_start=None, on_pause=None):
    global _on_start, _on_pause
    _on_start = on_start
    _on_pause = on_pause

    if _is_running and _on_start:
        _on_start()
    elif not _is_running and _on_pause:
        _on_pause()


def init_button_interrupt(on_start=None, on_pause=None):
    register_callbacks(on_start=on_start, on_pause=on_pause)


def is_bot_running():
    return _is_running


def _handle_button(pin):
    global _is_running, _last_press_ms
    now = ticks_ms()
    if now - _last_press_ms < DEBOUNCE_MS:
        return
    _last_press_ms = now
    _toggle_state()


def _toggle_state(initial=False):
    global _is_running, _sm
    _ensure_blink_sm()
    _is_running = not _is_running if not initial else _is_running
    # Start/stop yellow LED blinking via PIO
    if _is_running:
        _sm.active(1)
    else:
        _sm.active(0)
        led.value(0)  # make sure LED is off when paused

    if _is_running:
        print("Bot started{}".format(" (initial)" if initial else ""))
        if _on_start:
            _on_start()
    else:
        print("Bot paused")
        if _on_pause:
            _on_pause()



button.irq(trigger=Pin.IRQ_RISING, handler=_handle_button)


_ensure_blink_sm()
if _is_running:
    _sm.active(1)
else:
    _sm.active(0)
    led.value(0)
print("Bot started (initial)")