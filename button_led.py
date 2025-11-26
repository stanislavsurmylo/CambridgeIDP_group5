from machine import Pin
from utime import ticks_ms

LED_PIN = 28
BUTTON_PIN = 21
DEBOUNCE_MS = 200

led = Pin(LED_PIN, Pin.OUT)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)

_is_running = True  # bot starts in running state
_last_press_ms = 0
_on_start = None
_on_pause = None


def register_callbacks(on_start=None, on_pause=None):
    """
    Optionally provide functions to run when the button toggles the bot state.
    - on_start(): called when the bot transitions to running.
    - on_pause(): called when the bot transitions to paused.
    """
    global _on_start, _on_pause
    _on_start = on_start
    _on_pause = on_pause

    # Fire the appropriate callback immediately so the bot state is synced.
    if _is_running and _on_start:
        _on_start()
    elif not _is_running and _on_pause:
        _on_pause()


def is_bot_running():
    """Return True when the bot is in 'running' state."""
    return _is_running


def _handle_button(pin):
    """IRQ handler that toggles the bot state with debounce protection."""
    global _is_running, _last_press_ms
    now = ticks_ms()
    if now - _last_press_ms < DEBOUNCE_MS:
        return
    _last_press_ms = now

    _toggle_state()


def _toggle_state(initial=False):
    """Flip run/pause state and notify listeners."""
    global _is_running
    _is_running = not _is_running if not initial else _is_running
    led.value(_is_running)

    if _is_running:
        print("Bot started{}".format(" (initial)" if initial else ""))
        if _on_start:
            _on_start()
    else:
        print("Bot paused")
        if _on_pause:
            _on_pause()


# Attach interrupt so pressing GPIO21 toggles bot state without a polling loop.
button.irq(trigger=Pin.IRQ_RISING, handler=_handle_button)

# Ensure LED reflects the initial "running" state and emit notification.
led.value(_is_running)
print("Bot started (initial)")