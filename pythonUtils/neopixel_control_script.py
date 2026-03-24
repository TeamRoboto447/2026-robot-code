from pi5neo import Pi5Neo
from networktables import NetworkTables, NetworkTablesInstance
import time, sys, tty, termios, threading

NUM_LEDS = 300
NT_LIVE = True

# ------------------------------------------------------------------ #
# Configuration
# ------------------------------------------------------------------ #

BRIGHTNESS = 0.3

DEFAULT_PPS   = 10
DEFAULT_PURE  = 20
DEFAULT_TRANS = 5.0

TRIGGER_PPS   = 100
TRIGGER_PURE  = 5.0
TRIGGER_TRANS = 20.0

RAMP_UP_DURATION   = 7.0
RAMP_DOWN_DURATION = 10.0
HOLD_DURATION      = 2.0

BLINK_ON_TIME  = 0.3   # seconds each color is ON
BLINK_OFF_TIME = 0.2   # seconds the strip is OFF between colors

# Cycle order during the hold blink
BLINK_COLORS = [
    (255, 255, 255),   # White
    (255,   0,   0),   # Red
    (  0,   0, 255),   # Blue
]

COLORS = [
    (255,   0,   0),
    (  0,   0, 255),
]
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# ------------------------------------------------------------------ #
# Color math
# ------------------------------------------------------------------ #

def lerp(a, b, t):
    return a + (b - a) * t

def lerp_color(c1, c2, t):
    t = max(0.0, min(1.0, t))
    return (
        int(lerp(c1[0], c2[0], t)),
        int(lerp(c1[1], c2[1], t)),
        int(lerp(c1[2], c2[2], t)),
    )

def apply_brightness(r, g, b, brightness=1.0):
    brightness = max(0.0, min(1.0, brightness))
    return int(r * brightness), int(g * brightness), int(b * brightness)

def get_color(pos, pure_px, trans_px):
    n         = len(COLORS)
    zone_size = pure_px + trans_px
    total     = zone_size * n

    p     = pos % total
    zone  = min(int(p / zone_size), n - 1)
    local = max(0.0, p - zone * zone_size)

    c1 = COLORS[zone]
    c2 = COLORS[(zone + 1) % n]

    if local <= pure_px:
        return c1

    t = (local - pure_px) / trans_px
    if t < 0.5:
        return lerp_color(c1, WHITE, t * 2.0)
    else:
        return lerp_color(WHITE, c2, (t - 0.5) * 2.0)

def get_blink_color(elapsed):
    """
    Returns (r, g, b) for the current blink phase, or None if in the
    OFF gap between colors.

    Each step in the cycle = BLINK_ON_TIME + BLINK_OFF_TIME.
    Within a step: [0, BLINK_ON_TIME) = color ON, rest = OFF.
    """
    step_duration = BLINK_ON_TIME + BLINK_OFF_TIME
    cycle_duration = step_duration * len(BLINK_COLORS)

    pos_in_cycle = elapsed % cycle_duration
    step_index   = int(pos_in_cycle / step_duration)
    pos_in_step  = pos_in_cycle - step_index * step_duration

    if pos_in_step < BLINK_ON_TIME:
        return BLINK_COLORS[step_index % len(BLINK_COLORS)]
    return None   # OFF gap

# ------------------------------------------------------------------ #
# State machine
# ------------------------------------------------------------------ #

IDLE, RAMP_UP, TRIGGERED, RAMP_DOWN = range(4)

space_event = threading.Event()
stop_event  = threading.Event()

def ease_in_out(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)

def keyboard_listener():
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not stop_event.is_set():
            ch = sys.stdin.read(1)
            if ch == ' ':
                space_event.set()
            elif ch in ('\x03', 'q'):
                stop_event.set()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ------------------------------------------------------------------ #
# Main loop
# ------------------------------------------------------------------ #

def scroll(neo):
    state       = IDLE
    state_start = time.monotonic()
    offset      = 0.0
    last_t      = time.monotonic()

    print("Running.")

    while not stop_event.is_set():
        now     = time.monotonic()
        dt      = now - last_t
        last_t  = now
        elapsed = now - state_start

        # ---- State transitions ----
        if state == IDLE and space_event.is_set():
            space_event.clear()
            state = RAMP_UP
            state_start = now
            elapsed = 0.0
            print(">> Triggered: ramping up...")

        elif state == RAMP_UP and elapsed >= RAMP_UP_DURATION:
            state = TRIGGERED
            state_start = now
            elapsed = 0.0
            print(">> Holding: cycling blink.")

        elif state == TRIGGERED and elapsed >= HOLD_DURATION:
            state = RAMP_DOWN
            state_start = now
            elapsed = 0.0
            print(">> Ramping down...")

        elif state == RAMP_DOWN and elapsed >= RAMP_DOWN_DURATION:
            state = IDLE
            state_start = now
            elapsed = 0.0
            print(">> Back to idle.")

        # ---- Blink override during TRIGGERED ----
        if state == TRIGGERED:
            blink_color = get_blink_color(elapsed)
            if blink_color is not None:
                r, g, b = apply_brightness(*blink_color, BRIGHTNESS)
                for i in range(NUM_LEDS):
                    neo.set_led_color(i, r, g, b)
            else:
                for i in range(NUM_LEDS):
                    neo.set_led_color(i, 0, 0, 0)
            neo.update_strip()
            time.sleep(max(0.0, (1.0 / 60.0) - (time.monotonic() - now)))
            continue

        # ---- Factor ----
        if state == IDLE:
            factor = 0.0
        elif state == RAMP_UP:
            factor = ease_in_out(elapsed / RAMP_UP_DURATION)
        else:  # RAMP_DOWN
            factor = ease_in_out(1.0 - elapsed / RAMP_DOWN_DURATION)

        # ---- Interpolate parameters ----
        pure_px  = lerp(DEFAULT_PURE,  TRIGGER_PURE,  factor)
        trans_px = lerp(DEFAULT_TRANS, TRIGGER_TRANS, factor)
        pps      = lerp(DEFAULT_PPS,   TRIGGER_PPS,   factor)

        # ---- Advance and wrap offset ----
        zone_size = pure_px + trans_px
        total     = zone_size * len(COLORS)
        offset    = (offset + pps * dt) % total

        # ---- Render gradient ----
        for i in range(NUM_LEDS):
            r, g, b = get_color(i + offset, pure_px, trans_px)
            r, g, b = apply_brightness(r, g, b, BRIGHTNESS)
            neo.set_led_color(i, r, g, b)

        neo.update_strip()
        time.sleep(max(0.0, (1.0 / 60.0) - (time.monotonic() - now)))

def flash_strip(neo : Pi5Neo, pauseEvent : threading.Thread, r, g, b, on_time, off_time):
    while not stop_event.is_set:
        while pauseEvent.is_set:
            r, g, b = apply_brightness(r, g, b, BRIGHTNESS)
            for i in range(NUM_LEDS):
                neo.set_led_color(i, r, g, b)
            time.sleep(on_time)

            for i in range(NUM_LEDS):
                neo.set_led_color(i, 0, 0, 0)
            neo.update_strip()
            time.sleep(off_time)
        time.sleep(0.1)

# ------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------ #

if __name__ == "__main__":
    try:
        pixelStrip = Pi5Neo('/dev/spidev0.0', NUM_LEDS, 800)

        disabledNoCamEvent = threading.Event()
        disabledNoTagsEvent = threading.Event()

        disabledNoCamThread = threading.Thread(target=flash_strip, args=(pixelStrip, disabledNoCamEvent, *RED, 0.3, 0.2))
        disabledNoCamThread.start()
        

        nt = NetworkTablesInstance.getDefault()
        if NT_LIVE:
            nt.initialize(server=("10.4.47.2", 1735))
        else:
            nt.initialize(server="127.0.0.1")
        
        pixelControlTable = nt.getTable("Neopixels")
        controlModeEntry = pixelControlTable.getEntry("Control Mode")
        controlTriggerEntry = pixelControlTable.getEntry("Control Trigger")

        controlModeEntry.setDefaultString("DISABLED_NO_CAMERA")
        controlTriggerEntry.setDefaultString("No Control")

        while True:
            controlMode = controlModeEntry.getString()
            controlTrigger = controlTriggerEntry.getString()

            disabledNoCamEvent.set(controlMode == "DISABLED_NO_CAMERA")
            disabledNoTagsEvent.set(controlMode == "DISABLED_NO_TAGS")


            time.sleep(0.05)


    except KeyboardInterrupt:
        stop_event.set()
    finally:
        pixelStrip.clear_strip()
        pixelStrip.update_strip()

