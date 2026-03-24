"""NeoPixel LED strip control for FRC robot 2026."""

from pi5neo import Pi5Neo
from networktables import NetworkTables, NetworkTablesInstance
import time
import logging
from enum import Enum
from typing import Optional
import math

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

NUM_LEDS = 45
NT_LIVE = True
BRIGHTNESS = 0.3

DEFAULT_PPS = 10
DEFAULT_PURE = 20
DEFAULT_TRANS = 5.0
TRIGGER_PPS = 100
TRIGGER_PURE = 5.0
TRIGGER_TRANS = 20.0

RAMP_UP_DURATION = 5
HOLD_DURATION = 2
RAMP_DOWN_DURATION = 7

BLINK_ON_TIME = 0.3
BLINK_OFF_TIME = 0.2
PULSE_PERIOD = 1.0

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
COLORS = [RED, BLUE]
BLINK_COLORS = [WHITE, RED, BLUE]


def blink_is_on(elapsed: float) -> bool:
    """Whether a basic blink cycle is in the ON portion."""
    step_duration = BLINK_ON_TIME + BLINK_OFF_TIME
    pos_in_step = elapsed % step_duration
    return pos_in_step < BLINK_ON_TIME


class ControlMode(Enum):
    """Valid LED control modes from robot."""
    DISABLED_NO_CAMERA = "DISABLED_NO_CAMERA"
    DISABLED_NO_TAGS = "DISABLED_NO_TAGS"
    DISABLED_HAS_TAGS = "DISABLED_HAS_TAGS"
    DISABLED_CORRECT_POSITION = "DISABLED_CORRECT_POSITION"
    ENABLED_DEFAULT = "ENABLED_DEFAULT"


class TriggerType(Enum):
    """Recognized trigger IDs."""
    PHASE_SHIFT_INCOMING = "PHASE_SHIFT_INCOMING"


def nt_get_string(entry, default: str) -> str:
    """Read a string from a NetworkTables entry across API variants."""
    if hasattr(entry, "getString"):
        return entry.getString(default)
    if hasattr(entry, "get"):
        try:
            return entry.get(default)
        except TypeError:
            value = entry.get()
            return default if value is None else str(value)
    return default


def nt_set_string(entry, value: str) -> None:
    """Write a string to a NetworkTables entry across API variants."""
    if hasattr(entry, "setString"):
        entry.setString(value)
    elif hasattr(entry, "set"):
        entry.set(value)


def nt_set_default_string(table, key: str, default: str):
    """Create/publish a string NT key with a default value."""
    if hasattr(table, "putString"):
        try:
            current = table.getString(key, "")
        except Exception:
            current = ""
        if current in (None, ""):
            table.putString(key, default)
        return table.getEntry(key)

    entry = table.getEntry(key)
    if hasattr(entry, "setDefaultString"):
        entry.setDefaultString(default)
    else:
        current = nt_get_string(entry, "")
        if current in (None, ""):
            nt_set_string(entry, default)
    return entry


def nt_is_connected(nt_instance) -> bool:
    """Best-effort NT connection check across API variants."""
    if hasattr(nt_instance, "isConnected"):
        return bool(nt_instance.isConnected())
    if hasattr(NetworkTables, "isConnected"):
        return bool(NetworkTables.isConnected())
    return False


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation."""
    return a + (b - a) * t


def lerp_color(c1: tuple, c2: tuple, t: float) -> tuple:
    """Linear interpolation between two colors."""
    t = max(0.0, min(1.0, t))
    return (
        int(lerp(c1[0], c2[0], t)),
        int(lerp(c1[1], c2[1], t)),
        int(lerp(c1[2], c2[2], t)),
    )


def apply_brightness(r: int, g: int, b: int, brightness: float = 1.0) -> tuple:
    """Apply brightness scaling to RGB."""
    brightness = max(0.0, min(1.0, brightness))
    return (int(r * brightness), int(g * brightness), int(b * brightness))


def get_color(pos: float, pure_px: float, trans_px: float) -> tuple:
    """Get color for scrolling animation at position."""
    n = len(COLORS)
    zone_size = pure_px + trans_px
    total = zone_size * n
    p = pos % total
    zone = min(int(p / zone_size), n - 1)
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


def get_blink_color(elapsed: float) -> Optional[tuple]:
    """Get color for blink cycle, or None if in OFF gap."""
    step_duration = BLINK_ON_TIME + BLINK_OFF_TIME
    cycle_duration = step_duration * len(BLINK_COLORS)
    pos_in_cycle = elapsed % cycle_duration
    step_index = int(pos_in_cycle / step_duration)
    pos_in_step = pos_in_cycle - step_index * step_duration
    if pos_in_step < BLINK_ON_TIME:
        return BLINK_COLORS[step_index % len(BLINK_COLORS)]
    return None


def get_pulse_color(elapsed: float) -> tuple:
    """Get color for pulse effect (sine wave brightness on red)."""
    phase = (elapsed / PULSE_PERIOD) % 1.0
    brightness = 0.3 + 0.7 * abs(math.sin(phase * math.pi))
    return apply_brightness(*RED, brightness)


def ease_in_out(t: float) -> float:
    """Smoothstep easing."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def render_solid(neo: Pi5Neo, color: tuple) -> None:
    """Fill entire strip with solid color."""
    r, g, b = apply_brightness(*color, BRIGHTNESS)
    for i in range(NUM_LEDS):
        neo.set_led_color(i, r, g, b)


def render_flash(neo: Pi5Neo, elapsed: float) -> None:
    """Flash color on/off based on elapsed time."""
    blink = get_blink_color(elapsed)
    if blink is not None:
        r, g, b = apply_brightness(*blink, BRIGHTNESS)
        for i in range(NUM_LEDS):
            neo.set_led_color(i, r, g, b)
    else:
        for i in range(NUM_LEDS):
            neo.set_led_color(i, 0, 0, 0)


def render_flash_single_color(neo: Pi5Neo, elapsed: float, color: tuple) -> None:
    """Flash a single color on/off based on elapsed time."""
    if blink_is_on(elapsed):
        r, g, b = apply_brightness(*color, BRIGHTNESS)
        for i in range(NUM_LEDS):
            neo.set_led_color(i, r, g, b)
    else:
        for i in range(NUM_LEDS):
            neo.set_led_color(i, 0, 0, 0)


def render_scroll(neo: Pi5Neo, offset: float, pure_px: float, trans_px: float) -> None:
    """Render scrolling gradient animation."""
    for i in range(NUM_LEDS):
        r, g, b = get_color(i + offset, pure_px, trans_px)
        r, g, b = apply_brightness(r, g, b, BRIGHTNESS)
        neo.set_led_color(i, r, g, b)


def render_pulse(neo: Pi5Neo, elapsed: float) -> None:
    """Render pulsing red (error/fallback state)."""
    color = get_pulse_color(elapsed)
    for i in range(NUM_LEDS):
        neo.set_led_color(i, *color)


class ControlState:
    """Encapsulates current control state."""
    
    def __init__(self):
        self.mode: Optional[ControlMode] = None
        self.trigger_id: Optional[str] = None
        self.nt_connected: bool = False
        self.nt_last_heard: float = 0.0


class TriggerState:
    """Encapsulates trigger animation phase."""
    
    def __init__(self, trigger_type: TriggerType):
        self.trigger_type = trigger_type
        self.start_time: float = 0.0
        self.phase: str = "IDLE"
    
    def update(self, now: float) -> None:
        """Advance trigger state machine."""
        elapsed = now - self.start_time
        if self.phase == "IDLE":
            return
        if self.phase == "RAMP_UP" and elapsed >= RAMP_UP_DURATION:
            self.phase = "HOLD"
        elif self.phase == "HOLD" and elapsed >= RAMP_UP_DURATION + HOLD_DURATION:
            self.phase = "RAMP_DOWN"
        elif self.phase == "RAMP_DOWN" and elapsed >= RAMP_UP_DURATION + HOLD_DURATION + RAMP_DOWN_DURATION:
            self.phase = "IDLE"
    
    def is_active(self) -> bool:
        """Whether trigger is currently animating."""
        return self.phase != "IDLE"
    
    def get_blend_factor(self, now: float) -> float:
        """Get blend factor [0, 1] for trigger intensity."""
        elapsed = now - self.start_time
        if self.phase == "RAMP_UP":
            return ease_in_out(elapsed / RAMP_UP_DURATION)
        elif self.phase == "HOLD":
            return 1.0
        elif self.phase == "RAMP_DOWN":
            return ease_in_out(1.0 - (elapsed - RAMP_UP_DURATION - HOLD_DURATION) / RAMP_DOWN_DURATION)
        return 0.0


def setup_networktables() -> tuple:
    """Initialize NetworkTables and set up listeners."""
    nt = NetworkTablesInstance.getDefault()
    if NT_LIVE:
        nt.initialize(server=("10.4.47.2", 1735))
        logger.info("Connecting to NT server at 10.4.47.2:1735")
    else:
        nt.initialize(server="127.0.0.1")
        logger.info("Connecting to NT server at 127.0.0.1")
    
    table = nt.getTable("Neopixels")
    mode_entry = nt_set_default_string(table, "Control Mode", "DISABLED_NO_CAMERA")
    trigger_entry = nt_set_default_string(table, "Control Trigger", "")
    control_state = ControlState()

    control_state.nt_connected = nt_is_connected(nt)
    control_state.nt_last_heard = time.monotonic()
    return table, mode_entry, trigger_entry, control_state


def main_loop(neo: Pi5Neo, mode_entry, trigger_entry, control_state: ControlState, trigger_state: TriggerState) -> None:
    """Main animation loop."""
    TARGET_FPS = 60
    FRAME_TIME = 1.0 / TARGET_FPS
    NT_TIMEOUT = 5.0
    
    scroll_offset = 0.0
    last_time = time.monotonic()
    frame_count = 0
    last_mode_value = ""
    last_trigger_value = ""
    nt = NetworkTablesInstance.getDefault()
    
    logger.info("Starting main render loop...")
    
    while True:
        try:
            now = time.monotonic()
            dt = now - last_time
            last_time = now
            frame_count += 1

            control_state.nt_connected = nt_is_connected(nt)
            if control_state.nt_connected:
                control_state.nt_last_heard = now

            mode_value = nt_get_string(mode_entry, "DISABLED_NO_CAMERA")
            if mode_value != last_mode_value:
                last_mode_value = mode_value
                try:
                    control_state.mode = ControlMode(mode_value)
                    logger.info(f"Mode changed: {mode_value}")
                except ValueError:
                    logger.warning(f"Invalid mode: {mode_value}")
                    control_state.mode = None

            trigger_value = nt_get_string(trigger_entry, "")
            if trigger_value != last_trigger_value:
                last_trigger_value = trigger_value
                if trigger_value:
                    control_state.trigger_id = trigger_value
                    logger.info(f"Trigger received: {trigger_value}")
            
            time_since_last_nt = now - control_state.nt_last_heard
            if time_since_last_nt > NT_TIMEOUT and control_state.nt_connected:
                logger.warning(f"NT timeout: {time_since_last_nt:.1f}s")
                control_state.nt_connected = False
            
            trigger_state.update(now)
            
            if control_state.trigger_id:
                try:
                    trigger_enum = TriggerType(control_state.trigger_id)
                    trigger_state.trigger_type = trigger_enum
                    trigger_state.start_time = now
                    trigger_state.phase = "RAMP_UP"
                    logger.info(f"Trigger latched: {trigger_enum.name}")
                    nt_set_string(trigger_entry, "")
                    control_state.trigger_id = None
                except ValueError:
                    logger.warning(f"Unknown trigger ID: {control_state.trigger_id}")
                    nt_set_string(trigger_entry, "")
                    control_state.trigger_id = None
            
            if not control_state.nt_connected or control_state.mode is None:
                render_pulse(neo, now)
                if frame_count % 60 == 0:
                    logger.warning(f"Using fallback: NT connected={control_state.nt_connected}, mode={control_state.mode}")
            else:
                mode = control_state.mode
                if mode == ControlMode.DISABLED_NO_CAMERA:
                    render_flash_single_color(neo, now, RED)
                elif mode == ControlMode.DISABLED_NO_TAGS:
                    render_solid(neo, RED)
                elif mode == ControlMode.DISABLED_HAS_TAGS:
                    render_solid(neo, WHITE)
                elif mode == ControlMode.DISABLED_CORRECT_POSITION:
                    render_solid(neo, BLUE)
                elif mode == ControlMode.ENABLED_DEFAULT:
                    blend = trigger_state.get_blend_factor(now) if trigger_state.is_active() else 0.0
                    pure_px = lerp(DEFAULT_PURE, TRIGGER_PURE, blend)
                    trans_px = lerp(DEFAULT_TRANS, TRIGGER_TRANS, blend)
                    pps = lerp(DEFAULT_PPS, TRIGGER_PPS, blend)
                    zone_size = pure_px + trans_px
                    total = zone_size * len(COLORS)
                    scroll_offset = (scroll_offset + pps * dt) % total
                    
                    if trigger_state.phase == "HOLD":
                        trigger_elapsed = now - trigger_state.start_time - RAMP_UP_DURATION
                        blink_color = get_blink_color(trigger_elapsed)
                        if blink_color is not None:
                            r, g, b = apply_brightness(*blink_color, BRIGHTNESS)
                            for i in range(NUM_LEDS):
                                neo.set_led_color(i, r, g, b)
                        else:
                            for i in range(NUM_LEDS):
                                neo.set_led_color(i, 0, 0, 0)
                    else:
                        render_scroll(neo, scroll_offset, pure_px, trans_px)
            
            neo.update_strip()
            elapsed_frame = time.monotonic() - now
            sleep_time = max(0.0, FRAME_TIME - elapsed_frame)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        except Exception as e:
            logger.error(f"Error in main loop: {e}", exc_info=True)
            time.sleep(0.1)


if __name__ == "__main__":
    pixel_strip: Optional[Pi5Neo] = None
    try:
        logger.info("Initializing NeoPixel LED strip control...")
        pixel_strip = Pi5Neo('/dev/spidev0.0', NUM_LEDS, 800)
        logger.info(f"LED strip initialized: {NUM_LEDS} pixels")
        table, mode_entry, trigger_entry, control_state = setup_networktables()
        trigger_state = TriggerState(TriggerType.PHASE_SHIFT_INCOMING)
        logger.info("Setup complete. Starting render loop...")
        main_loop(pixel_strip, mode_entry, trigger_entry, control_state, trigger_state)
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
    finally:
        if pixel_strip is not None:
            try:
                pixel_strip.clear_strip()
                pixel_strip.update_strip()
                logger.info("LED strip cleared.")
            except Exception as e:
                logger.error(f"Error clearing strip: {e}")
        logger.info("Shutdown complete.")
