import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ===== SENSOR PINS =====
WET_SENSOR_PIN   = 20   # LOW = wet detected
METAL_SENSOR_PIN = 21  # LOW = metal detected

# ===== GPIO SETUP (ONCE ONLY) =====
GPIO.setup(WET_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(METAL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

_last_wet = None
_last_metal = None
_last_raw_print = 0

# ===== PUBLIC API =====
def is_wet_detected():
    try:
        return GPIO.input(WET_SENSOR_PIN) == GPIO.LOW
    except RuntimeError:
        return False


def is_metal_detected():
    try:
        return GPIO.input(METAL_SENSOR_PIN) == GPIO.LOW
    except RuntimeError:
        return False


def debug_print():
    """Print RAW + state-change messages (like your test code)"""
    global _last_raw_print, _last_wet, _last_metal
    now = time.time()

    wet = is_wet_detected()
    metal = is_metal_detected()

    # raw GPIO print every 1s
    if now - _last_raw_print >= 1.0:
        print(f"[RAW] Wet GPIO={GPIO.input(WET_SENSOR_PIN)}  Metal GPIO={GPIO.input(METAL_SENSOR_PIN)}")
        _last_raw_print = now

    # state-change prints
    if wet != _last_wet:
        print("💧 WET DETECTED" if wet else "💧 WET CLEARED")
        _last_wet = wet

    if metal != _last_metal:
        print("🧲 METAL DETECTED" if metal else "🧲 METAL CLEARED")
        _last_metal = metal

    return wet, metal
