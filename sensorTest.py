import RPi.GPIO as GPIO
import time

# ===== GPIO PINS =====
WET_SENSOR_PIN   = 20   # LOW = wet detected
METAL_SENSOR_PIN = 16   # LOW = metal detected

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Pull-ups because sensor shorts to GND when detected
GPIO.setup(WET_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(METAL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("✅ Wet + Metal Sensor Test Started")
print("Touch wet sensor with water / finger")
print("Bring metal near metal sensor")
print("Press CTRL+C to exit\n")

last_wet = None
last_metal = None

try:
    while True:
        wet_raw   = GPIO.input(WET_SENSOR_PIN)
        metal_raw = GPIO.input(METAL_SENSOR_PIN)

        wet_detected   = (wet_raw == GPIO.LOW)
        metal_detected = (metal_raw == GPIO.LOW)

        # ---- RAW DEBUG ----
        print(f"[RAW] Wet GPIO={wet_raw}  Metal GPIO={metal_raw}")

        # ---- STATE CHANGE PRINTS ----
        if wet_detected != last_wet:
            if wet_detected:
                print("💧 WET DETECTED")
            else:
                print("💧 WET CLEARED (Dry again)")
            last_wet = wet_detected

        if metal_detected != last_metal:
            if metal_detected:
                print("🧲 METAL DETECTED")
            else:
                print("🧲 METAL CLEARED (No metal)")
            last_metal = metal_detected

        time.sleep(1.0)

except KeyboardInterrupt:
    print("\n🛑 Exiting test")

finally:
    GPIO.cleanup()
    print("🧹 GPIO cleaned up")
