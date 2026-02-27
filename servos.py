# servos.py
import smbus
import time

# ================= PCA9685 REGISTERS =================
MODE1       = 0x00
MODE2       = 0x01
PRESCALE    = 0xFE
LED0_ON_L   = 0x06

# ================= I2C CONFIG =================
I2C_BUS = 1
PCA9685_ADDR = 0x40   # change if needed

bus = smbus.SMBus(I2C_BUS)

# ================= SERVO SETTINGS =================
SERVO_MIN = 0   # ~0 degree
SERVO_MAX = 180   # ~180 degree

# ================= DEFAULT POSITIONS =================
DEFAULTS = {
    0: 140,   # S0 Garbage type detector
    1: 30,    # S1 Pick bucket (closed)
    2: 0,     # S2 Dry dustbin (closed)
    3: 70,    # S3 Sorting gate (center)
    4: 90,    # S4 Metal dustbin (closed)
    5: 0      # S5 Dry dustbin (closed)
}

# ================= LOW LEVEL I2C =================
def write_byte(reg, value):
    bus.write_byte_data(PCA9685_ADDR, reg, value)

def read_byte(reg):
    return bus.read_byte_data(PCA9685_ADDR, reg)

# ================= PCA9685 INIT =================
def init_pca9685():
    write_byte(MODE1, 0x00)   # normal mode
    write_byte(MODE2, 0x04)   # totem pole
    time.sleep(0.01)

def set_pwm_freq(freq_hz):
    prescaleval = 25000000.0
    prescaleval /= 4096.0
    prescaleval /= float(freq_hz)
    prescaleval -= 1.0
    prescale = int(prescaleval + 0.5)

    oldmode = read_byte(MODE1)
    sleepmode = (oldmode & 0x7F) | 0x10   # sleep
    write_byte(MODE1, sleepmode)
    write_byte(PRESCALE, prescale)
    write_byte(MODE1, oldmode)
    time.sleep(0.005)
    write_byte(MODE1, oldmode | 0x80)    # restart

def set_pwm(channel, on, off):
    base = LED0_ON_L + 4 * channel
    write_byte(base, on & 0xFF)
    write_byte(base + 1, on >> 8)
    write_byte(base + 2, off & 0xFF)
    write_byte(base + 3, off >> 8)

# ================= SERVO CORE =================
def angle_to_pwm(angle):
    pulse = SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN)
    return int(pulse)

def set_servo_angle(channel, angle):
    angle = max(0, min(180, angle))
    pwm = angle_to_pwm(angle)
    set_pwm(channel, 0, pwm)

# ================= LIMIT ENFORCEMENT =================
def clamp_angle(channel, angle):
    limits = {
        0: (0, 140),     # S0
        1: (30, 150),    # S1
        2: (0, 90),      # S2
        3: (0, 140),     # S3 (0 / 70 / 140)
        4: (0, 90),      # S4
        5: (0, 90)       # S5
    }

    if channel in limits:
        lo, hi = limits[channel]
        return max(lo, min(hi, angle))

    return angle

# ================= HIGH LEVEL API =================
_initialized = False

def init_servos():
    global _initialized
    if _initialized:
        return

    print("🧠 Initializing PCA9685 (servos)...")
    init_pca9685()
    set_pwm_freq(50)
    time.sleep(0.5)

    # Move all servos to defaults
    for ch, ang in DEFAULTS.items():
        set_servo_angle(ch, ang)
        time.sleep(0.1)

    _initialized = True
    print("✅ Servos initialized to default positions")

def move_servo(channel, angle):
    angle = clamp_angle(channel, angle)
    set_servo_angle(channel, angle)
