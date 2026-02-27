from flask import Flask, render_template, request, Response, jsonify
import RPi.GPIO as GPIO
import cv2, threading, time
import smbus

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
try:
    GPIO.cleanup()
except:
    pass

app = Flask(__name__)

# ================= DRIVE (L298N) =================
IN1, IN2 = 27, 17
IN3, IN4 = 22, 23
ENA, ENB = 12, 13

# ================= LIFT =================
LIFT_UP = 15
LIFT_DOWN = 14

# ================= METAL + WET SENSOR =================
METAL_SENSOR_PIN = 16   # LOW = metal
WET_SENSOR_PIN   = 20   # LOW = wet

# ================= ULTRASONIC =================
BINS = {
    "dry":   {"trig": 24, "echo": 25},
    "wet":   {"trig": 8,  "echo": 7},
    "metal": {"trig": 19, "echo": 26}
}

BIN_EMPTY_CM = 28.0
BIN_FULL_CM  = 5.0

# ================= PCA9685 =================
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

I2C_BUS = 1
PCA9685_ADDR = 0x40
bus = smbus.SMBus(I2C_BUS)

SERVO_MIN = 150
SERVO_MAX = 600

# Servo defaults + limits
SERVO_LIMITS = {
    0: (0, 140, 140),
    1: (30, 150, 30),
    2: (0, 90, 0),
    3: (0, 140, 70),
    4: (0, 90, 90),
    5: (0, 90, 0),
}

# ================= GLOBALS =================
pwm_a = None
pwm_b = None
current_speed = 70
latest_frame = None
frame_lock = threading.Lock()

# ================= PCA9685 LOW LEVEL =================
def write_byte(reg, value):
    bus.write_byte_data(PCA9685_ADDR, reg, value)

def read_byte(reg):
    return bus.read_byte_data(PCA9685_ADDR, reg)

def init_pca9685():
    write_byte(MODE1, 0x00)
    write_byte(MODE2, 0x04)
    time.sleep(0.01)

def set_pwm_freq(freq_hz):
    prescaleval = 25000000.0
    prescaleval /= 4096.0
    prescaleval /= float(freq_hz)
    prescaleval -= 1.0
    prescale = int(prescaleval + 0.5)

    oldmode = read_byte(MODE1)
    sleepmode = (oldmode & 0x7F) | 0x10
    write_byte(MODE1, sleepmode)
    write_byte(PRESCALE, prescale)
    write_byte(MODE1, oldmode)
    time.sleep(0.005)
    write_byte(MODE1, oldmode | 0x80)

def set_pwm(channel, on, off):
    base = LED0_ON_L + 4 * channel
    write_byte(base, on & 0xFF)
    write_byte(base + 1, on >> 8)
    write_byte(base + 2, off & 0xFF)
    write_byte(base + 3, off >> 8)

def angle_to_pwm(angle):
    return int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))

def set_servo_angle(channel, angle):
    min_a, max_a, _ = SERVO_LIMITS[channel]
    angle = max(min_a, min(max_a, angle))
    pwm = angle_to_pwm(angle)
    set_pwm(channel, 0, pwm)
    print(f"🦾 Servo {channel} → {angle}°")

# ================= GPIO INIT =================
def init_gpio():
    global pwm_a, pwm_b

    GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB, LIFT_UP, LIFT_DOWN], GPIO.OUT)
    GPIO.setup(METAL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(WET_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    for b in BINS.values():
        GPIO.setup(b["trig"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(b["echo"], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    pwm_a = GPIO.PWM(ENA, 1000)
    pwm_b = GPIO.PWM(ENB, 1000)
    pwm_a.start(current_speed)
    pwm_b.start(current_speed)

    init_pca9685()
    set_pwm_freq(50)

    # Default servo positions
    for ch, (_, _, d) in SERVO_LIMITS.items():
        set_servo_angle(ch, d)

    print("✅ GPIO + PCA9685 initialized")

# ================= DRIVE =================
def stop_drive():
    GPIO.output([IN1, IN2, IN3, IN4], 0)

def safe_drive(fn):
    stop_drive()
    time.sleep(0.12)
    fn()

def forward():
    GPIO.output(IN1,0); GPIO.output(IN2,1)
    GPIO.output(IN3,0); GPIO.output(IN4,1)

def backward():
    GPIO.output(IN1,1); GPIO.output(IN2,0)
    GPIO.output(IN3,1); GPIO.output(IN4,0)

def right():
    GPIO.output(IN1,1); GPIO.output(IN2,0)
    GPIO.output(IN3,0); GPIO.output(IN4,1)

def left():
    GPIO.output(IN1,0); GPIO.output(IN2,1)
    GPIO.output(IN3,1); GPIO.output(IN4,0)

# ================= SPEED =================
def set_speed(val):
    global current_speed
    current_speed = int(val)
    pwm_a.ChangeDutyCycle(current_speed)
    pwm_b.ChangeDutyCycle(current_speed)

# ================= LIFT =================
def lift_stop():
    GPIO.output(LIFT_UP,0); GPIO.output(LIFT_DOWN,0)

def lift_up():
    lift_stop(); time.sleep(0.1)
    GPIO.output(LIFT_UP,1)

def lift_down():
    lift_stop(); time.sleep(0.1)
    GPIO.output(LIFT_DOWN,1)

# ================= SENSORS =================
def is_metal_detected():
    val = GPIO.input(METAL_SENSOR_PIN)
    if val == GPIO.LOW:
        print("🧲 METAL DETECTED")
    return val == GPIO.LOW

def is_wet_detected():
    val = GPIO.input(WET_SENSOR_PIN)
    if val == GPIO.LOW:
        print("💧 WET DETECTED")
    return val == GPIO.LOW

# ================= ULTRASONIC =================
def read_distance(trig, echo, timeout=0.03):
    GPIO.output(trig, 0)
    time.sleep(0.0002)
    GPIO.output(trig, 1)
    time.sleep(0.00001)
    GPIO.output(trig, 0)

    start = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - start > timeout:
            return None

    pulse_start = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - pulse_start > timeout:
            return None

    pulse_end = time.time()
    return round((pulse_end - pulse_start) * 17150, 1)

def level_percent(d):
    if d is None: return 0
    if d >= BIN_EMPTY_CM: return 0
    if d <= BIN_FULL_CM: return 100
    return int(((BIN_EMPTY_CM - d)/(BIN_EMPTY_CM - BIN_FULL_CM))*100)

# ================= CAMERA =================
def camera_loop():
    global latest_frame
    cap = cv2.VideoCapture(0)
    print("📷 Camera started")

    while True:
        ret, frame = cap.read()
        if ret:
            h,w = frame.shape[:2]
            cx,cy = w//2, h//2
            box = 120
            cv2.rectangle(frame,(cx-box//2,cy-box//2),(cx+box//2,cy+box//2),(0,255,0),2)

            txt = "METAL" if is_metal_detected() else "NON-METAL"
            cv2.putText(frame, txt, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)

            with frame_lock:
                latest_frame = frame
        time.sleep(0.02)

def generate_frames():
    while True:
        with frame_lock:
            if latest_frame is None: continue
            ret, buf = cv2.imencode('.jpg', latest_frame)
        if ret:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

# ================= START =================
init_gpio()
threading.Thread(target=camera_loop, daemon=True).start()

# ================= ROUTES =================
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/move", methods=["POST"])
def move():
    {"forward":lambda:safe_drive(forward),
     "backward":lambda:safe_drive(backward),
     "left":lambda:safe_drive(left),
     "right":lambda:safe_drive(right),
     "stop":stop_drive}.get(request.form.get("cmd"), stop_drive)()
    return ("",204)

@app.route("/speed", methods=["POST"])
def speed():
    set_speed(request.form.get("speed",70))
    return ("",204)

@app.route("/lift", methods=["POST"])
def lift():
    {"up":lift_up,"down":lift_down}.get(request.form.get("cmd"), lift_stop)()
    return ("",204)

@app.route("/servo", methods=["POST"])
def servo():
    ch = int(request.form.get("channel"))
    ang = int(request.form.get("angle"))
    set_servo_angle(ch, ang)
    return ("",204)

@app.route("/bin_levels")
def bin_levels():
    levels = {}
    for k,b in BINS.items():
        levels[k] = level_percent(read_distance(b["trig"], b["echo"]))
    return jsonify(levels)

@app.route("/sensor_status")
def sensor_status():
    return jsonify({
        "metal": is_metal_detected(),
        "wet": is_wet_detected()
    })

def start_web_server():
    app.run(host="0.0.0.0", port=5000, threaded=True)
