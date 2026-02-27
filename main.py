import web_control
import time

print("🌐 Web control + live camera started (browser only)")

# Start Flask server (camera handled inside web_control)
web_control.start_web_server()

# Keep program alive
while True:
    time.sleep(1)
