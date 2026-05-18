#!/usr/bin/env python3
import time
import sys
import threading
import termios
import tty
import select
from flask import Flask, jsonify, render_template_string

from pid2m import (
    MSP, DroneController, 
    SERIAL_PORT, BAUD_RATE, 
    CH_THROTTLE, RC_LOW, RC_LOOP_HZ
)

# Global variables to share data between the main control loop and Flask
sensor_data = {
    "throttle": 1000,
    "altitude": 0.0,
    "imu": {}
}

app = Flask(__name__)

# Premium looking UI with Flask template
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Drone Sensor Interface</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        :root {
            --bg: #0f172a;
            --panel: #1e293b;
            --text: #f8fafc;
            --accent: #38bdf8;
        }
        body { 
            font-family: 'Inter', sans-serif; 
            background-color: var(--bg); 
            color: var(--text);
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
        }
        .card { 
            background: var(--panel); 
            padding: 30px; 
            border-radius: 12px; 
            box-shadow: 0 10px 25px rgba(0,0,0,0.5); 
            width: 100%;
            max-width: 500px; 
        }
        h1 { 
            text-align: center; 
            color: var(--accent);
            margin-top: 0;
        }
        .data-row {
            display: flex;
            justify-content: space-between;
            padding: 15px 0;
            border-bottom: 1px solid #334155;
            font-size: 1.2rem;
        }
        .data-row:last-child {
            border-bottom: none;
        }
        .value {
            font-weight: bold;
            color: var(--accent);
        }
        pre {
            background: #0f172a;
            padding: 15px;
            border-radius: 8px;
            overflow-x: auto;
            color: #a5b4fc;
        }
    </style>
    <script>
        async function updateData() {
            try {
                const response = await fetch('/api/data');
                const data = await response.json();
                document.getElementById('throttle-val').innerText = data.throttle;
                document.getElementById('alt-val').innerText = data.altitude.toFixed(1) + ' cm';
                document.getElementById('imu-val').innerText = JSON.stringify(data.imu, null, 2);
            } catch (error) {
                console.error("Error fetching data:", error);
            }
        }
        setInterval(updateData, 100); // 10 times per second update for smooth visual
    </script>
</head>
<body>
    <div class="card">
        <h1>Drone Live Telemetry</h1>
        <div class="data-row">
            <span>Throttle Channel</span>
            <span class="value" id="throttle-val">Loading...</span>
        </div>
        <div class="data-row">
            <span>Altitude</span>
            <span class="value" id="alt-val">Loading...</span>
        </div>
        <div>
            <p style="margin-bottom: 5px; color: #94a3b8;">IMU Sensor Data:</p>
            <pre id="imu-val">Loading...</pre>
        </div>
    </div>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/api/data')
def api_data():
    return jsonify(sensor_data)

def run_flask():
    import logging
    # Disable flask default logging so it doesn't interrupt the terminal
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def get_key():
    """Non-blocking function to read a single character from standard input."""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    global sensor_data

    current_throttle = 1000

    # Start Flask server in a background thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    print(f"🔌 Connecting to FC on {SERIAL_PORT} @ {BAUD_RATE} ...")
    try:
        msp = MSP(SERIAL_PORT, BAUD_RATE)
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        return

    drone = DroneController(msp)

    print("\n" + "="*50)
    print("🚀 Drone Control Initialized (WASD Mode).")
    print("🌐 Web interface available at: http://localhost:5000")
    print("="*50)
    print("🎮 Controls:")
    print("   [W] : Increase Throttle (+10)")
    print("   [S] : Decrease Throttle (-10)")
    print("   [Q] : Quit / Emergency Stop")
    print("="*50 + "\n")

    print("⚠️  Ensure the drone is ARMED if you want motors to spin.\n")

    interval = 1.0 / RC_LOOP_HZ

    try:
        while True:
            start_loop = time.time()
            
            # Read keyboard input
            key = get_key()
            if key:
                if key.lower() == 'w':
                    current_throttle = min(2000, current_throttle + 10)
                elif key.lower() == 's':
                    current_throttle = max(1000, current_throttle - 10)
                elif key.lower() == 'q':
                    print("\n🛑 Quit requested. Stopping motors and exiting...")
                    break
                    
            drone.rc[CH_THROTTLE] = current_throttle
            drone._tx()

            # Read sensors
            alt, _ = drone.msp.get_altitude()
            imu = drone.msp.get_imu()
            
            alt_val = alt if alt is not None else 0.0

            # Update global sensor data for Flask
            sensor_data["throttle"] = current_throttle
            sensor_data["altitude"] = alt_val
            sensor_data["imu"] = imu if imu else {}

            # Print to terminal
            # Use carriage return `\r` to overwrite the same line continuously
            sys.stdout.write(f"\r[ Terminal ] Throttle: {current_throttle:4d} | Alt: {alt_val:5.1f} cm | IMU: {str(imu)[:40]}...     ")
            sys.stdout.flush()

            # Maintain loop rate (typically 50Hz as per RC_LOOP_HZ)
            elapsed = time.time() - start_loop
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n🛑 Exiting due to KeyboardInterrupt...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        print("\n⬇️  Shutting down safely...")
        # Ensure motors stop on exit
        drone.rc[CH_THROTTLE] = RC_LOW
        drone._tx_for(1.0)
        drone.disarm()
        msp.close()
        print("🔌 Serial port closed.")

if __name__ == '__main__':
    main()
