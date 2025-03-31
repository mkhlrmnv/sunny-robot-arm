from flask import Flask, render_template_string, request, jsonify
import subprocess
import time
import select
import os
import sys
import threading

app = Flask(__name__)

# Global variable for the interactive (manual control) session.
global_wsd_proc = None

# Add the src folder to the Python path and import the cooling module.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
import cooling

# Start the cooling controller in its own thread.
def start_cooling_thread():
    controller = cooling.FanController(fan_pin=18, min_temp=20, max_temp=45)
    controller.run(verbal=False, interval=1)

cooling_thread = threading.Thread(target=start_cooling_thread, daemon=True)
cooling_thread.start()

# Endpoint to serve the latest cooling reading as JSON.
@app.route('/cooling_info')
def cooling_info():
    try:
        reading = cooling.latest_reading
    except AttributeError:
        reading = {"temperature": "--", "fan_speed": "--"}
    print(reading)
    return jsonify(reading)

def start_wsd_control():
    global global_wsd_proc
    print("Starting local wsd_control process...")
    # Build absolute path to your script in the src folder.
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    script_path = os.path.join(base_dir, 'wsd_control.py')
    # Use unbuffered mode with -u flag.
    command = f"python3 -u {script_path}"
    
    try:
        global_wsd_proc = subprocess.Popen(
            command, shell=True,
            cwd=base_dir,  # Set working directory to the src folder.
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        # Set stdout to non-blocking mode.
        os.set_blocking(global_wsd_proc.stdout.fileno(), False)
    except Exception as e:
        print("Error starting process:", e)
        return

    time.sleep(1)
    if global_wsd_proc.poll() is not None:
        # Process has terminated; capture and print error output.
        err = global_wsd_proc.stderr.read()
        print("wsd_control.py did not start successfully.")
        print("Error output:", err)
    else:
        print("wsd_control.py started successfully.")


def run_non_interactive_command(cmd):
    """Execute a one-off command locally and return its output and error."""
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return result.stdout, result.stderr

@app.route('/')
def index():
    template = '''
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="UTF-8" />
      <title>J&M Automation</title>
      <style>
        body {
          margin: 0;
          font-family: Arial, sans-serif;
          height: 100vh;
          display: flex;
        }
        /* Left Panel Styles */
        .left-panel {
          width: 33.33%;
          background-color: #333;
          color: #fff;
          padding: 20px;
          box-sizing: border-box;
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
        }
        .left-panel h2 {
          margin-bottom: 20px;
        }
        .label {
          margin: 10px 0 5px;
          font-size: 18px;
          text-align: center;
        }
        .temp-value {
          margin-bottom: 20px;
          font-size: 20px;
        }
        .gauge {
          width: 120px;
          height: 120px;
          border-radius: 50%;
          background: conic-gradient(#3498db 0%, #3498db var(--percent), #e0e0e0 var(--percent), #e0e0e0 100%);
          margin-bottom: 5px;
        }

        /* Right Panel Styles */
        .right-panel {
          width: 66.66%;
          background: linear-gradient(to right, #79CED7, #1A56AF);
          display: flex;
          flex-direction: column;
          align-items: center; /* center horizontally */
        }
        .logo-container {
          margin-top: 40px; /* space from the top */
          display: flex;
          justify-content: center;
          width: 100%;
        }
        .logo-container img {
          max-height: 200px; /* adjust as needed */
        }
        .button-container {
          display: flex;
          flex-direction: column; /* stack buttons vertically */
          align-items: center;    /* center them horizontally */
          margin-top: 50px;
        }
        .big-button {
          background-color: #4f4f4f;
          color: #fff;
          border: none;
          border-radius: 50px;
          padding: 20px 40px;
          margin: 20px;
          font-size: 24px;
          cursor: pointer;
          box-shadow: 0 4px 8px rgba(0,0,0,0.2);
          transition: background-color 0.3s ease;
        }
        .big-button:hover {
          background-color: #5f5f5f;
        }
      </style>
    </head>
    <body>
      <!-- LEFT PANEL: Gauges for Temperature and Fan Speed -->
      <div class="left-panel">
        <h2>Cooling Info</h2>
        <!-- Temperature Gauge -->
        <div class="label">Temperature</div>
        <div class="gauge" id="gaugeTemp" style="--percent: 0%;"></div>
        <div class="label temp-value" id="tempValue">-- °C</div>

        <!-- Fan Speed Gauge -->
        <div class="label">Fan Speed</div>
        <div class="gauge" id="gaugeFan" style="--percent: 0%;"></div>
        <div class="label" id="fanValue">-- %</div>
      </div>

      <!-- RIGHT PANEL: Logo + Main Buttons -->
      <div class="right-panel">
        <div class="logo-container">
          <img src="{{ url_for('static', filename='logo.png') }}" alt="J&M Automation Logo">
        </div>
        <div class="button-container">
          <button class="big-button" onclick="location.href='/init'">INIT MOTORS</button>
          <button class="big-button" onclick="location.href='/play'">PLAY PATH</button>
          <button class="big-button" onclick="location.href='/manual'">MANUAL CONTROL</button>
        </div>
      </div>

      <script>
        // Poll /cooling_info every second to update the gauges
        function fetchCoolingInfo() {
          fetch('/cooling_info')
            .then(response => response.json())
            .then(data => updateGauges(data))
            .catch(err => console.error('Error fetching cooling info:', err));
        }
        function updateGauges(data) {
          // Assume max temperature for full gauge fill is 60°C
          const maxTemp = 60.0;
          let temperature = parseFloat(data.temperature);
          let tempPercent = (temperature / maxTemp) * 100;
          if (tempPercent > 100) tempPercent = 100;

          // Update Temperature Gauge
          document.getElementById('gaugeTemp').style.setProperty('--percent', tempPercent + '%');
          document.getElementById('tempValue').innerText = temperature + " °C";

          // Assume fan_speed is already a percentage
          let fanSpeed = parseFloat(data.fan_speed);
          if (fanSpeed > 100) fanSpeed = 100;

          // Update Fan Speed Gauge
          document.getElementById('gaugeFan').style.setProperty('--percent', fanSpeed + '%');
          document.getElementById('fanValue').innerText = fanSpeed + " %";
        }
        fetchCoolingInfo();
        setInterval(fetchCoolingInfo, 1000);
      </script>
    </body>
    </html>
    '''
    return render_template_string(template)

# Endpoint for "INIT MOTORS" – one-off command.
@app.route('/init')
def init_motors():
    # Change the path to point to the init script in your src folder
    output, error = run_non_interactive_command('python ../src/init.py')
    template = '''
    <!DOCTYPE html>
    <html>
    <head>
      <title>Init Motors</title>
      <style>
        body { font-family: Arial, sans-serif; background-color: #f4f4f4; padding: 20px; }
        .button { background-color: #4f4f4f; border: none; color: white; padding: 10px 20px;
                  margin-top: 20px; font-size: 18px; border-radius: 5px; cursor: pointer; }
        pre { background-color: #eee; padding: 10px; border-radius: 5px; }
      </style>
    </head>
    <body>
      <h1>Init Motors Result</h1>
      <pre>{{ output }}</pre>
      <pre>{{ error }}</pre>
      <button class="button" onclick="location.href='/'">Back</button>
    </body>
    </html>
    '''
    return render_template_string(template, output=output, error=error)

# Endpoint for "PLAY PATH" – one-off command.
@app.route('/play')
def play_path():
    # Update the command to point to your play script in the src folder.
    output, error = run_non_interactive_command('python ../src/play.py')
    template = '''
    <!DOCTYPE html>
    <html>
    <head>
      <title>Play Path</title>
      <style>
        body { font-family: Arial, sans-serif; background-color: #f4f4f4; padding: 20px; }
        .button { background-color: #4f4f4f; border: none; color: white; padding: 10px 20px;
                  margin-top: 20px; font-size: 18px; border-radius: 5px; cursor: pointer; }
        pre { background-color: #eee; padding: 10px; border-radius: 5px; }
      </style>
    </head>
    <body>
      <h1>Play Path Result</h1>
      <pre>{{ output }}</pre>
      <pre>{{ error }}</pre>
      <button class="button" onclick="location.href='/'">Back</button>
    </body>
    </html>
    '''
    return render_template_string(template, output=output, error=error)

# Manual Control page.
@app.route('/manual')
def manual_control():
    global global_wsd_proc
    if global_wsd_proc is None:
        start_wsd_control()
        # Wait briefly to allow the interactive script to start.
        time.sleep(1)
        # Optionally flush any initial output from the process.
        while True:
            rlist, _, _ = select.select([global_wsd_proc.stdout], [], [], 0.1)
            if not rlist:
                break
            dummy = global_wsd_proc.stdout.readline()
    template = '''
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="UTF-8" />
      <title>Manual Control</title>
      <style>
        body {
          margin: 0;
          font-family: Arial, sans-serif;
          background: linear-gradient(to right, #79CED7, #1A56AF);
          min-height: 100vh;
          display: flex;
          flex-direction: column;
          align-items: center;
        }
        .logo-container {
          position: absolute;
          top: 20px;
          left: 20px;
          display: flex;
          align-items: center;
        }
        .logo-container img {
          height: 300px;
          margin-right: 10px;
        }
        .logo-container span {
          color: #fff;
          font-size: 1.2rem;
          font-weight: bold;
        }
        h1 {
          color: #fff;
          margin-top: 80px;
          margin-bottom: 40px;
        }
        .motor-row {
          display: flex;
          align-items: center;
          justify-content: center;
          margin: 20px 0;
        }
        .arrow-button {
          background-color: #4f4f4f;
          color: #fff;
          border: none;
          border-radius: 50px;
          width: 80px;
          height: 80px;
          font-size: 36px;
          cursor: pointer;
          margin: 0 20px;
          box-shadow: 0 4px 8px rgba(0,0,0,0.2);
          transition: background-color 0.3s ease;
        }
        .arrow-button:hover {
          background-color: #5f5f5f;
        }
        .motor-info {
          display: flex;
          flex-direction: column;
          align-items: center;
          color: #fff;
          font-size: 1.2rem;
        }
        .motor-info .motor-name {
          font-weight: bold;
          font-size: 1.4rem;
          margin-bottom: 5px;
        }
        .step-control {
          display: flex;
          align-items: center;
          justify-content: center;
          margin: 20px;
          color: #fff;
          font-size: 20px;
        }
        .small-button {
          background-color: #4f4f4f;
          border: none;
          color: #fff;
          border-radius: 50px;
          width: 50px;
          height: 50px;
          font-size: 24px;
          cursor: pointer;
          margin: 0 10px;
          box-shadow: 0 4px 8px rgba(0,0,0,0.2);
          transition: background-color 0.3s ease;
        }
        .small-button:hover {
          background-color: #5f5f5f;
        }
        .big-button {
          background-color: #4f4f4f;
          color: #fff;
          border: none;
          border-radius: 50px;
          padding: 15px 30px;
          margin: 40px;
          font-size: 24px;
          cursor: pointer;
          box-shadow: 0 4px 8px rgba(0,0,0,0.2);
          transition: background-color 0.3s ease;
        }
        .big-button:hover {
          background-color: #5f5f5f;
        }
      </style>
    </head>
    <body>
      <!-- Logo -->
      <div class="logo-container">
        <img src="{{ url_for('static', filename='logo.png') }}" alt="J&M Automation Logo">
      </div>
      <h1>MANUAL CONTROL</h1>
      <!-- Motor Control Rows -->
      <div class="motor-row">
        <button class="arrow-button" onclick="sendCommand('w')">&larr;</button>
        <div class="motor-info">
          <div class="motor-name">MOTOR 1</div>
        </div>
        <button class="arrow-button" onclick="sendCommand('s')">&rarr;</button>
      </div>
      <div class="motor-row">
        <button class="arrow-button" onclick="sendCommand('a')">&larr;</button>
        <div class="motor-info">
          <div class="motor-name">MOTOR 2</div>
        </div>
        <button class="arrow-button" onclick="sendCommand('d')">&rarr;</button>
      </div>
      <div class="motor-row">
        <button class="arrow-button" onclick="sendCommand('j')">&larr;</button>
        <div class="motor-info">
          <div class="motor-name">MOTOR 3</div>
        </div>
        <button class="arrow-button" onclick="sendCommand('k')">&rarr;</button>
      </div>
      <!-- Step Size Control -->
      <div class="step-control">
        <button class="small-button" onclick="sendCommand('o')">-</button>
        <span id="step-size-text">Step Size: 1</span>
        <button class="small-button" onclick="sendCommand('i')">+</button>
      </div>
      <button class="big-button" onclick="location.href='/'">BACK</button>
      <script>
        function sendCommand(cmd) {
          fetch('/send_char?cmd=' + cmd)
            .then(response => response.text())
            .then(data => {
              console.log("Response:", data);
              if(cmd === 'i' || cmd === 'o' || cmd === 'increase_step' || cmd === 'decrease_step'){
                const match = data.match(/(\\d+)/);
                if (match && match[1]) {
                  document.getElementById('step-size-text').innerText = "Step Size: " + match[1];
                }
              }
            });
        }
      </script>
    </body>
    </html>
    '''
    return render_template_string(template)


@app.route('/send_char')
def send_char():
    global global_wsd_proc
    cmd = request.args.get('cmd')
    # If process isn't started or has terminated, restart it.
    if global_wsd_proc is None or global_wsd_proc.poll() is not None:
        print("Interactive process not active. Restarting...")
        start_wsd_control()
        time.sleep(1)
    
    try:
        print("Sending command:", cmd)
        global_wsd_proc.stdin.write(cmd)
        global_wsd_proc.stdin.flush()
    except BrokenPipeError:
        print("Broken pipe detected. Restarting interactive process.")
        start_wsd_control()
        return "Interactive process restarted due to broken pipe."
    
    # Allow a brief moment for the process to respond.
    time.sleep(0.1)
    output = ""
    # Use select to check if there's data to read.
    rlist, _, _ = select.select([global_wsd_proc.stdout], [], [], 0.1)
    if rlist:
        # Read up to 1024 bytes to avoid blocking indefinitely.
        output = global_wsd_proc.stdout.read(1024)
    if output:
        print("Process output:", output)
        return output
    return "No output from process"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)