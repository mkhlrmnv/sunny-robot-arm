#!/usr/bin/python3
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

# Add the absolute path to the 'src' directory to sys.path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.append(src_path)

# Now you can import the cooling module
import cooling

# Start the cooling controller in its own thread.
def start_cooling_thread():
    print("starting cooling")
    controller = cooling.FanController(fan_pin=18, min_temp=20, max_temp=45)
    controller.run(verbal=False, interval=0.1)
    print("started cooling")

print("on own thread")
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
    # Get the absolute path to the directory containing this script
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Build path to the 'src/wsd_control.py' file relative to current script
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    script_path = os.path.join(current_dir, '..', 'src', 'wsd_control.py')
    script_path = os.path.abspath(script_path)  # Resolve to full absolute path
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
        /* Ensure full-page background */
        html, body {
          margin: 0;
          padding: 0;
          height: 100%;
        }
        body {
          background: linear-gradient(to right, #79CED7, #1A56AF);
          display: flex;
        }
        /* Desktop layout: two panels side by side */
        .left-panel {
          width: 33.33%;
          background-color: rgba(51, 51, 51, 0.9);
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
        .center-panel {
          width: 66.66%;
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
          position: relative;
          padding: 20px;
          box-sizing: border-box;
        }
        .logo-container {
          margin-bottom: 10px;
          display: flex;
          justify-content: center;
          width: 100%;
        }
        .logo-container img {
          max-height: 250px;
        }
        .button-container {
          display: flex;
          flex-direction: column;
          align-items: center;
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
        /* Responsive design for mobile */
        @media (max-width: 768px) {
          body {
            flex-direction: column;
          }
          .left-panel {
            display: none;
          }
          .center-panel {
            width: 100%;
          }
          .big-button {
            width: 80%;
            padding: 30px 0;
            font-size: 32px;
            margin: 10px 0;
          }
        }
      </style>
    </head>
    <body>
      <!-- LEFT PANEL: Cooling Info (visible on desktop) -->
      <div class="left-panel">
        <h2>Cooling Info</h2>
        <div class="label">Temperature</div>
        <div class="gauge" id="gaugeTemp" style="--percent: 0%;"></div>
        <div class="label temp-value" id="tempValue">-- °C</div>
        <div class="label">Fan Speed</div>
        <div class="gauge" id="gaugeFan" style="--percent: 0%;"></div>
        <div class="label" id="fanValue">-- %</div>
      </div>
      
      <!-- CENTER PANEL: Logo and Main Buttons -->
      <div class="center-panel">
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
        // Only poll cooling info if the left panel is visible (desktop)
        if (window.innerWidth > 768) {
          function fetchCoolingInfo() {
            fetch('/cooling_info')
              .then(response => response.json())
              .then(data => updateGauges(data))
              .catch(err => console.error('Error fetching cooling info:', err));
          }
          function updateGauges(data) {
            // Assume max temperature for a full gauge is 60°C
            const maxTemp = 60.0;
            let temperature = parseFloat(data.temperature);
            let tempPercent = (temperature / maxTemp) * 100;
            if (tempPercent > 100) tempPercent = 100;
            document.getElementById('gaugeTemp').style.setProperty('--percent', tempPercent + '%');
            document.getElementById('tempValue').innerText = temperature + " °C";
            
            let fanSpeed = parseFloat(data.fan_speed);
            if (fanSpeed > 100) fanSpeed = 100;
            document.getElementById('gaugeFan').style.setProperty('--percent', fanSpeed + '%');
            document.getElementById('fanValue').innerText = fanSpeed + " %";
          }
          fetchCoolingInfo();
          setInterval(fetchCoolingInfo, 1000);
        }
      </script>
    </body>
    </html>
    '''
    return render_template_string(template)

# Endpoint for "INIT MOTORS" – one-off command.
@app.route('/init')
def init_motors():
    template = '''
    <!DOCTYPE html>
    <html>
    <head>
      <title>Init Motors</title>
      <style>
        body {
          font-family: Arial, sans-serif;
          background-color: #f4f4f4;
          margin: 0;
          padding: 20px;
          display: flex;
          flex-direction: column;
          justify-content: center;
          align-items: center;
          height: 100vh;
          text-align: center;
        }
        .button {
          background-color: #4f4f4f;
          border: none;
          color: white;
          padding: 15px 30px;
          font-size: 24px;
          border-radius: 50px;
          cursor: pointer;
          margin-top: 20px;
        }
      </style>
    </head>
    <body>
      <h1>Nothing here yet</h1>
      <button class="button" onclick="location.href='/'">Back</button>
    </body>
    </html>
    '''
    return render_template_string(template)

# Endpoint for "PLAY PATH" – one-off command.
@app.route('/play')
def play_path():
    template = '''
    <!DOCTYPE html>
    <html>
    <head>
      <title>Play Path</title>
      <style>
        body {
          font-family: Arial, sans-serif;
          background-color: #f4f4f4;
          margin: 0;
          padding: 20px;
          display: flex;
          flex-direction: column;
          justify-content: center;
          align-items: center;
          height: 100vh;
          text-align: center;
        }
        .button {
          background-color: #4f4f4f;
          border: none;
          color: white;
          padding: 15px 30px;
          font-size: 24px;
          border-radius: 50px;
          cursor: pointer;
          margin-top: 20px;
        }
      </style>
    </head>
    <body>
      <h1>Nothing here yet</h1>
      <button class="button" onclick="location.href='/'">Back</button>
    </body>
    </html>
    '''
    return render_template_string(template)

@app.route('/manual')
def manual_control():
    global global_wsd_proc
    if global_wsd_proc is None or global_wsd_proc.poll() is not None:
        start_wsd_control()
        time.sleep(1)
        # Flush any initial output from the process.
        while True:
            rlist, _, _ = select.select([global_wsd_proc.stdout], [], [], 0.1)
            if not rlist:
                break
            dummy = global_wsd_proc.stdout.readline()
    template = '''
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="UTF-8">
      <title>Manual Control - J&M Automation</title>
      <style>
        /* Ensure full-page background */
        html, body {
          margin: 0;
          padding: 0;
          height: 100%;
        }
        body {
          background: linear-gradient(to right, #79CED7, #1A56AF);
          display: flex;
        }
        /* LEFT PANEL: Cooling Info (visible on desktop) */
        .left-panel {
          width: 33.33%;
          background-color: rgba(51,51,51,0.9);
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
        /* CENTER PANEL: Manual Control UI */
        .center-panel {
          width: 66.66%;
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
          position: relative;
          padding: 20px;
          box-sizing: border-box;
        }
        .logo-container {
          position: absolute;
          top: 20px;
          left: 50%;
          transform: translateX(-50%);
        }
        .logo-container img {
          max-height: 100px;
        }
        h1 {
          color: #fff;
          margin-top: 100px;
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
        /* Responsive Design for Mobile */
        @media (max-width: 768px) {
          body {
            flex-direction: column;
          }
          .left-panel {
            display: none;
          }
          .center-panel {
            width: 100%;
          }
          .arrow-button {
            width: 100px;
            height: 100px;
            font-size: 40px;
            margin: 10px;
          }
          .small-button {
            width: 60px;
            height: 60px;
            font-size: 28px;
            margin: 5px;
          }
          .big-button {
            width: 80%;
            padding: 30px 0;
            font-size: 32px;
            margin: 10px 0;
          }
        }
      </style>
    </head>
    <body>
      <!-- LEFT PANEL: Cooling Info (desktop only) -->
      <div class="left-panel">
        <h2>Cooling Info</h2>
        <div class="label">Temperature</div>
        <div class="gauge" id="gaugeTemp" style="--percent: 0%;"></div>
        <div class="label temp-value" id="tempValue">-- °C</div>
        <div class="label">Fan Speed</div>
        <div class="gauge" id="gaugeFan" style="--percent: 0%;"></div>
        <div class="label" id="fanValue">-- %</div>
      </div>
      
      <!-- CENTER PANEL: Manual Control UI -->
      <div class="center-panel">
        <div class="logo-container">
          <img src="{{ url_for('static', filename='logo.png') }}" alt="J&M Automation Logo">
        </div>
        <h1>MANUAL CONTROL</h1>
        <div class="motor-row">
          <button class="arrow-button" 
                  onmousedown="startCommandRepeat('w')" 
                  onmouseup="stopCommandRepeat()" 
                  onmouseleave="stopCommandRepeat()"
                  ontouchstart="startCommandRepeat('w')" 
                  ontouchend="stopCommandRepeat()">&larr;</button>
          <div class="motor-info">
            <div class="motor-name">MOTOR 1</div>
          </div>
          <button class="arrow-button" 
                  onmousedown="startCommandRepeat('s')" 
                  onmouseup="stopCommandRepeat()" 
                  onmouseleave="stopCommandRepeat()"
                  ontouchstart="startCommandRepeat('s')" 
                  ontouchend="stopCommandRepeat()">&rarr;</button>
        </div>
        <div class="motor-row">
          <button class="arrow-button" 
                  onmousedown="startCommandRepeat('a')" 
                  onmouseup="stopCommandRepeat()" 
                  onmouseleave="stopCommandRepeat()"
                  ontouchstart="startCommandRepeat('a')" 
                  ontouchend="stopCommandRepeat()">&larr;</button>
          <div class="motor-info">
            <div class="motor-name">MOTOR 2</div>
          </div>
          <button class="arrow-button" 
                  onmousedown="startCommandRepeat('d')" 
                  onmouseup="stopCommandRepeat()" 
                  onmouseleave="stopCommandRepeat()"
                  ontouchstart="startCommandRepeat('d')" 
                  ontouchend="stopCommandRepeat()">&rarr;</button>
        </div>
        <div class="motor-row">
          <button class="arrow-button" 
                  onmousedown="startCommandRepeat('j')" 
                  onmouseup="stopCommandRepeat()" 
                  onmouseleave="stopCommandRepeat()"
                  ontouchstart="startCommandRepeat('j')" 
                  ontouchend="stopCommandRepeat()">&larr;</button>
          <div class="motor-info">
            <div class="motor-name">MOTOR 3</div>
          </div>
          <button class="arrow-button" 
                  onmousedown="startCommandRepeat('k')" 
                  onmouseup="stopCommandRepeat()" 
                  onmouseleave="stopCommandRepeat()"
                  ontouchstart="startCommandRepeat('k')" 
                  ontouchend="stopCommandRepeat()">&rarr;</button>
        </div>
        <div class="step-control">
          <button class="small-button" onclick="sendCommand('o')">-</button>
          <span id="step-size-text">Step Size: 1</span>
          <button class="small-button" onclick="sendCommand('i')">+</button>
        </div>
        <button class="big-button" onclick="location.href='/'">BACK</button>
      </div>
      
      <script>
        // Only poll cooling info on desktop (left panel visible)
        if (window.innerWidth > 768) {
          function fetchCoolingInfo() {
            fetch('/cooling_info')
              .then(response => response.json())
              .then(data => updateGauges(data))
              .catch(err => console.error('Error fetching cooling info:', err));
          }
          function updateGauges(data) {
            const maxTemp = 60.0;
            let temperature = parseFloat(data.temperature);
            let tempPercent = (temperature / maxTemp) * 100;
            if (tempPercent > 100) tempPercent = 100;
            document.getElementById('gaugeTemp').style.setProperty('--percent', tempPercent + '%');
            document.getElementById('tempValue').innerText = temperature + " °C";
            
            let fanSpeed = parseFloat(data.fan_speed);
            if (fanSpeed > 100) fanSpeed = 100;
            document.getElementById('gaugeFan').style.setProperty('--percent', fanSpeed + '%');
            document.getElementById('fanValue').innerText = fanSpeed + " %";
          }
          fetchCoolingInfo();
          setInterval(fetchCoolingInfo, 1000);
        }
      </script>
      
      <!-- New script to enable press and hold functionality -->
      <script>
        var commandInterval;
        function startCommandRepeat(cmd) {
          // Immediately send the command on press
          sendCommand(cmd);
          // Set an interval to repeatedly send the command (adjust interval as needed)
          commandInterval = setInterval(() => sendCommand(cmd), 250);
        }
        function stopCommandRepeat() {
          clearInterval(commandInterval);
        }
      </script>
      
      <script>
        function sendCommand(cmd) {
          fetch('/send_char?cmd=' + cmd)
            .then(response => response.text())
            .then(data => {
              console.log("Response:", data);
              // If the command is for changing the step size, try to extract the new value.
              if(cmd === 'i' || cmd === 'o' || cmd === 'increase_step' || cmd === 'decrease_step'){
                // Example expected output: "Step per key increased to 3"
                // Use a regex to extract the first number found:
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