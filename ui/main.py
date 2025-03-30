from flask import Flask, render_template_string, redirect, url_for
import paramiko

app = Flask(__name__)

# SSH connection settings (adjust these to your environment)
SSH_HOST = '192.168.176.137'
SSH_PORT = 22
SSH_USER = 'mkhl'

# Global variables for the interactive (manual control) session.
global_ssh_client = None
global_ssh_channel = None

def start_wsd_control():
    global global_ssh_client, global_ssh_channel
    print("Starting SSH connection to {}...".format(SSH_HOST))
    global_ssh_client = paramiko.SSHClient()
    global_ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    global_ssh_client.connect(SSH_HOST, port=SSH_PORT, username=SSH_USER)
    global_ssh_channel = global_ssh_client.invoke_shell()
    print("SSH channel created.")
    # Start the interactive control script on the Pi.
    global_ssh_channel.send('python3 /home/mkhl/Desktop/wsd_control.py\n')
    print("Interactive wsd_control.py started.")

def run_non_interactive_command(cmd):
    """Execute a one-off command via SSH and return its output and error."""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(SSH_HOST, port=SSH_PORT, username=SSH_USER)
    stdin, stdout, stderr = ssh.exec_command(cmd)
    output = stdout.read().decode()
    error = stderr.read().decode()
    ssh.close()
    return output, error

# Home page with three big buttons.
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
          background: linear-gradient(to right, #79CED7, #1A56AF);
          height: 100vh;
          display: flex;
          justify-content: center;
          align-items: center;
        }
        .logo-container {
          position: absolute
          top: 20px;
          left: 20px;
          display: flex;
          align-items: center;
        }
        .logo-container img {
          height: 400px;
          margin-right: 10px;
        }
        .logo-container span {
          color: #fff;
          font-size: 1.2rem;
          font-weight: bold;
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
      </style>
    </head>
    <body>
      <!-- Logo in top-left corner -->
      <div class="logo-container">
        <img src="{{ url_for('static', filename='logo.png') }}" alt="J&M Automation Logo">
      </div>

      <!-- Centered Buttons -->
      <div class="button-container">
        <button class="big-button" onclick="location.href='/init'">INIT MOTORS</button>
        <button class="big-button" onclick="location.href='/play'">PLAY PATH</button>
        <button class="big-button" onclick="location.href='/manual'">MANUAL CONTROL</button>
      </div>
    </body>
    </html>
    '''
    return render_template_string(template)

# Endpoint for "INIT MOTORS" – one-off command.
@app.route('/init')
def init_motors():
    output, error = run_non_interactive_command('python3 /home/mkhl/Desktop/init_motors.py')
    template = '''
    <!DOCTYPE html>
    <html>
    <head>
      <title>Init Motors</title>
      <style>
        body { font-family: Arial, sans-serif; background-color: #f4f4f4; padding: 20px; }
        .button { background-color: #4f4f4f; border: none; color: white; padding: 10px 20px; margin-top: 20px; font-size: 18px; border-radius: 5px; cursor: pointer; }
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
    output, error = run_non_interactive_command('python3 /home/mkhl/Desktop/execute_path.py')
    template = '''
    <!DOCTYPE html>
    <html>
    <head>
      <title>Play Path</title>
      <style>
        body { font-family: Arial, sans-serif; background-color: #f4f4f4; padding: 20px; }
        .button { background-color: #4f4f4f; border: none; color: white; padding: 10px 20px; margin-top: 20px; font-size: 18px; border-radius: 5px; cursor: pointer; }
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
    global global_ssh_channel
    if global_ssh_channel is None:
        start_wsd_control()
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
        <button class="arrow-button" onclick="sendCommand('motor1_left')">&larr;</button>
        <div class="motor-info">
          <div class="motor-name">MOTOR 1</div>
        </div>
        <button class="arrow-button" onclick="sendCommand('motor1_right')">&rarr;</button>
      </div>

      <div class="motor-row">
        <button class="arrow-button" onclick="sendCommand('motor2_left')">&larr;</button>
        <div class="motor-info">
          <div class="motor-name">MOTOR 2</div>
        </div>
        <button class="arrow-button" onclick="sendCommand('motor2_right')">&rarr;</button>
      </div>

      <div class="motor-row">
        <button class="arrow-button" onclick="sendCommand('motor3_left')">&larr;</button>
        <div class="motor-info">
          <div class="motor-name">MOTOR 3</div>
        </div>
        <button class="arrow-button" onclick="sendCommand('motor3_right')">&rarr;</button>
      </div>

      <!-- Step Size Control -->
      <div class="step-control">
        <button class="small-button" onclick="sendCommand('decrease_step')">-</button>
        <span id="step-size-text">Step Size: n</span>
        <button class="small-button" onclick="sendCommand('increase_step')">+</button>
      </div>

      <button class="big-button" onclick="location.href='/'">BACK</button>

      <script>
        function sendCommand(cmd) {
          // Replace with your actual AJAX call if needed:
          fetch('/send_char?cmd=' + cmd)
            .then(response => response.text())
            .then(data => {
              console.log("Response:", data);
              // Optionally update step size text if your backend returns a new value:
              if(cmd === 'increase_step' || cmd === 'decrease_step'){
                // You might have logic here to update the step size display.
                document.getElementById('step-size-text').innerText = "Step Size: n";
              }
            });
        }
      </script>
    </body>
    </html>
    '''
    return render_template_string(template)

# Endpoint to send commands to the interactive SSH channel.
@app.route('/send_char')
def send_char():
    global global_ssh_channel
    cmd = request.args.get('cmd')
    if global_ssh_channel:
        print("Sending command:", cmd)
        global_ssh_channel.send(cmd + '\n')
        output = ""
        while global_ssh_channel.recv_ready():
            output += global_ssh_channel.recv(1024).decode()
        if output:
            print("Channel output:", output)
        return f"Sent '{cmd}'"
    else:
        print("SSH channel not active.")
        return "SSH channel not active."

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)