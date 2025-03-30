from flask import Flask, request, render_template_string
import paramiko

app = Flask(__name__)

# SSH configuration for your Raspberry Pi (update these with your actual settings)
SSH_HOST = '192.168.176.137'  # or use an IP address, e.g., '192.168.1.100'
SSH_PORT = 22
SSH_USER = 'mkhl'

def run_command(command):
    """Execute a command on the Raspberry Pi via SSH."""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(SSH_HOST, port=SSH_PORT, username=SSH_USER, password=SSH_PASS)
    stdin, stdout, stderr = ssh.exec_command(command)
    output = stdout.read().decode()
    error = stderr.read().decode()
    ssh.close()
    return output, error

@app.route('/')
def index():
    template = '''
    <!doctype html>
    <html>
    <head>
      <title>Robot Arm Control</title>
      <style>
         body { 
             margin: 0; 
             font-family: Arial, sans-serif; 
             background-color: #f4f4f4; 
         }
         .container { 
             display: flex; 
             height: 100vh; 
         }
         .sidebar {
             width: 300px;
             background-color: #333;
             color: white;
             display: flex;
             flex-direction: column;
             align-items: center;
             padding: 20px;
         }
         .sidebar h2 { 
             margin-bottom: 20px; 
         }
         .button {
             background-color: #4CAF50;
             border: none;
             color: white;
             padding: 15px;
             margin: 10px 0;
             text-align: center;
             font-size: 16px;
             width: 80%;
             border-radius: 5px;
             cursor: pointer;
         }
         .content {
             flex-grow: 1;
             padding: 20px;
             background-color: #fff;
             overflow: auto;
         }
         .output {
             border: 1px solid #ccc;
             background-color: #f9f9f9;
             padding: 10px;
             border-radius: 5px;
             height: 90%;
             overflow: auto;
         }
      </style>
    </head>
    <body>
      <div class="container">
        <div class="sidebar">
          <h2>Controls</h2>
          <!-- High-level commands -->
          <button class="button" onclick="sendCommand('init')">Init Motors</button>
          <button class="button" onclick="sendCommand('execute')">Execute Pre-existing Path</button>
          <hr style="width:80%; border:1px solid #555;">
          <!-- Manual motor controls -->
          <button class="button" onclick="sendCommand('motor1_up')">Motor 1 Up</button>
          <button class="button" onclick="sendCommand('motor1_down')">Motor 1 Down</button>
          <button class="button" onclick="sendCommand('motor2_left')">Motor 2 Left</button>
          <button class="button" onclick="sendCommand('motor2_right')">Motor 2 Right</button>
          <button class="button" onclick="sendCommand('motor3_forward')">Motor 3 Forward</button>
          <button class="button" onclick="sendCommand('motor3_backward')">Motor 3 Backward</button>
        </div>
        <div class="content">
          <h2>Output</h2>
          <div class="output" id="output">
            <p>Command output will appear here...</p>
          </div>
        </div>
      </div>
      <script>
         function sendCommand(cmd) {
             fetch('/control?cmd=' + cmd)
             .then(response => response.text())
             .then(data => {
                 const outputDiv = document.getElementById('output');
                 outputDiv.innerHTML += '<p>' + data + '</p>';
                 outputDiv.scrollTop = outputDiv.scrollHeight;
             });
         }
      </script>
    </body>
    </html>
    '''
    return render_template_string(template)

@app.route('/control')
def control():
    cmd = request.args.get('cmd')
    # Mapping commands to actual shell commands.
    # For real commands, replace the echo commands or script paths accordingly.
    command_map = {
         'init': 'python /home/pi/robot/init_motors.py',
         'execute': 'python /home/pi/robot/execute_path.py',
         'motor1_up': 'echo "Motor 1 up executed"',
         'motor1_down': 'echo "Motor 1 down executed"',
         'motor2_left': 'echo "Motor 2 left executed"',
         'motor2_right': 'echo "Motor 2 right executed"',
         'motor3_forward': 'echo "Motor 3 forward executed"',
         'motor3_backward': 'echo "Motor 3 backward executed"'
    }
    if cmd in command_map:
        command = command_map[cmd]
        output, error = run_command(command)
        # Return the result, trimming any extra whitespace.
        return f"Command '{cmd}': Output: {output.strip()}, Error: {error.strip()}"
    else:
        return f"Unknown command: {cmd}"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)