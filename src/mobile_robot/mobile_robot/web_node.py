import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PSA ROBUST - Web Control</title>
<style>
    body {
        background: #101827;
        color: white;
        font-family: Arial, sans-serif;
        text-align: center;
        margin: 0;
        padding: 30px 15px;
    }

    .container {
        max-width: 650px;
        margin: auto;
    }

    h1 {
        color: #38bdf8;
        margin-bottom: 5px;
    }

    .subtitle {
        color: #94a3b8;
        margin-bottom: 30px;
    }

    .card {
        background: #1e293b;
        padding: 25px;
        border-radius: 15px;
        margin-bottom: 20px;
    }

    .zones {
        display: flex;
        gap: 10px;
        margin: 25px 0;
    }

    .zone {
        flex: 1;
        padding: 30px 0;
        background: #334155;
        border-radius: 10px;
        font-size: 32px;
        font-weight: bold;
    }

    .active {
        background: #0284c7;
    }

    .value {
        font-size: 22px;
        color: #38bdf8;
    }

    button {
        padding: 18px 45px;
        margin: 10px;
        border: none;
        border-radius: 10px;
        color: white;
        font-size: 18px;
        font-weight: bold;
        cursor: pointer;
    }

    .start {
        background: #16a34a;
    }

    .stop {
        background: #dc2626;
    }

    button:disabled {
        opacity: 0.5;
        cursor: not-allowed;
    }

    #feedback {
        color: #94a3b8;
        min-height: 24px;
    }
</style>
</head>

<body>
<div class="container">

    <h1>PSA ROBUST</h1>
    <p class="subtitle">ROS 2 Web Control Panel</p>

    <div class="card">
        <h2>BALL POSITION</h2>

        <div class="zones">
            <div class="zone" id="L">L</div>
            <div class="zone" id="C">C</div>
            <div class="zone" id="R">R</div>
        </div>

        <p>Detection: <span class="value" id="detection">N</span></p>
        <p>Ball Center: <span id="center">N/A</span></p>
    </div>

    <div class="card">
        <h2>ROBOT CONTROL</h2>

        <button class="start" onclick="sendRobotCommand('start')">START</button>
        <button class="stop" onclick="sendRobotCommand('stop')">STOP</button>

        <p>FSM State: <span class="value" id="state">UNKNOWN</span></p>
        <p id="feedback"></p>
    </div>

</div>

<script>
async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();

        document.getElementById('detection').textContent = data.detection;
        document.getElementById('state').textContent = data.state;

        document.getElementById('center').textContent =
            data.center ? `(${data.center.x}, ${data.center.y})` : 'N/A';

        ['L', 'C', 'R'].forEach(zone => {
            document.getElementById(zone).classList.toggle(
                'active', zone === data.detection
            );
        });
    } catch (error) {
        document.getElementById('state').textContent = 'DISCONNECTED';
    }
}

async function sendRobotCommand(action) {
    const feedback = document.getElementById('feedback');
    feedback.textContent = 'Sending command...';

    try {
        const response = await fetch('/api/' + action, {
            method: 'POST'
        });

        const data = await response.json();
        feedback.textContent = data.message || 'No response';

        await updateStatus();
    } catch (error) {
        feedback.textContent = 'Command failed';
    }
}

setInterval(updateStatus, 200);
updateStatus();
</script>

</body>
</html>
"""


class WebNode(Node):

    def __init__(self):
        super().__init__('web_node')

        self.lock = threading.Lock()

        self.detection = 'N'
        self.center = None
        self.state = 'UNKNOWN'

        self.create_subscription(
            String, '/detection', self.detection_callback, 10
        )

        self.create_subscription(
            Point, '/ball/center', self.center_callback, 10
        )

        self.create_subscription(
            String, '/fsm/state', self.state_callback, 10
        )

        self.start_client = self.create_client(
            Trigger, '/fsm/start'
        )

        self.stop_client = self.create_client(
            Trigger, '/fsm/stop'
        )

        self.server = ThreadingHTTPServer(
            ('0.0.0.0', 8080),
            self.create_handler()
        )

        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            daemon=True
        )

        self.server_thread.start()

        self.get_logger().info(
            'Web Control started: http://localhost:8080'
        )

    def detection_callback(self, msg):
        with self.lock:
            self.detection = msg.data
            if msg.data == 'N':
                self.center = None

    def center_callback(self, msg):
        with self.lock:
            self.center = {
                'x': round(msg.x),
                'y': round(msg.y)
            }

    def state_callback(self, msg):
        with self.lock:
            self.state = msg.data

    def get_status(self):
        with self.lock:
            return {
                'detection': self.detection,
                'center': self.center,
                'state': self.state
            }

    def create_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):

            def send_response_data(self, data, content_type, status=200):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def send_json(self, data, status=200):
                self.send_response_data(
                    json.dumps(data).encode(),
                    'application/json',
                    status
                )

            def do_GET(self):
                if self.path == '/':
                    self.send_response_data(
                        HTML.encode(),
                        'text/html; charset=utf-8'
                    )

                elif self.path == '/api/status':
                    self.send_json(node.get_status())

                else:
                    self.send_json(
                        {'message': 'Not found'}, 404
                    )

            def do_POST(self):
                if self.path == '/api/start':
                    client = node.start_client

                elif self.path == '/api/stop':
                    client = node.stop_client

                else:
                    self.send_json(
                        {'message': 'Not found'}, 404
                    )
                    return

                if not client.service_is_ready():
                    self.send_json(
                        {'message': 'FSM service unavailable'}, 503
                    )
                    return

                future = client.call_async(Trigger.Request())

                event = threading.Event()
                future.add_done_callback(lambda _: event.set())

                if not event.wait(timeout=3.0):
                    self.send_json(
                        {'message': 'FSM response timeout'}, 504
                    )
                    return

                try:
                    result = future.result()

                    self.send_json({
                        'success': result.success,
                        'message': result.message
                    })

                except Exception as error:
                    self.send_json({
                        'success': False,
                        'message': str(error)
                    }, 500)

            def log_message(self, format, *args):
                pass

        return Handler

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = WebNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
