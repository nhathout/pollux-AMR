#!/usr/bin/env python3

import rospy
import threading
from std_msgs.msg import Float32MultiArray, Int32
from flask import Flask, jsonify, request

app = Flask(__name__)

# Globals for storing latest sensor data and the ROS publisher handle
latest_ultrasonic = []
cmd_pub = None

# Motor command definitions (must match motor_cmd_node.py)
FORWARD_CMD  = 0
BACKWARD_CMD = 1
STOP_CMD     = 6

@app.route('/api/sensors', methods=['GET'])
def get_sensors():
    """
    Returns the latest ultrasonic data as JSON.
    Example response:
    {
      "ultrasonic": [10.5, 9.8, 11.2, 12.0]
    }
    """
    return jsonify({"ultrasonic": latest_ultrasonic})

@app.route('/api/motor_cmd', methods=['POST'])
def motor_cmd():
    """
    Expects JSON body: { "cmd": <integer> }
    Publishes that integer on /pollux/motor_cmd.
    """
    global cmd_pub
    if cmd_pub is None:
        return jsonify({"error": "Motor command publisher not available"}), 500

    data = request.json or {}
    cmd = data.get('cmd')
    if cmd is None:
        return jsonify({"error": "No 'cmd' field provided"}), 400

    rospy.loginfo(f"[web_bridge_node] Received motor_cmd from web: {cmd}")
    cmd_pub.publish(Int32(data=cmd))
    return jsonify({"status": "ok", "cmd": cmd})

@app.route('/api/stop', methods=['POST', 'GET'])
def stop_robot():
    """
    Simple endpoint to stop the robot using STOP_CMD (6).
    """
    global cmd_pub
    if cmd_pub is None:
        return jsonify({"error": "Motor command publisher not available"}), 500

    rospy.loginfo("[web_bridge_node] Stop command received from web UI.")
    cmd_pub.publish(Int32(data=STOP_CMD))
    return jsonify({"status": "robot stopped"})

@app.route('/api/start', methods=['POST', 'GET'])
def start_robot():
    """
    Simple endpoint to start the robot in forward motion using FORWARD_CMD (0).
    """
    global cmd_pub
    if cmd_pub is None:
        return jsonify({"error": "Motor command publisher not available"}), 500

    rospy.loginfo("[web_bridge_node] Start command received from web UI.")
    cmd_pub.publish(Int32(data=FORWARD_CMD))
    return jsonify({"status": "robot moving forward"})

def ultrasonic_callback(msg):
    """
    Subscriber callback for /pollux/ultrasonic
    """
    global latest_ultrasonic
    # e.g. [front_left, front_right, rear_left, rear_right]
    latest_ultrasonic = list(msg.data)

def ros_thread():
    """
    Initializes ROS node, subscribers, and publishers.
    Runs in a background thread so Flask can run in the main thread.
    """
    global cmd_pub

    rospy.init_node('web_bridge_node', anonymous=True)
    rospy.loginfo("web_bridge_node started. Subscribing to /pollux/ultrasonic.")

    # Subscribe to the ultrasonic data
    rospy.Subscriber('/pollux/ultrasonic', Float32MultiArray, ultrasonic_callback)

    # Create a publisher for motor commands
    cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

    # Keep this thread alive with spin()
    rospy.spin()

def main():
    # Start ROS in a separate thread
    t = threading.Thread(target=ros_thread)
    t.start()

    # Run the Flask app in the main thread, on all interfaces (0.0.0.0)
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass