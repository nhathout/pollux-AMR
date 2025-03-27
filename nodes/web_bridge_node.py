

import rospy
import threading
from std_msgs.msg import Float32MultiArray, Int32
from flask import Flask, jsonify, request

app = Flask(__name__)


latest_ultrasonic = []
cmd_pub = None 

@app.route('/api/sensors', methods=['GET'])
def get_sensors():
    """
    Returns the latest ultrasonic data in JSON.
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
    Then publishes that integer on /pollux/motor_cmd.
    """
    if cmd_pub is None:
        return jsonify({"error": "Motor command publisher not available"}), 500

    data = request.json
    cmd = data.get('cmd')
    if cmd is None:
        return jsonify({"error": "No 'cmd' field provided"}), 400

    rospy.loginfo(f"web_bridge_node => Received motor_cmd from web: {cmd}")
    cmd_pub.publish(Int32(data=cmd))
    return jsonify({"status": "ok", "cmd": cmd})

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
    Runs in a background thread so Flask can run in main thread.
    """
    global cmd_pub

    rospy.init_node('web_bridge_node', anonymous=True)
    rospy.loginfo("web_bridge_node started. Subscribing to /pollux/ultrasonic.")

    
    rospy.Subscriber('/pollux/ultrasonic', Float32MultiArray, ultrasonic_callback)

    
    cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

    rospy.spin()

def main():
    
    t = threading.Thread(target=ros_thread)
    t.start()

    
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

