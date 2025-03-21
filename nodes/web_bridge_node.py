import rospy
from std_msgs.msg import Float32MultiArray
from flask import Flask, jsonify
import threading

app = Flask(__name__)


latest_distances = []

def ultrasonic_callback(msg):
    global latest_distances
    latest_distances = msg.data 

@app.route('/api/sensors', methods=['GET'])
def get_sensors():
    return jsonify({"ultrasonic": list(latest_distances)})

def ros_thread():
    rospy.init_node('web_bridge_node', anonymous=True)
    rospy.loginfo("web_bridge_node started. Subscribing to /pollux/ultrasonic.")
    rospy.Subscriber('/pollux/ultrasonic', Float32MultiArray, ultrasonic_callback)
    rospy.spin()

def main():
    t = threading.Thread(target=ros_thread)
    t.start()

    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
