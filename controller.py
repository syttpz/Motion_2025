import rospy
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Float32MultiArray
from rospy import sleep
import argparse

from util import move, gyroCallback, distanceCallback, depthCallback

parser = argparse.ArgumentParser()
parser.add_argument("--direction", type=str, required=True, help="Type f for moving forward, b for moving backward, u for moving up, d for moving down, l for turning left, r for turning right, s for stoping all thrusters. Typing anything else will terminate the program.")
parser.add_argument("--distance", type=float, required=True, help="Enter the distance or angle.")
parser.add_argument("--sleep", type=int, required=True, help="Enter the amount of time to wait before moving")
args = parser.parse_args()

rospy.init_node('controller', anonymous=True)

thrusterPub = rospy.Publisher("thruster", Int32MultiArray, queue_size=2)
sensor = {}
# Get angle from IMU
gyroSub = rospy.Subscriber('zed/gyro', Float32MultiArray, gyroCallback, callback_args=(sensor, thrusterPub))
# Get distance travelled from IMU
distanceSub = rospy.Subscriber("zed/displacement", Float32MultiArray, distanceCallback, callback_args=sensor)
# Get distance from bottom from bottom camera
depthSub = rospy.Subscriber('depth_sensor', Float64MultiArray, depthCallback, callback_args=sensor)

direction = args.direction
distance = args.distance
sleep_time = args.sleep
sleep(int(sleep_time))
direction = direction.upper()
if direction == "F":
    move("forward", sensor, thrusterPub, distance)
elif direction == 'B':
    move("backward", sensor, thrusterPub, distance)
elif direction == "U":
    move("up", sensor, thrusterPub, distance)
elif direction == "D":
    move("down", sensor, thrusterPub, distance)
elif direction == "L":
    move("left", sensor, thrusterPub, distance)
elif direction == "R":
    move("right", sensor, thrusterPub, distance)
elif direction == "S":
    pubMsg = Int32MultiArray()
    pubMsg.data=[0, 0]
    thrusterPub.publish(pubMsg)
else:
    exit()

pubMsg = Int32MultiArray()
pubMsg.data=[0, 0]
thrusterPub.publish(pubMsg)