import rospy
from rospy import sleep
from std_msgs.msg import Float64MultiArray, Float64, Int32MultiArray
from util import cvCallback, depthCallback, gyroCallback, cv, turn, changeDepth, searchGate, move, moveTillGone, distanceCallback, pressureCallback

rospy.init_node('test_sub', anonymous=True)


sensor = {}

# Subscribe to the CV output
cvSub = rospy.Subscriber('CV', Float64MultiArray, cvCallback, callback_args=sensor)
thrusterPub = rospy.Publisher("thruster", Int32MultiArray)

#Subscribing to the depth sensor
depthSub = rospy.Subscriber('depth_sensor', Float64, depthCallback, callback_args=sensor)
pressureSub = rospy.Subscriber('pressure_sensor', Float64, pressureCallback, callback_args=sensor)

# Subscribing to IMU to get angle
gyroSub = rospy.Subscriber('gyro_sensor', Float64MultiArray, gyroCallback, callback_args=(sensor, thrusterPub))

# Subscribe to IMU to get distance
distanceSub = rospy.Subscriber("displacement_sensor", Float64MultiArray, distanceCallback, callback_args=sensor)

rospy.spin()