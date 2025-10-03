import rospy
import ast
from time import sleep
from std_msgs.msg import Float64MultiArray, Float64, Int32MultiArray, Bool
from util import cvCallback, depthCallback, gyroCallback, cv, turn, changeDepth, searchGate, move, moveTillGone, distanceCallback


DEPTH = 2


if __name__ == '__main__':
    rospy.init_node('publish')

    cvPub = rospy.Publisher('CV', Float64MultiArray)
    depthPub = rospy.Publisher('depth_sensor', Float64MultiArray)
    gyroPub = rospy.Publisher('gyro_sensor', Float64MultiArray)
    distancePub = rospy.Publisher('displacement_sensor', Float64MultiArray)
    pressurePub = rospy.Publisher("pressure_sensor", Float64)
    leakPub = rospy.Publisher("leak_sensor", Float64)

    #prevyear
    cvBottomPub = rospy.Publisher('CV_bottom', Float64MultiArray)
     


    while not rospy.is_shutdown():
        topic = input("Topic: ").upper()
        msg = input("Message: ")
        if topic == "SETUP":
            initial_CV = Float64MultiArray()
            initial_CV.data = []
            initial_depth = Float64MultiArray()
            initial_depth.data = 640*480*[DEPTH]
            initial_gyro = Float64MultiArray()
            initial_gyro.data = [0, 0, 0]
            initial_distance = Float64MultiArray()
            initial_distance.data = [0, 0, 0]
            initial_CV_bottom = Float64MultiArray()
            initial_CV_bottom.data = []
            initial_pressure = Float64(0)
            rospy.sleep(1)
            print("Sending initial values")
            cvPub.publish(initial_CV)
            depthPub.publish(initial_depth)
            gyroPub.publish(initial_gyro)
            distancePub.publish(initial_distance)
            cvBottomPub.publish(initial_CV_bottom)
            pressurePub.publish(initial_pressure)
        elif topic == "CV":
            msg = ast.literal_eval(msg)
            cvMsg = Float64MultiArray()
            cvMsg.data = msg
            cvPub.publish(cvMsg)
        elif topic == "DEPTH":
            depthF = float(msg)
            pressureF = DEPTH - depthF
            pressureMsg = Float64(pressureF)
            depthMsg = Float64MultiArray(640*480*[depthF])
            depthPub.publish(depthMsg)
            pressurePub.publish(pressureMsg)
        elif topic == "GYRO":
            msg = ast.literal_eval(msg)
            gyroMsg = Float64MultiArray()
            gyroMsg.data = msg
            gyroPub.publish(gyroMsg)
        elif topic == "DISTANCE":
            msg = ast.literal_eval(msg)
            distanceMsg = Float64MultiArray()
            distanceMsg.data = msg
            distancePub.publish(distanceMsg)
        elif topic == "CV_BOTTOM":
            msg = ast.literal_eval(msg)
            cvBottomMsg = Float64MultiArray()
            cvBottomMsg.data = msg
            cvBottomPub.publish(cvBottomMsg)
        elif topic == "LEAK":
            leakF = float(msg)
            leakPub.publish(Float64(leakF))
        elif topic == "EXIT":
            exit(0)
        else:
            print("Invalid topic")


