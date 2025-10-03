# -*- coding: utf-8 -*-

# @title
#                   ***General Outline     ***
# 1) Submerge down to 0.3 meters above the floor
# 2) Turn until we find at least one pole on the gate and record the angle.
#   2a) Rotate until we see the second pole, store the angle. 
#   2b) Use the two angles to compute the angle for directly facing the gate.
# 3) Move forward until the marker is within the CV detection
#   3a) Turn left until marker is at the 0.8 axis in image
#   3b) Move forward for the marker's distance from camera
#   3c) Turn right 90 degrees and check marker's location in image.
#   3d) If marker is in the right portion of the image, record the distance moved in step 3, and move forward.
# 4) Repeat step 3b to 3d
# 5) Move the same distance as step 3.
# 6) Re-execute the gate search function
# 7) Pass through gate
# 8) Move forward until 0.5 meters away from the wall
# 9) Surface the sub!

# Computer vision: A list of bounding boxes [x1, x2, y1, y2, class]. (x1, y1) is the top left corner. (x2, y2) is the bottom right corner. Coordinates from 0-1
import rospy
from rospy import sleep
from std_msgs.msg import Float32, Int32MultiArray, Bool, Float32MultiArray
from util import *



rospy.init_node('qualification', anonymous=True)
# The number of move forward needed to move through the gate after gate poles are out of sight.
through_gate = 0.5

sensor = {}

# Subscribe to the CV output
cvSub = rospy.Subscriber('CV', Float32MultiArray, cvCallback, callback_args=sensor)
thrusterPub = rospy.Publisher("thruster", Int32MultiArray, queue_size=2)

#Subscribing to the depth sensor
# depthSub = rospy.Subscriber('depth', Float32, depthCallback, callback_args=sensor)
# pressureSub = rospy.Subscriber('pressure_sensor', Float32, pressureCallback, callback_args=sensor)

# Subscribing to IMU to get angle
gyroSub = rospy.Subscriber('zed/gyro', Float32MultiArray, gyroCallback, callback_args=(sensor, thrusterPub))

# Subscribing to leak sensor and temperature sensor for emergency exit.
# leakSub = rospy.Subscriber('leak', Bool, leakCallback, callback_args=(sensor, thrusterPub))
temperatureSub = rospy.Subscriber('front/temp', Float32, temperatureCallback, callback_args=(sensor, thrusterPub))

# Subscribe to IMU to get distance
distanceSub = rospy.Subscriber("zed/displacement", Float32MultiArray, distanceCallback, callback_args=sensor)

# Contain the class number for each object
CV_dictionary = {"gate":0, "pole":[1, 2]}


def alignMarker(axis):
  # Ensure marker is at the axis at the specific x-coordinate.
  while (True):
    for i in cv():
      # Detected the marker
      if compareClass(i, 'marker', CV_dictionary):
        x1 = i[0]
        x2 = i[1]
        if (abs((x1+x2)/2)-axis)<=0.05:
          # Return the width of the marker
          return (x2-x1)
        elif abs(axis - x1) < abs(x2 - axis):
          move('right', sensor, thrusterPub, distance=3)
        else:
          move('left', sensor, thrusterPub, distance=3)
    # Marker not detected by cv
    move("right", sensor, thrusterPub, distance=5)




def objectCaptured(object):
  # Check whether the object is captured by the camera
  # Return the x-coordinates of the center of the object
  for i in cv():
    if compareClass(i, object, CV_dictionary):
      print(object ," captured by the camera, the x-coordinates is: ", (i[0] + i[1]) / 2)
      return (i[0] + i[1]) / 2
  print("NO ", object, " captured by the camera")
  return -1


def aroundMarker():
# Get close to the marker
# Take a left turn
# Keep going forward til the marker is not visible
# Turn right
# Keep going forward until marker is not visible
# Turn right
  print("Around marker begins")
  sleep(5)
  alignMarker(0.5)
  distance_from_marker = getDistance("marker", sensor, CV_dictionary)
  if len(distance_from_marker) > 0:
    move("forward", sensor, thrusterPub, distance=distance_from_marker[0]-1)
  else:
    print("Error: Cannot get distance from marker after aligning marker")
    surfacing(sensor, thrusterPub)
    exit()
  alignMarker(0.8) 

  # move till gone for the marker
  distanceMoved = getDistance("marker", sensor, CV_dictionary)
  if len(distanceMoved) > 0:
    move("forward", sensor, thrusterPub, distance=distanceMoved[0])
  else:
    print("Error: Cannot get distance from marker after aligning marker to 0.8")
    surfacing(sensor, thrusterPub)
    exit()

  turn(90, sensor, thrusterPub)
  captured = objectCaptured("marker")
  # Move forward until the marker is right of the 0.7 axis when sub is turned toward the marker.
  # This ensures the submarine
  while captured < 0.7 and captured != -1:
    # Turn left 90, move forward, turn right 90, check position of marker
    turn(270, sensor, thrusterPub)
    move("forward", sensor, thrusterPub)
    distanceMoved += 0.2
    turn(90, sensor, thrusterPub)
    captured = objectCaptured("marker")

  # move past the marker
  if captured == -1:
    alignMarker(0.8) 
  distance_from_marker = getDistance("marker", sensor, CV_dictionary)
  if len(distanceMoved) > 0:
    move("forward", sensor, thrusterPub, distance=distance_from_marker[0])
  else:
    print("Error: Cannot get distance from marker after aligning marker to 0.8")
    surfacing(sensor, thrusterPub)
    exit()

  turn(90, sensor, thrusterPub)
  captured = objectCaptured("marker")
  while captured < 0.7 and captured != -1:
    turn(270, sensor, thrusterPub)
    move("forward", sensor, thrusterPub)
    turn(90, sensor, thrusterPub)
    captured = objectCaptured("marker")
  # Move back the same distance as move forward
  move("forward", sensor, thrusterPub, distance=distanceMoved)
  print("Around marker ends")


def main():
  print("Qualification Start")
  sleep(6)
  print(sensor)
  changeDepth(0.3, sensor, thrusterPub)
  distance_from_pole, angle_from_left, angle_difference = searchGate("center", sensor, thrusterPub, CV_dictionary)
  correction_angle, distance_to_move = gateAngleCorrection(distance_from_pole, angle_from_left, angle_difference)
  # move through the gate 
  move("forward", sensor, thrusterPub, distance=distance_to_move + through_gate)
  print("passed the gate")
  turn(correction_angle)

  # Uncomment code block below for pre qualification
  # aroundMarker()

  # distance_from_pole, angle_from_left, angle_difference = searchGate("center", sensor, thrusterPub, CV_dictionary)
  # correction_angle, distance_to_move = gateAngleCorrection(distance_from_pole, angle_from_left, angle_difference)
  # # move through the gate 
  # move("forward", sensor, thrusterPub, distance=distance_to_move + through_gate)
  # print("Finished the qualification task.") 

  surfacing(sensor, thrusterPub)

if __name__ == "__main__":
  main()