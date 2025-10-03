# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Int32MultiArray
from math import *
from util import *
from time import sleep

rospy.init_node('qualification', anonymous=True)

# The distance needed to move through the gate after the flag are out of sight in m
through_gate = 1
# Width of the gate in m
gate_width = 3
# Move step size
step = 0.2
# Contain the class number for each object
cvDict = {"pole":0}
# Flag for whether to pass through the gate with style 
style = True

# CV_result: A list of bounding boxes [x1, x2, y1, y2, class]. (x1, y1) is the top left corner. (x2, y2) is the bottom right corner. Coordinates from 0-1
# angle: The angles on x-axis, y-axis, and z-axis from gyrometer. Format is 360 degrees. angles[2] is suppose to be the horizontal angle
# depth: The distance from the bottom of the pool in meters
# pressure: The distance from water surface in meters.
sensor={}

# Subscribe to the CV output
cvSub = rospy.Subscriber('CV', Float64MultiArray, cvCallback, callback_args=sensor)
cvBottomSUb = rospy.Subscriber('CV_bottom', Float64MultiArray, cvBottomCallback, callback_args=sensor)

thrusterPub = rospy.Publisher("thruster", Int32MultiArray)
# Get distance from bottom from bottom camera
depthSub = rospy.Subscriber('depth_sensor', Float64MultiArray, depthCallback, callback_args=sensor)

# Get angle from IMU
gyroSub = rospy.Subscriber('gyro_sensor', Float64MultiArray, gyroCallback, callback_args=(sensor, thrusterPub))
# Get distance from surface from pressure sensor
pressureSub = rospy.Subscriber("pressure_sensor", Float64, pressureCallback, callback_args=sensor)
# Get distance travelled from IMU
distanceSub = rospy.Subscriber("displacement_sensor", Float64MultiArray, distanceCallback, callback_args=sensor)

def alignPath(path):
  # First place the path at the center of the image.
  # Then use the width of the bounding box to determine the direction of the path.
  # path is the initial bounding box of the path.
  x_center = (path[0] + path[1])/2
  y_center = (path[2] + path[3])/2
  print(f"Starting alignment. Initial x_center: {x_center}, y_center: {y_center}")
  while abs(x_center - 0.5) > 0.05 or abs(y_center - 0.5) > 0.05:
    if abs(y_center - 0.5) > 0.05:
      # y-coordinate is not aligned
      if y_center > 0.5:
        # If path is at the upper part of the frame, move forward.
        print("Path is at the upper part of the frame, moving forward.")
        move("forward", sensor, thrusterPub, 0.1)
        bboxes = cvBottom()
        path = findObject("path", bboxes, cvDict)
        while(not path):
          # Path is no longer in the frame, which means we overshoot
          print("Path overshot, moving backward.")
          move("backward", sensor, thrusterPub, 0.01)
          bboxes = cvBottom()
          path = findObject("path", bboxes, cvDict)
      else:
        # Path is at the lower part of the frame, move backward.
        print("Path is at the lower part of the frame, moving backward.")
        move("backward", sensor, thrusterPub, 0.1)
        bboxes = cvBottom(sensor)
        path = findObject("path", bboxes, cvDict)
        while(not path):
          print("Path overshot, moving forward.")
          move("forward", sensor, thrusterPub, 0.01)
          bboxes = cvBottom(sensor)
          path = findObject("path", bboxes, cvDict)
    else:
      # x-coordinate is not aligned
      if x_center > 0.5: 
        # If path is at the right part of the frame, move right.
        print("Path is at the right part of the frame, moving to the right.")
        turn(90, sensor, thrusterPub)
        move("forward", sensor, thrusterPub, 0.1)
        turn(270, sensor, thrusterPub)
        bboxes = cvBottom(sensor)
        path = findObject("path", bboxes, cvDict)
        while(not path):
          print("Path overshot, moving left.")
          turn(270, sensor, thrusterPub)
          move("forward", sensor, thrusterPub, 0.01)
          turn(90, sensor, thrusterPub)
          bboxes = cvBottom(sensor)
          path = findObject("path", bboxes, cvDict)
      else:
        print("Path is at the left part of the frame, moving to the left.")
        turn(270, sensor, thrusterPub)
        move("forward", sensor, thrusterPub, 0.1)
        turn(90, sensor, thrusterPub)
        bboxes = cvBottom(sensor)
        path = findObject("path", bboxes, cvDict)
        while(not path):
          print("Path overshot, moving right.")
          turn(90, sensor, thrusterPub)
          move("forward", sensor, thrusterPub, 0.01)
          turn(270, sensor, thrusterPub)
          bboxes = cvBottom(sensor)
          path = findObject("path", bboxes, cvDict)
      x_center = (path[0][0] + path[0][1])/2
      y_center = (path[0][2] + path[0][3])/2
      print(f"Updated x_center: {x_center}, y_center: {y_center}")
    directPath(path)


def directPath(pathObj):
    # Use the width of the bounding box to determine the direction of the path.
    # At the start of the function, we are pointing away from the gate
    width = pathObj[1] - pathObj[0]
    length = pathObj[3] - pathObj[2]
    cur_ratio = length / width
    turning_angle = 0
    print(f"Initial ratio of length and width: {cur_ratio}, starting direction alignment.")

    # cur_ratio is the largest ratio
    for i in range(0, 90, 1):
        turn(1, sensor, thrusterPub)
        sleep(0.1)
        # Get the current bounding box ratio
        pathObj = cvBottom(sensor)
        width = pathObj[1] - pathObj[0]
        length = pathObj[3] - pathObj[2]
        new_ratio = length / width
        if new_ratio > cur_ratio:
            cur_ratio = new_ratio
            turning_angle = i
        print(f"Turned {i} degrees, new ratio: {new_ratio}, current best angle: {turning_angle}")

    turn(180, sensor, thrusterPub)
    print("Turned 180 degrees, checking the other side.")

    for i in range(0, 90, 1):
        turn(1, sensor, thrusterPub)
        sleep(0.1)
        # Get the current bounding box ratio
        width = pathObj[1] - pathObj[0]
        length = pathObj[3] - pathObj[2]
        new_ratio = length / width
        if new_ratio > cur_ratio:
            cur_ratio = new_ratio
            turning_angle = i + 270
        print(f"Turned {i + 90} degrees, new ratio: {new_ratio}, current best angle: {turning_angle}")

    # turning_angle with the largest ratio
    turn(turning_angle, sensor, thrusterPub)
    # robot facing the direction of the path marker
    print(f"Turned to the best angle: {turning_angle}. Robot is facing the path marker.")

def followThePath():
  # Find the path and then orient the robot in the direction of the path
  print("Finding the path and alighning the robot in the direction of the path.")
  changeDepth(1.5, sensor, thrusterPub)
  while True:
    turn(270, sensor, thrusterPub)
    # Move left from the point out of the gate for the gate's width while search for the path
    for i in range(ceil(gate_width / step)):
      bboxes = cvBottom(sensor)
      path = findObject("path", bboxes, cvDict)
      if path:
        # If path is found, turn to point away from the gate and align the gate.
        turn (90)
        alignPath(path[0])
        return
      move("forward", sensor, thrusterPub, distance=step)
    turn(180, sensor, thrusterPub)
    # Turn 180 degrees, move right from the point out of the gate for double the gate's width while search for the path
    # This is to cover the distance we moved left
    for i in range(ceil((gate_width * 2) / step)):
      bboxes = cvBottom(sensor)
      path = findObject("path", bboxes, cvDict)
      if path:
        turn(270, sensor, thrusterPub)
        # If path is found, turn to point away from the gate and align the gate.
        alignPath(path[0])
        return
      move("forward", sensor, thrusterPub, distance=step)
    print("Path not found, moving backward and retrying.")
    move("backward", sensor, thrusterPub, gate_width * 2)
    turn(270, sensor, thrusterPub)
    move("forward", sensor, thrusterPub, 1)

def alignVertical(obj):
  # Align the obj vertically.
  print('Aligning the object vertically')
  reachedBottom = False
  while (True):
    found = False
    for i in cv(sensor):
      # Detected the obj
      if i[4] == cvDict[obj]:
        y1 = i[2]
        y2 = i[3]

        y_center = (y1+y2)/2
        found = True
        if abs(y_center - 0.5) > 0.05:
          if y_center > 0.5:
            move("up", sensor, thrusterPub)
          else:
            move("down", sensor, thrusterPub)
        else:
          print('Object aligned vertically')
          return True

    if not found:
      print("Object not found, change depth to try to find the object")
      # obj not detected by cv
      if sensor.get("depth") > 0.2 and not reachedBottom:
        # Have not reached bottom of pool before, so search by going down.
        move("down", sensor, thrusterPub)
      elif sensor.get("pressure") > 0.3:
        # Have reached bottom of pool, so search by going up.
        reachedBottom = True
        move("up", sensor, thrusterPub)
      else:
        # Reached top and bottom, object not found 
        print("Cannot find object")
        return False

def slalom(sensor, thrusterPub, cvDict):
  # Task 2 of 2025 competition
  # Pass through a series of gates.
  class PoleInfo():
    def __init__(self, angle, distance):
      self.angle = angle
      self.distance = distance

  class PoleCollection():
    def __init__(self):
      self.poles = []
      self.closest = None
    
    def add_pole(self, pole:PoleInfo):
      pole.append(pole)
      if self.closest is None:
        self.closest = pole
      elif pole.distance < self.closest.distance:
        self.closest = pole
    
    def remove_closest(self):
      self.poles.remove(self.closest)
      self.closest = self.poles[0]
      for pole in self.poles:
        if pole.distance < self.closest.distance:
          self.closest = pole

  print("Navigate the channel begins")
  poleTot = 3
  clockwiseRot = 90
  antiClockwiseRot = 90
  marginOfError = 0.05
  step = 5
  gate_width = 2

  for i in range(poleTot):
    whitePipes = PoleCollection()
    redPipes = PoleCollection()
    # Turn left by 180 - clockwiseRot (90 degrees)
    turn(360-clockwiseRot, sensor, thrusterPub)
    # Rotate by clockwiseRot + antiClockwiseRot, with step (5) degree each time 
    for i in range((antiClockwiseRot+clockwiseRot)//step):
      # At each step, do a cv check and find all white and red pipes. 
      # Save their angle and distance
      bboxes = cv(sensor)
      depth_map_front = depthMapFront(sensor)
      whitePipeBbs = findObject("whitePipe", bboxes, cvDict)
      redPipeBbs = findObject("redPipe", bboxes, cvDict)
      for bbox in whitePipeBbs:
        # Only save pipe if they are in the center, so we can store their angle.
        if abs((bbox[0] + bbox[1])/2 -0.5) < marginOfError:
          newPole = PoleInfo(sensor.get("angles")[2], getBBdistance(bbox, depth_map_front))
          whitePipes.add_pole(newPole)
      for bbox in redPipeBbs:
        if abs((bbox[0] + bbox[1])/2 -0.5) < marginOfError:
          newPole = PoleInfo(sensor.get("angles")[2], getBBdistance(bbox, depth_map_front))
          redPipes.add_pole(newPole) 
      turn(step, sensor, thrusterPub)
    # So all pipe in the 180 degree in front of robot are captured
    
    # We want to pass through the left side of the pipes, 
    # so white pipe is on the left and red pip on the right.
    leftPole = whitePipes.closest
    rightPole = redPipes.closest

    # Both poles has been found. Decide where is the gate based on the angles
    angleDifference = (rightPole.angle - leftPole.angle) % 360

    while angleDifference > 180:
      # This means the white pipe is right of the red pipe, but we want to pass through the left side of the pipes.
      print("Angle difference larger than 180, one pole is selected wrong")
      whitePipes.remove_closest()
      leftPole = whitePipes.closest
      angleDifference = (rightPole.angle - leftPole.angle) % 360
    
    targetAngle = None
    # Add angle difference to the pole
    print("The angle difference is: ", angleDifference)
    angleFromLeftPole = angleDifference/2
    targetAngle = (angleFromLeftPole + leftPole.angle) % 360
    turning_angle = (targetAngle - sensor.get("angles")[2])%360
    print(f"Target angle is {targetAngle} degree. Turning clockwise {turning_angle} degree")
    turn(turning_angle, sensor, thrusterPub)
    print(f"Pointing to the center of set {i+1} poles")

    angleCorrection, distanceToMove = gateAngleCorrection([leftPole.distance, rightPole.distance], angleFromLeftPole, angleDifference, gate_width)
    move("forward", sensor, thrusterPub, distance=distanceToMove)
    turn(angleCorrection)
    move("forward", sensor, thrusterPub, distance=0.5)
    changeDepth(1, sensor, thrusterPub)


def style_through_gate(sensor, thrusterPub):
  # Move through the gate with style
  start_angle = sensor["angles"].copy()
  pubMsg = Int32MultiArray()
  message = []
  message.append(0)
  message.append(300)
  pubMsg.data = message
  thrusterPub.publish(pubMsg)
  sleep(0.5)
  pubMsg.data[0] = 0
  pubMsg.data[1] = 0
  thrusterPub.publish(pubMsg)
  sensor["roll_pitch"] = True
  PIDroll(sensor, 180, thrusterPub)
  pitch_diff = sensor["angles"][1]
  if abs(pitch_diff) > 1:
    PIDpitch(sensor, -pitch_diff, thrusterPub)
  yaw_diff = start_angle[2] - sensor["angles"][2]
  if abs(yaw_diff) > 5:
    PIDturn(sensor, yaw_diff, thrusterPub)
  pubMsg.data[0] = 0
  pubMsg.data[1] = 300
  thrusterPub.publish(pubMsg)
  sleep(0.5)
  pubMsg.data[0] = 0
  pubMsg.data[1] = 0
  thrusterPub.publish(pubMsg)
  PIDroll(sensor, 180, thrusterPub)
  sensor["roll_pitch"] = False
  
  print('Style through gate completed')

def main():
  changeDepth(0.3, sensor, thrusterPub)
  poleDistances, angleFromLeftPole, angleDifference = searchGate("left", sensor, thrusterPub, cvDict)
  targetClass = None # The string for which class we are targeting.
  while True:
    class1 = findObject("class1img1", cv(sensor), cvDict)
    class2 = findObject("class2img1", cv(sensor), cvDict)
    if len(class1) > 0:
      class1Center = abs((class1[0][0] + class1[0][1])/2 - 0.5)
      if class1Center < 0.2:
        targetClass = "class1"
    if len(class2) > 0:
      class2Center = abs((class2[0][0] + class2[0][1])/2 - 0.5)
      if class2Center < 0.2:
        if targetClass is None or class1Center > class1Center:
          targetClass = "class2"
          
    if targetClass is not None:
      break
    move("forward", sensor, thrusterPub)

  angleCorrection, _ = gateAngleCorrection(poleDistances, angleFromLeftPole, angleDifference, gate_width)
  # Move till gone using the distance from the cv sensor
  distances = getDistance(targetClass+"img1", sensor, cvDict)
  if len(distances) == 0:
    print("Already close to gate such that image cannot be captured")
    if style:
      style_through_gate(sensor, thrusterPub)
    else:
      move("forward", sensor, thrusterPub, through_gate)
  else:
    move_distance = distances[0]
    if len(distances) > 1:
      move_distance = min(distances)
    if style:
      move("forward", sensor, thrusterPub, move_distance - through_gate)
      style_through_gate(sensor, thrusterPub)
    else:
      move("forward", sensor, thrusterPub, move_distance)
  turn(angleCorrection)
  followThePath()
  # Travel to task 2
  for i in range(20):
    bboxes = cv(sensor)
    depth_map_front = depthMapFront(sensor)
    redPipeBbs = findObject("redPipe", bboxes, cvDict)
    closest = None
    for bb in redPipeBbs:
      distance = getBBdistance(bb, depth_map_front)
      if closest is None or distance < closest:
        closest = distance
    if closest < 1.8:
      break
    else:
      move("forward", sensor, thrusterPub, 0.3)

  slalom(sensor, thrusterPub, cvDict)
  changeDepth(0, sensor, thrusterPub)


if __name__ == "__main__":
  wait_time = input("Start the program in x seconds: ")
  sleep(int(wait_time))
  main()
