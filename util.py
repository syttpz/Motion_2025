# -*- coding: utf-8 -*-

from std_msgs.msg import Int32MultiArray
from math import *
from time import time
from rospy import sleep
import copy
import numpy as np


# Threshold for temperature and moisture
TEMP_T = 45

# initial value for depth and pressure, added for changeDepth function
#INIT_DEPTH = 1
#INIT_PRESSURE = 0

# constants for image size
IMG_WIDTH = 640
IMG_HEIGHT = 480

# Assumed pool depth in m
POOL_DEPTH = 3 

def unflatten(data, length=5):
  # Convert the CV data to a list of list
  # Parameter: length is the number of elements in the inner array
  
  if len(data) % length != 0:
    print("unflatten - Invalid data length in CV result")
    return []
  output_count = int(len(data) / length)
  result = []
  for i in range(output_count):
    bbox = []
    for j in range(length*i, length*(i+1)):
      bbox.append(float(data[j]))
    result.append(bbox)
  return result


def cvCallback(data, sensor):
  print("CV result received")
  # sensor is the object to be modified
  # Computer vision: A list of bounding boxes [x1, x2, y1, y2, class, object depth, confidence]. (x1, y1) is the top left corner. (x2, y2) is the bottom right corner. Coordinates from 0-1
  sensor["CV_result"] = unflatten(list(data.data), length=7)
  



def cv(sensor):
  # Return the copy of computer vision data to avoid the risk of CV_result being modified while being processed
  return copy.deepcopy(sensor.get("CV_result"))


def cvBottomCallback(data, sensor):
  print("CV bottom result received")
  sensor["CV_bottom"] = unflatten(list(data.data))
  

def depthMapFrontCallback(data, sensor):
  print("Depth map front received")
  sensor["depth_map_front"] = unflatten(list(data.data), length=IMG_WIDTH)


def depthMapFront(sensor):
  result = sensor.get("depth_map_front") 
  if result is None:
    return None
  else:
    return copy.deepcopy(result)
  

def cvBottom(sensor):
  return copy.deepcopy(sensor.get("CV_bottom"))


def compareClass(bbox, objectName, cvDict):
  object_id = cvDict.get(objectName)
  bbox_class = bbox[4]
  if object_id is None:
    raise Exception(f"{objectName} not in cvDict")
  if isinstance(object_id, int):
    return bbox_class == object_id
  elif isinstance(object_id, (list, tuple, set)):
    return bbox_class in object_id
  else:
    raise Exception(f"Invalid type for {objectName}'s id in cvDict")


def findObject(object, bboxes, cvDict):
  # Look for the object in the list of bounding boxes
  # bboxes is a list of bounding boxes.
  bounding_boxes = []
  for i in bboxes:
    if compareClass(i, object, cvDict):
      bounding_boxes.append(i)
  return bounding_boxes


def move(direction, sensor, thrusterPub, distance=0.2):
  # direction can be "forward", "backward", "left", "right", "up", "down"
  # Default distance to move is 0.2 m each time
  if direction == "left" or direction == "right":
    print(f"Turning {direction} by distance: {distance} degree")
  else:
    print(f"Moving {direction} by distance: {distance} meters")
  # 0 for forward and backward, 1 for turning, 2 for changing depth, 3 for pitch and 4 for roll
  if direction == "forward":
    PIDxy(sensor, distance, thrusterPub)
  elif direction == "backward":
    PIDxy(sensor, -distance, thrusterPub)
  elif direction == "left":
    PIDturn(sensor, -distance, thrusterPub)
  elif direction == "right":
    PIDturn(sensor, distance, thrusterPub)
  elif direction == "up":
    PIDdepth(sensor, distance, thrusterPub)
  elif direction == "down":
    PIDdepth(sensor, -distance, thrusterPub)
  else:
    print("Invalid direction input: %s", direction)
    return


def depthCallback(data, sensor):
  # Set the distance from the bottom of the pool in meter
  print("Depth updated")
  sensor["depth"] = float(data.data)

  # depth_matrix = np.array(depth_msg.data).reshape((720, 1280)) #height, width  

  # sensor["depth"] = getDepth(depth_matrix)

def getDepth(depthmap):
    dmap = np.array(depthmap)
    row, col = dmap.shape

    percent = 0.25

    srow = round(row / 2 - row*percent)
    erow = round(row / 2 + row*percent)
    scol = round(col / 2 - col*percent)
    ecol = round(col / 2 + col*percent)

    cent_matrix = dmap[srow:erow, scol:ecol]

    return cent_matrix.mean() #distance in float
'''
def pressureCallback(data, sensor):
  # Set the distance from the surface of the water in meter
  print("Pressure updated")
  sensor["pressure"] = float(data.data)
'''

def gyroCallback(data, args):
  sensor = args[0]
  thrusterPub = args[1]
  print("Gyro updated")
  # The angles on x-axis, y-axis, and z-axis from gyrometer. Format is 360 degrees. angles[2] is suppose to be the horizontal angle 
  # angle[0] is angle with x-axis used for pitch 
  # angle[1] is angle with y-axis used for roll
  # angle[2] is angle with z-axis used for yaw -> turn

  #We can get the current angle of the robot and adjust 
  # movement constantly to ensure the robot is always facing 
  # the correct direction during movement.
  data = list(data.data)
  angles = []
  angles.append(float(data[0]))
  angles.append(float(data[2]))
  angles.append(float(data[1]))
  # for i in range(len(data)):
  #   angles.append(float(data[i]))
  sensor["angles"] = angles

  # if not sensor.get("roll_pitch", False) and abs(angles[1]) > 30:
  #   sensor["roll_pitch"] = True
  #   PIDroll(sensor, -angles[1] , thrusterPub)
  #   sensor["roll_pitch"] = False

  # # if we are not doing roll ourself adjust the pitch and roll to stablize 
  # if not sensor.get("roll_pitch", False) and abs(angles[0]) > 30:
  #   sensor["roll_pitch"] = True
  #   PIDpitch(sensor, -angles[0] , thrusterPub)
  #   sensor["roll_pitch"] = False



def distanceCallback(data, sensor):
  print("Distance updated")
  data = list(data.data)
  displacements = []
  displacements.append(float(data[0]))
  displacements.append(float(data[2]))
  displacements.append(float(data[1]))
  sensor["distance"] = displacements
  sensor["depth"] = POOL_DEPTH - float(data[1])
  

def temperatureCallback(data, args):
  sensor = args[0]
  thrusterPub = args[1]
  print(f"Temperature updated: {sensor['temperature']}")
  # Callback function for the temperature sensor subscriber
  temp_val = float(data.data)
  sensor['temperature'] = temp_val
  if temp_val > TEMP_T:
    print("Critical temperature dected")
    endRun(sensor, thrusterPub)


def leakCallback(data, args):
  sensor = args[0]
  thrusterPub = args[1]
  print("Leak updated")
  leak_val = bool(data.data)
  sensor['leak'] = leak_val
  # Callback function for the leak sensor subscriber
  sensor['leak'] = data # data is float
  if leak_val: # if data is greater than the threshold
    print("Leak detected")
    endRun(sensor, thrusterPub)


def endRun(sensor, thrusterPub):
  # Make the robot move to the surface and end the program.
  print("Ending run")
  while sensor.get("depth") < POOL_DEPTH:
    move("up", sensor, thrusterPub)  
    pubMsg = Int32MultiArray()
    pubMsg.data=[2, 400]
    thrusterPub.publish(pubMsg)
  pubMsg = Int32MultiArray()
  pubMsg.data=[2, 0]
  thrusterPub.publish(pubMsg)
  exit()

def changeDepth(target, sensor, thrusterPub):
  print(f"Changing depth to {target} meters")
  cur_depth = sensor.get("depth")
  # Change the depth to target meters above the bottom of the pool. Depth from camera being used
  #initial_depth = 0
  if target > POOL_DEPTH or target < 0:
    print("Invalid target, out of range")
    return -1
  if sensor.get("depth") > target:
    move("down", sensor, thrusterPub, sensor.get("depth") - target) 
  elif sensor.get("depth") < target:
    move("up", sensor, thrusterPub, target - sensor.get("depth")) 
  print("Finished changing depth")

def surfacing(sensor, thrusterPub):

  print("Surfacing begin")
  tolerance = 0.1 # 10 cm tolerance
  prev_depth = sensor.get("depth")
  attempts = 0

  while attempts < 15: # Max attemp 15
    move("up", sensor, thrusterPub, 0.3) #move up 30 cm
    time.sleep(0.5)

    new_depth = sensor.get("depth")
    #if the change in distance is within tolerance, the robot surfaced
    if abs(new_depth - prev_depth) < tolerance:  
      print(f"Surfaced at depth: {new_depth}")
      return True
    prev_depth = new_depth
    attempts += 1
  print("Failed to surface within the allowed attempts.")
  return False


def turn(degree, sensor, thrusterPub):
  # Turn degree clockwise. Does not support negative
  print(f"Turning by {degree} degrees")
  initAngle = sensor.get("angles")[2]
  if degree > 180:
    move("left", sensor, thrusterPub, (360-degree))
  else:
    move("right", sensor, thrusterPub, degree)
  angleDiff = (sensor.get("angles")[2] - initAngle) % 360
  while abs(angleDiff - degree) >= 2:
    if angleDiff < degree and degree - angleDiff < 180:
      move("right", sensor, thrusterPub, 1)
    else:
      move("left", sensor, thrusterPub, 1)
    angleDiff = (sensor.get("angles")[2] - initAngle) % 360


def searchGate(target, sensor, thrusterPub, cvDict):
  # Find the gate and point target.
  # Steps:
  # 1. Reotate right until find one pole, record its angle
  # 2. Rotate right until find second pole, record its angle
  # 3. Calculate difference in angle, if difference less than 180, turn left until facing center/left center of poles
  # 4. If difference larger than 180, turn right until facing center/left center of poles.

  # target can be set to "center" or "left", which the center of the gate or the midpoint between left pole and center of the gate.
  # poleFound is the number of pole whose angle has been determined.
  
  print("Searching gate begins")
  poleFound = 0
  poleAngle = [0, 0]
  prevPoleCenter = None
  marginOfError = 0.05
  poleDistances = []
  while (True):
    # Get a list of bounding boxes [x1, x2, y1, y2, class]. (x1, y1) is the top left corner. (x2, y2) is the bottom right corner. Coordinates from 0-1
    bboxes = cv(sensor)
    # poleCount is the number of poles detected
    poleCount = 0
    # curPoleCenter records the x-coordinate of the center of the poles detected.
    curPoleCenter = []
    polebb = []
    for bbox in bboxes:
      if compareClass(bbox, 'pole', cvDict):
        poleCount += 1
        print("Total of", poleCount, "poles detected")
        # Add the x-coordinate of the middle of the pole that is detected.
        curPoleCenter.append((bbox[0] + bbox[1])/2)
        polebb.append(bbox)
        print("Current pole center:", curPoleCenter)

    # The pole at the center of the frame
    centeredPole = None

    # Decide action based on the number of poles in the image
    if poleCount == 1 and abs(curPoleCenter[0] - 0.5) <= marginOfError:
      # If one pole is detected and that pole is in the center of the frame.
      print("One pole detected at the center of the frame")
      centeredPole =  curPoleCenter[0]
    elif poleCount == 2:
      print("Both poles detected in the same frame")
      # Two poles are found in the same frame
      if abs(curPoleCenter[0] - 0.5) < marginOfError:
        centeredPole = curPoleCenter[0]
      elif abs(curPoleCenter[1] - 0.5) < marginOfError:
        centeredPole = curPoleCenter[1]
      print("Current pole at the center of the frame: ", curPoleCenter)    
    # No pole found otherwise


    if centeredPole:
        # Decide whether this pole is different from last pole whose angle is stored.
        if not prevPoleCenter:
          # No pole in the last frame, this is a new pole
          poleFound += 1
          depth_map_front = depthMapFront(sensor)
          poleDistances.append(getBBdistance(polebb, depth_map_front))
          print("New pole found, total pole found:", poleFound)
          poleAngle[poleFound - 1] = sensor.get("angles")[2]
        elif prevPoleCenter <= centeredPole:
          # The pole in previous frame is left of the pole in the current frame.
          # As the robot is moving right, the pole is a new pole
          poleFound += 1
          depth_map_front = depthMapFront(sensor)
          poleDistances.append(getBBdistance(polebb, depth_map_front))
          print("New pole found and is not a repeat, total pole found:", poleFound)
          poleAngle[poleFound - 1] = sensor.get("angles")[2]
        else:
          print(f"Repeated pole: previous pole at {prevPoleCenter} and current pole at {centeredPole}")

        # Otherwise, the pole in previous frame is right of the pole in the current frame.
        # As the robot is moving right, the pole is the previous pole
        
        prevPoleCenter = centeredPole
        if poleFound ==2:
          # Both pole has been found. Decide where is the gate based on the angles
          angleDifference = (poleAngle[1] - poleAngle[0]) % 360
          targetAngle = None
          normalizedAngleDifference = angleDifference
          # Add angle difference to the pole
          if angleDifference > 180:
            # The difference in angle of pole 2 and pole 1 is larger than 180
            # The gate must be behind the curve of rotation.
            normalizedAngleDifference = 360-angleDifference
            print("The angle difference is: ", angleDifference, "behind the gate")
            if target == "center":
              angleFromLeftPole = (360-angleDifference)/2
              targetAngle = (angleFromLeftPole + poleAngle[1]) % 360
            elif target == "left":
              angleFromLeftPole = (360-angleDifference)/4
              targetAngle = (angleFromLeftPole + poleAngle[1]) % 360
            poleDistances[0], poleDistances[1] = poleDistances[1], poleDistances[0]
          else:
            print("The angle difference is: ", angleDifference, ", facing the gate")
            if target == "center":
              angleFromLeftPole = angleDifference/2
              targetAngle = (angleFromLeftPole + poleAngle[0]) % 360
            elif target == "left":
              angleFromLeftPole = angleDifference/4
              targetAngle = (angleFromLeftPole + poleAngle[0]) % 360
          turning_angle = (targetAngle - sensor.get("angles")[2])%360
          print(f"Target angle is {targetAngle} degree. Turning clockwise {turning_angle} degree")
          turn(turning_angle, sensor, thrusterPub)
          print("Searching gate ends")
          return poleDistances, angleFromLeftPole, normalizedAngleDifference
    else:
      prevPoleCenter = None
    # Turn until we find at least one pole on the gate
    move("right", sensor, thrusterPub, distance=5)


def gateAngleCorrection(poleDistances, angleFromLeftPole, normalizedAngleDifference, gatewidth):
  # angleFromLeftPole is the difference between the current angle and the angle when facing left pole.
  # Find the angle between the line joining current position to left pole and the gate using sine rule.
  pole1Angle = np.arcsin(np.sin(normalizedAngleDifference)/gatewidth * poleDistances[1])
  angleFromGate = angleFromLeftPole + pole1Angle
  # Find the angle needs to be corrected after the robot passed through the gate 
  # such that the robot facing direction will perpendicular to both poles
  angleCorrection = (90 - angleFromGate) % 360

  # Get the horizontal distance from the left pole
  distanceFromLeftPole = (normalizedAngleDifference/angleFromLeftPole) * gatewidth
  # Find the slant distance to move to pass the gate
  distanceToMove = distanceFromLeftPole/np.sin(angleFromLeftPole) * np.sin(pole1Angle)
  return angleCorrection, distanceToMove


def alignObj(obj, sensor, thrusterPub, cvDict, axis=0.5):
  print("Begin aligning to ", obj)
  # ALign horizontally to ensure the specified object is at a specific part of the camera frame.
  angleMoved = 0
  while (True):
    for i in cv(sensor):
      # Detected the marker
      if compareClass(i, obj, cvDict):
        print("Marker detected")
        x1 = i[0]
        x2 = i[1]
        if (abs((x1+x2)/2)-axis)<=0.05:
          # Return the width of the marker
          print("Align object ends")
          return (x2-x1)
        elif abs(axis - x1) < abs(x2 - axis):
          move('right', sensor, thrusterPub, distance=1)
          angleMoved += 1
        else:
          move('left', sensor, thrusterPub, distance=1)
          angleMoved -= 1
    # Marker not detected by cv
    print("Marker not detected by cv, moving to the right")
    move("right", sensor, thrusterPub, distance=5)
    angleMoved+=5
    return angleMoved%360

def getDistance(object, sensor, cvDict):
  # Get the distance from the object to the camera
  # object is the string representing the object
  # sensor is the dictionary containing the sensor data
  bounding_box = findObject(object, cv(sensor), cvDict)
  if len(bounding_box) == 0:
    print("Object not found")
    return []
  else:
    depth_map_front = depthMapFront(sensor)
    distance = []
    for bb in bounding_box:
      # Get the average depth of the object
     distance.append(getBBdistance(bb, depth_map_front))
    return distance


def getBBdistance(bb, depth_map_front=None):
  if depth_map_front is None:
    return bb[5]
  #get the average distance from the center of the bounding box
  x1, x2, y1, y2, _ = bb

  # converting 0-1 coordinate into pixel values
  x1 = x1 * IMG_WIDTH
  x2 = x2 * IMG_WIDTH
  y1 = y1 * IMG_HEIGHT
  y2 = y2 * IMG_HEIGHT

  new_x1 = round(x1 + (x2 - x1) / 4)
  new_x2 = round(x2 - (x2 - x1) / 4)

  new_y1 = round(y1 + (y2 - y1) / 4)
  new_y2 = round(y2 - (y2 - y1) / 4)

  depth_map_front = np.array(depth_map_front)
  reduced_depth_map_front = depth_map_front[new_y1:new_y2 , new_x1:new_x2]
  return float(np.mean(reduced_depth_map_front))


def moveTillGone(object, sensor, thrusterPub, cvDict):
  print("Move till gone begins")
  counter = 0
  while True:
    result = findObject(object, cv(sensor), cvDict)
    if result:
      move("forward", sensor, thrusterPub)
      counter += 0.2
    else:
      print("Object gone")
      return counter


def PID(Kp, Ki, Kd, e, time_prev, e_prev, integral):
    cur_time = time()

    # PID calculations
    P = Kp*e
    integral = integral
    D = 0
    if time_prev and e_prev:
      integral = integral + Ki*e*(cur_time - time_prev)
      D = Kd*(e - e_prev)/(cur_time - time_prev + 1e-6)
    # print(f"P: {P}, I: {integral}, D: {D}")
    # calculate manipulated variable - MV
    MV = P + integral + D
    sleep(0.01)
    return MV, cur_time, integral


def PIDxy(sensor, target, thrusterPub):
  # Move target distance in the xy plane, target can be negative
  start_x = sensor.get("distance")[0]
  start_y = sensor.get("distance")[1]
  target_x = start_x + cos(sensor.get("angles")[2]) * target
  target_y = start_y + sin(sensor.get("angles")[2]) * target
  time_prev = None
  e_prev = None
  integral = 0
  while True:
    cur_x = sensor.get("distance")[0]
    cur_y = sensor.get("distance")[1]
    e = ((target_x - cur_x)**2 + (target_y - cur_y)**2)**0.5
    speed, time_prev, integral = PID(200, 0.1, 1000, e, time_prev, e_prev, integral)
    e_prev = e
    # speed is m/s^2
    message = []
    pubMsg = Int32MultiArray()
    if abs(speed) < 10 and abs(e_prev) < 0.1:
      message.append(0)
      message.append(0)
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
      break
    else:
      message.append(0)
      message.append(round(speed))
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
    print("PID xy")


def PIDturn(sensor, target, thrusterPub):
  # turn the target angle, clockwise is positive
  start = sensor.get("angles")[2]
  target = start + target
  time_prev = None
  e_prev = None
  integral = 0
  while True:
    e = target - sensor.get("angles")[2]
    speed, time_prev, integral = PID(10, 0.01, 10, e, time_prev, e_prev, integral)
    e_prev = e
    # speed is degree/s^2
    message = []
    pubMsg = Int32MultiArray()
    if abs(speed) < 10 and abs(e_prev) < 1:
      message.append(1)
      message.append(0)
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
      break
    else:
      message.append(1)
      message.append(round(speed))
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
    print("PID turn")
    

def PIDdepth(sensor, target, thrusterPub):
    # Move up the target distance. target can be negative
    start = sensor.get("depth")
    target = start + target
    time_prev = None
    e_prev = None
    integral = 0
    while True:
      e = target - sensor.get("depth")
      speed, time_prev, integral = PID(200, 0.1, 1000, e, time_prev, e_prev, integral)
      e_prev = e
      # speed is degree/s^2
      message = []
      pubMsg = Int32MultiArray()
      if abs(speed) < 10 and abs(e_prev) < 0.1:
        message.append(2)
        message.append(0)
        pubMsg.data = message
        thrusterPub.publish(pubMsg)
        break
      else:
        message.append(2)
        message.append(round(speed))
        pubMsg.data = message
        thrusterPub.publish(pubMsg)
      print("PID depth")


def PIDpitch(sensor, target, thrusterPub):
  # Move the target angle in pitch. Clockwise (sub's head is lifting) is positive.
  start = sensor['angles'][0] # angle with x-axis
  target = start + target
  time_prev = None
  e_prev = None
  integral = 0
  while True:
    e = target - sensor.get("angles")[0] # error
    speed, time_prev, integral = PID(10, 0.01, 10, e, time_prev, e_prev, integral)
    e_prev = e
    # speed is degree/s^2
    message = []
    pubMsg = Int32MultiArray()
    if abs(speed) < 10 and abs(e_prev) < 1:
      message.append(3) # 3 for pitch
      message.append(0)
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
      break
    else:
      message.append(3)
      message.append(round(speed))
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
    print("PID pitch")


def PIDroll(sensor, target, thrusterPub):
  # Move the target angle in roll. Clockwise is positive
  start = sensor['angles'][1] # angle with y-axis
  target = start + target
  time_prev = None
  e_prev = None
  integral = 0
  while True:
    e = target - sensor.get("angles")[1] # error
    speed, time_prev, integral = PID(10, 0.01, 10, e, time_prev, e_prev, integral)
    e_prev = e
    # speed is degree/s^2
    message = []
    pubMsg = Int32MultiArray()
    if abs(speed) < 10 and abs(e_prev) < 1:
      message.append(4) # 4 for roll
      message.append(0)
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
      break
    else:
      message.append(4)
      message.append(round(speed))
      pubMsg.data = message
      thrusterPub.publish(pubMsg)
    print("PID roll")


