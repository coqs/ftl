# your comments are still here
# Turn() - for turning a specific angle, can be used for zigzag lines or 90-degree turns
# lfStopBlue() - stops once either of the color sensors is blue
# lfDistance() - line follows for a distance
# lfwaitturnLR90 - keeps following line until left or right line detect then aligns (turns 90 degrees) to that line (only use for left or right lanes/lines)
# lfwaitturnLR45 - same as above but turns 45 degrees instead, good for zigzags!!!
#

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

# motors and sensors
leftMtr = Motor(port=Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
rightMtr = Motor(port=Port.C, positive_direction=Direction.CLOCKWISE)
leftSnsr = ColorSensor(Port.A)
rightSnsr = ColorSensor(Port.B)
distanceSensor = UltrasonicSensor(port=Port.D)

# pid gains - tune these for your robot
kp = 1.9
ki = 0.02
kd = 1.5
gyro_kp = 1.5 # for the gyro straight function

# pid state variables
integral = 0
last_error = 0
lr = None

distancestuff = 50 
wheelDiameter = 56

dbRobot = DriveBase(left_motor=leftMtr, right_motor=rightMtr, wheel_diameter=wheelDiameter, axle_track=119.5)

# --- core pid and movement logic ---

def reset_pid():
    # gotta reset the pid brain before each new move
    global integral, last_error
    integral = 0
    last_error = 0

def do_pid_step(speed):
    # this is the main pid calculation
    global integral, last_error
    error = leftSnsr.reflection() - rightSnsr.reflection()
    integral += error
    # this stops the integral from getting too huge if the robot gets lost
    if abs(integral) > 1000:
        if integral > 0:
            integral = 1000
        else:
            integral = -1000
    derivative = error - last_error
    turn_rate = (kp * error) + (ki * integral) + (kd * derivative)
    dbRobot.drive(speed, turn_rate)
    last_error = error

def go_straight_gyro(distance_mm: float, speed: int):
    # this is just for moving straight accurately, used by the spin turn now
    dbRobot.reset()
    hub.imu.reset_heading(0)
    while dbRobot.distance() < distance_mm:
        correction = hub.imu.heading() * gyro_kp
        dbRobot.drive(speed, -correction)
    dbRobot.stop()

def Turn(angle: int, speed: int, pivot: bool = False):
    # ok this is the smart turn function now
    dbRobot.stop()
    
    # if it's a spin turn we move forward first to center the robot on the junction
    # this is exactly what you were saying
    if not pivot:
        go_straight_gyro(distancestuff, speed)

    hub.imu.reset_heading(0)
    
    if pivot:
        if angle > 0: # turning right pivots on the right wheel
            rightMtr.hold()
            leftMtr.run(speed)
        else: # turning left pivots on the left wheel
            leftMtr.hold()
            rightMtr.run(-speed)
        
        while abs(hub.imu.heading()) < abs(angle):
            wait(2)
    else: # normal spin turn
        if angle > 0:
            dbRobot.drive(0, speed)
        else:
            dbRobot.drive(0, -speed)

        while abs(hub.imu.heading()) < abs(angle):
            wait(2)

    leftMtr.hold()
    rightMtr.hold()
    wait(50)

# --- your functions now clean and using the right logic ---

def lfStopBlue(speed: int) -> None:
    reset_pid()
    while leftSnsr.color() != Color.BLUE and rightSnsr.color() != Color.BLUE:
        do_pid_step(speed)
    # stops exactly on the blue line now, no extra movement
    dbRobot.stop()

def lfDistance(speed: int, distance: float) -> None:
    reset_pid()
    dbRobot.reset()
    target_mm = distance * 10
    while dbRobot.distance() < target_mm:
        do_pid_step(speed)
    dbRobot.stop()

def lfWaitTurnLR90(speed: int) -> None:
    reset_pid()
    dbRobot.reset()
    while dbRobot.distance() < 20:
        do_pid_step(speed)

    while leftSnsr.color() != Color.BLUE and rightSnsr.color() != Color.BLUE:
        do_pid_step(speed)
    dbRobot.stop()
    
    if leftSnsr.color() == Color.BLUE:
        lr = "L"
    else:
        lr = "R"
    
    # now we just call the turn function it handles the rest
    if lr == "L":
        Turn(angle=-90, speed=speed, pivot=True)
    else:
        Turn(angle=90, speed=speed, pivot=True)

def lfWaitTurnLR45(speed: int) -> None:
    reset_pid()
    dbRobot.reset()
    while dbRobot.distance() < 20:
        do_pid_step(speed)
        
    while leftSnsr.color() != Color.BLUE and rightSnsr.color() != Color.BLUE:
        do_pid_step(speed)
    dbRobot.stop()
    
    if leftSnsr.color() == Color.BLUE:
        lr = "L"
    else:
        lr = "R"

    if lr == "L":
        Turn(angle=-45, speed=speed, pivot=True)
    else:
        Turn(angle=45, speed=speed, pivot=True)
        
def lfWaitBlueLine(speed: int, startcm: float, stopCondition: str) -> None:
    lfDistance(speed=speed, distance=startcm)
    
    reset_pid()
    stop = None
    while stop != stopCondition:
        do_pid_step(speed)
        
        is_left_blue = leftSnsr.color() == Color.BLUE
        is_right_blue = rightSnsr.color() == Color.BLUE

        if is_left_blue and is_right_blue:
            stop = "B"
        elif is_left_blue:
            stop = "L"
        elif is_right_blue:
            stop = "R"

    # stops exactly where it sees the line
    dbRobot.stop()

def lfStopBarrier(speed: int, barrierDistance: float) -> None:
    reset_pid()
    barrierDistanceMM = barrierDistance * 10
    while distanceSensor.distance() > barrierDistanceMM:
        do_pid_step(speed)
    dbRobot.stop()

def turnAroundBarrier(turnRorL: str, turnSpeed: int, angle: float, moveSpeed: int, moveDistance: float):
    if turnRorL == "R":
        Turn(angle=angle, speed=turnSpeed)
        dbRobot.settings(straight_speed=moveSpeed)
        dbRobot.straight(moveDistance)
        Turn(angle=-angle, speed=turnSpeed)
        dbRobot.settings(straight_speed=moveSpeed)
        dbRobot.straight(moveDistance)
    elif turnRorL == "L":
        Turn(angle=-angle, speed=turnSpeed)
        dbRobot.settings(straight_speed=moveSpeed)
        dbRobot.straight(moveDistance)
        Turn(angle=angle, speed=turnSpeed)
        dbRobot.settings(straight_speed=moveSpeed)
        dbRobot.straight(moveDistance)

def mainFunc() -> None:
    while not any(hub.buttons.pressed()):
        wait(10)
        
    lfWaitBlueLine(speed=200, startcm=1, stopCondition="L")
    Turn(angle=-90, speed=200, pivot=True)
    lfWaitBlueLine(speed=200, startcm=1, stopCondition="R")
    Turn(angle=90, speed=200, pivot=True)
    lfWaitBlueLine(speed=200, startcm=1, stopCondition="R")
    Turn(angle=90, speed=200, pivot=True)
    lfStopBarrier(speed=200, barrierDistance=12)

mainFunc()
