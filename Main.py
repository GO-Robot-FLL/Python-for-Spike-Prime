# LEGO type:standard slot:1 autostart

from spike import PrimeHub, Motor, MotorPair, ColorSensor, DistanceSensor, App
from spike.control import wait_for_seconds, wait_until, Timer
from hub import battery
from util import log


import util
import math
import sys

#PID value Definition
pRegler = 0.0
iRegler = 0.0
dRegler = 0.0

pReglerLight = 0.0
iReglerLight = 0.0
dReglerLight = 0.0

#Preperation for parallel code execution
accelerate = True
run_generator = True
runSmall = True


# Create your objects here.
hub = PrimeHub()

#initialize the motors
leftMotor = Motor('B') #adjust the motor ports until they match your configuration, we recommend assigning your ports to the ones in the program for ease of use
rightMotor = Motor('C')
smallMotorA = Motor('A')
smallMotorD = Motor('D')
leftMotor.set_degrees_counted(0)
rightMotor.set_degrees_counted(0)

"""
Initialize color Sensors
left sensor: port F
right sensor: port E
"""
colorE = ColorSensor('E') #adjust the sensor ports until they match your configuration, we recommend assigning your ports to the ones in the program for ease of use
colorF = ColorSensor('F')




"""
Set movement motors
left motor: port B
right motor: port C
"""
movement_motors = MotorPair('C', 'B') #adjust the ports until they match your configuration, we recommend assigning your ports to the ones in the program for ease of use

cancel = False


def lineFollower(distance, startspeed, maxspeed, endspeed, addspeed, brakeStart, side , sensorPort, driveToLinePort = 0, lightValue = 0, detectLineStart = 0,):
    """
        This is the function used to let the robot follow a line until either the entered distance has been achieved or the other sensor of the robot senses a line.
        Like all functions that drive the robot this function has linear acceleration and breaking. It also uses PID values that are automatically set depending on the
        current speed of the robot (See function PIDCalculationLight)

        Parameters
        -------------

        distance: The value tells the program the distance the robot has to drive. Type: Integer. Default: No default value

        speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value

        maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value

        endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value

        addspeed: The percentage after which the robot reaches its maxspeed. Type: Float. Default: No default value

        brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: No default value

        side: Wich side of the line the robot should use. Type: String Accpeted inputs: "left" "right"

        sensorPort: This value tells the program which ColorSensor the robot should use to follow the line. Type: String. Accepted inputs: "E", "F"

        driveToLinePort: Tells the program if it should stop if it sees a light value on the specified sensor. Type: String. Accpeted inputs: "E", "F"

        lightvalue: Tells the program what lightvalue needs to be reached on the driveToLinePort as an EndCondition. Type: Integer

        detectLineStart: The value which we use to tell the robot after what percentage of the distance we need to look for the line to drive to. Type: Float. Default: 0
    """
    #start of break function implementation, used to cancel current program without terminating main
    if cancel:
        return
    #set the speed the robot starts at
    speed = startspeed
    #reset PID values to eliminate bugs
    change = 0
    old_change = 0
    integral = 0
    #reset the driven distance of the robot to eliminate bugs
    drivenDistance = 0
    #specifies the color sensor
    colorsensor = ColorSensor(sensorPort)
    #Get degrees of motors turned before robot has moved, allows for distance calculation without resetting motors
    loop = True
    b_Startvalue = - leftMotor.get_degrees_counted()
    c_Startvalue = rightMotor.get_degrees_counted()
    #Going backwards is not supported on our robot due to the motors then being in front of the colour sensors and the program not working
    if distance < 0:
        print('ERR: distance < 0')
        distance = abs(distance)
    #Calculate target values for the motors to turn to
    rotateDistance = (distance / 17.6) * 360
    #Calculate after what distance the robot has to reach max speed
    accelerateDistance = rotateDistance * addspeed

    invert = 1

    #Calculation of steering factor, depending on which side of the line we are on
    if side == "left":
        invert = -1
    elif side == "right":
        invert = 1
    else:
        print("you need to specify the side of the line with left or right")
        hub.speaker.beep(100,1.5)
        return

    #Speed debugging
    if speed < 0:
        print("Driving backwards is currently not supported")
        hub.speaker.beep(100,1.5)

        return
    #Calculation of the start of the robot slowing down
    brakeStartValue = brakeStart * rotateDistance
    detectLineStartValue = detectLineStart * rotateDistance

    while loop:
        #Checks the driven distance as an average of both motors for increased accuracy
        oldDrivenDistance = drivenDistance
        drivenDistance = abs(((- leftMotor.get_degrees_counted() - b_Startvalue) + (rightMotor.get_degrees_counted() - c_Startvalue)) / 2)

        #Calculates target value for Robot as the edge of black and white lines
        old_change = change

        change = colorsensor.get_reflected_light() - 50

        #Steering factor calculation using PID, sets new I value

        steering = (((change * pReglerLight) + (integral * iReglerLight) + (dReglerLight * (change - old_change)))) * invert
        integral = change + integral
        #Calculation of current speed for robot, used for acceleratiion, braking etc.
        speed = speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance,)

        pidCalculationLight(speed)
        #PID value updates
        steering = max(-100, min(steering, 100))

        #Driving using speed values calculated with PID and acceleration for steering, use of distance check
        if speed > 0:
            movement_motors.start_at_power(- int(speed), int(steering))
            if rotateDistance < drivenDistance:
                loop = False
        else:
            movement_motors.start_at_power(- int(speed),-int(steering))
            if rotateDistance > drivenDistance:
                loop = False

        #Checking for line on specified sensor as end condition



        if detectLineStartValue <= drivenDistance:
            if driveToLinePort != 0:
                colorSensor = ColorSensor(driveToLinePort)
                if colorSensor.get_reflected_light() < lightValue:
                    loop = False

    movement_motors.stop()
    return

def gyroRotation(angle, startspeed, maxspeed, endspeed, addspeed, brakeStart, lightSensorPort = 0, lightValue = 0, variant = 0, detectLineStart = 0):
    """

        This is the function that we use to make the robot turn the length of a specific angle or for the robot to turn until it senses a line. Even in this function the robot
        can accelerate and slow down. It also has Gyrosensor calibrations based on our experimental experience.

        Parameters
        -------------
        angle: The angle which the robot is supposed to turn. Type: Integer. Default value: No default value

        speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value

        maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value

        endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value

        addspeed: The percentage after which the robot reaches the maxspeed. Type: Float. Default: No default value

        brakeStart: The percentage after which the robot starts slowing down until it reaches endspeed. Type: Float. Default: No default value

        lightSensorPort: The port the robot monitors for turning until it sees a colour there. Type: String. Default: 0

        lightValue: The value the robot has to see on lightSensorPort to stop. Type: Integer. Default: 0

        variant: Different turning types. 0: Both motors turn, robot turns on the spot. 1: Only the outer motor turns, resulting in a corner. Type: Integer. Default: 0

        detectLineStart: the point at which the robot starts looking for a line to stop at, in percent. Type: Float. Default: 0
    """

    if cancel:
        return


    speedMultiplier = 0.125

    if variant == 0:
        startspeed = abs(startspeed)
        maxspeed = abs(maxspeed)
        endspeed = abs(endspeed)
        speedMultiplier = 0.25

    speed = startspeed

    #set standard variables
    finalGyroValue = 0
    rotatedDistance = 0
    brakeStartValue = 0
    steering = 100
    hub.motion_sensor.reset_yaw_angle() #Yaw angle used due to orientation of our hub. This might need to be changed

    accelerateDistance = abs(angle * addspeed)
    deccelerateDistance = abs(angle *(1 - brakeStart))

    #gyro sensor calibration
    #336/360 is experimental value based on turning the robot for 10 rotations
    angle = angle * (336/360)

    #Setting variables based on inputs
    loop = True
    gyroStartValue = hub.motion_sensor.get_yaw_angle() #Yaw angle used due to orientation of the hub. This might need to be changed
    finalGyroValue = gyroStartValue + angle
    brakeStartValue = angle * brakeStart + gyroStartValue

    #Inversion of steering value for turning counter clockwise
    if angle < 0:
        steering = steering * -1

    #Testing to see if turining is necessary, turns until loop = False

    detectLineStartValue = detectLineStart * angle

    while loop:
        rotatedDistance = hub.motion_sensor.get_yaw_angle() #Yaw angle used due to orientation of the hub. This might need to be changed


        #Calculation of speed values for positive speeds
        addspeedPerDegree = speedMultiplier * (maxspeed - startspeed) / accelerateDistance

        subSpeedPerDegree = speedMultiplier * (maxspeed - endspeed) / deccelerateDistance

        if speed > 0:
            if angle > 0:
                #Calculation and detection of brake speed
                if rotatedDistance > brakeStartValue and speed > endspeed:
                    speed = speed - subSpeedPerDegree

                #Acceleration during acceleration phase
                elif speed < maxspeed:
                    speed = speed + addspeedPerDegree

        #Calculation of speed values for positive speeds
            else:
                #Calculation and detection of brake speed
                if rotatedDistance < brakeStartValue and speed > endspeed:
                    speed = speed - subSpeedPerDegree
                #Acceleration during acceleration phase
                elif speed < maxspeed:
                    speed = speed + addspeedPerDegree
            #Calculation of speed values for positive speeds

        else:
            if angle > 0:
            #Calculation and detection of brake speedy
                if rotatedDistance > brakeStartValue and speed < endspeed:
                    speed = speed - subSpeedPerDegree
            #Acceleration during acceleration phase
                elif speed > maxspeed:
                    speed = speed + addspeedPerDegree
            else:

            #Calculation and detection of brake speed
                if rotatedDistance < brakeStartValue and speed < endspeed:
                    speed = speed - subSpeedPerDegree
            #Acceleration during acceleration phase

                elif speed > maxspeed:
                    speed = speed + addspeedPerDegree


        if angle > 0:
            #Ends rotation because robot has reached the desired angle while turning clockwise
            if finalGyroValue < rotatedDistance:
                loop = False

        else:
            #Ends rotation baecuase robot has reached the desired angle while turning counterclockwise
            if finalGyroValue > rotatedDistance:
                loop = False


        #Checking for variants
        #Both Motors turn, robot moves on the spot
        if variant == 0:
            movement_motors.start_at_power(int(speed), steering)
        #Only outer motor turns, robot has a wide turning radius
        elif variant == 1:
            if angle > 0:
                if speed > 0:
                    leftMotor.start_at_power(- int(speed))
                else:
                    rightMotor.start_at_power(+ int(speed))
            else:
                if speed > 0:
                    rightMotor.start_at_power( int(speed))
                else:
                    leftMotor.start_at_power(- int(speed))

        #Sets Colour sensor checking for line
        if abs(detectLineStartValue) < abs(rotatedDistance):
            if lightSensorPort != 0:
                colorSensor = ColorSensor(lightSensorPort)

                #Stops robot if colour sensor detects desired colour
                if colorSensor.get_reflected_light() < lightValue:
                    loop = False
    #Stops movement motors for increased accuracy while stopping
    movement_motors.stop()
    return # End of gyroStraightDrive

def gyroStraightDrive(distance, startspeed, maxspeed, endspeed, addspeed, brakeStart, port = 0, lightValue = 0, variant = 0, detectLineStart = 0, offset = 0, generator = 0):
    """
        This is the function that we use to make the robot go forwards or backwards without drifting. It can accelerate, it can slow down and there's also PID. You can set the values
        in a way where you can either drive until the entered distance has been achieved or until the robot senses a line.

        Parameters
        -------------
        distance: the distance that the robot is supposed to drive. Type: Integer. Default: No default value

        speed: The speed which the robot is supposed to start at. Type: Integer. Default: No default value

        maxspeed: The highest speed at which the robot drives. Type: Integer. Default: No default value

        endspeed: The speed which the robot achieves at the end of the function. Type: Integer. Default: No default value

        addspeed: The speed which the robot adds in order to accelerate. Type: Float. Default: No default value

        brakeStart: The value which we use to tell the robot after what percentage of the distance we need to slow down. Type: Float. Default: No default value

        port: This value tells the program whether the robot is supposed to check for a black line with the specified light snsor. Type: String. Default: 0

        lightValue: This value tells the program the value the robot should stop at if port sees it. Type: Integer. Default: 0

        variant: Tells the robot to align itself to a line if it sees one. 0: No alignment. 1: alignment. Type: Integer. Default: 0

        detectLineStart: The value which we use to tell the robot after what percentage of the distance we need to look for the line to drive to. Type: Float. Default: 0

        offset: The value sends the robot in a direction which is indicated by the value entered. Type: Integer. Default: 0

        generator: Function executed while robot is executing gyroStraightDrive. Write the wanted function and its parameters here. Type: . Default: 0
    """

    #Debugging of speed values so that robot always reaches end. Based on experimental values
    if (startspeed < 17 and startspeed > 0):
        print("startspeed should never be less than 17")
        startspeed = 17
    if(startspeed > -17 and startspeed < 0):
        print("startspeed should never be less than 17")
        startspeed = -17
    if (endspeed < 17 and endspeed > 0):
        print("startspeed should never be less than 17")
        #endspeed = 17
    if(endspeed > -17 and endspeed < 0):
        print("startspeed should never be less than 17")
        #endspeed = -17

    if cancel:
        return

    global run_generator
    global runSmall

    if generator == 0:
        run_generator = False
    #Set starting speed of robot
    speed = startspeed

    #Sets PID values
    change = 0
    old_change = 0
    integral = 0
    drivenDistance = 0

    #Sets values based on user inputs
    loop = True
    b_Startvalue = - leftMotor.get_degrees_counted()
    c_Startvalue = rightMotor.get_degrees_counted()
    hub.motion_sensor.reset_yaw_angle() #Yaw angle used due to orientation of the hub. This might need to be changed

    #Error check for distance
    if distance < 0:
        print('ERR: distance < 0')
        distance = abs(distance)

    #Calulation of degrees the motors should turn to
    #17.6 is wheel circumference in cm. You might need to adapt it
    rotateDistance = (distance / 17.6) * 360
    accelerateDistance = rotateDistance * addspeed



    #Inversion of target rotation value for negative values
    if speed < 0:
        rotateDistance = rotateDistance * -1

    #Calculation of braking point
    brakeStartValue = brakeStart * rotateDistance
    detectLineStartValue = detectLineStart * rotateDistance

    #Robot drives while loop = true

    while loop:

        if run_generator: #run parallel code execution
            next(generator)


        if breakFunction() or not loop:
            break

        #Calculation of driven distance and PID values
        oldDrivenDistance = drivenDistance
        drivenDistance = ((- leftMotor.get_degrees_counted() - b_Startvalue) + (rightMotor.get_degrees_counted() - c_Startvalue)) / 2

        change = hub.motion_sensor.get_yaw_angle() #yaw angle used due to orientation of the hub
        steering = ((change * pRegler) + (integral * iRegler) + (dRegler * (change - old_change))) + offset
        steering = max(-100, min(steering, 100))

        integral = change + integral
        old_change = change

        #Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight
        speed = speedCalculation(speed, startspeed,maxspeed, endspeed, accelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance)
        pidCalculation(speed)

        #Alignment based on variant
        if variant == 1:
            #Colour sensor E sees light first and triggers
            if colorE.get_reflected_light() < lightValue:


                rightMotor.stop()
                #Turning robot so that other colour sensor is over line
                while True:
                    leftMotor.start_at_power(- int(speed))

                    #Line detection and stopping
                    if colorF.get_reflected_light() < lightValue or breakFunction():
                        leftMotor.stop()
                        loop = False
                        break

            #Colour sensor F sees line first
            elif colorF.get_reflected_light() < lightValue:

                leftMotor.stop()
                #Turning robot so that other colour sensor is over line
                while True:
                    rightMotor.start_at_power(int(speed))

                    #Line detection and stopping
                    if colorE.get_reflected_light() < lightValue or breakFunction():
                        rightMotor.stop()
                        loop = False
                        break

            #Second ending condition: specified distance has been reached and line wasn't detected
            else:
                if speed > 0:
                    rightMotor.start_at_power(int(speed))
                    leftMotor.start_at_power(- int(speed))
                    if rotateDistance < drivenDistance:
                        loop = False
                else:
                    rightMotor.start_at_power(int(speed))
                    leftMotor.start_at_power(- int(speed))

                    if rotateDistance > drivenDistance:
                        loop = False

        else:
            #Sets colour sensor port to port specified by user

            if speed > 0:
                if detectLineStartValue < drivenDistance:
                    if port != 0:
                        colorSensor = ColorSensor(port)

                        if lightValue < 50:
                            #Stops loop based on colour detection
                            if colorSensor.get_reflected_light() < lightValue:
                                loop = False
                        else:
                            #Stops loop based on colour detection
                            if colorSensor.get_reflected_light() > lightValue:
                                loop = False


                #Calculation of speed and steering values
                movement_motors.start_at_power(- int(speed), int(steering))
                if rotateDistance < drivenDistance:
                    loop = False
            else:
                if detectLineStartValue > drivenDistance:
                    if port != 0:
                        colorSensor = ColorSensor(port)

                        #Stops loop based on colour detection
                        if colorSensor.get_reflected_light() < lightValue:
                            movement_motors.stop()
                            loop = False
                            break

                #Calculation of speed and steering values, inversion of steering value for driving backwards
                movement_motors.start_at_power(- int(speed), - int(steering))

                if rotateDistance > drivenDistance:
                    movement_motors.stop()
                    loop = False
                    break


    #Stopping movement motors at the end of program for increased accuracy
    movement_motors.stop()
    smallMotorA.stop()
    smallMotorD.stop()
    run_generator = True
    runSmall = True

    generator = 0
    return #End of gyroStraightDrive

def speedCalculation(speed, startspeed, maxspeed, endspeed, accelerateDistance, brakeStartValue, drivenDistance, oldDrivenDistance):
    """
        Used to calculate all the speeds in out programs. Done seperatly to reduce redundancy. Brakes and accelerates

        Parameters
        -------------
        speed: The current speed the robot has

        startspeed: Speed the robot starts at. Type: Integer. Default: No default value.

        maxspeed: The maximum speed the robot reaches. Type: Integer. Default: No default value.

        endspeed: Speed the robot aims for while braking, minimum speed at the end of the program. Type: Integer. Default: No default value.

        addspeed: Percentage of the distance after which the robot reaches the maximum speed. Type: Integer. Default: No default value.

        brakeStartValue: Percentage of the driven distance after which the robot starts braking. Type: Integer. Default: No default value.

        drivenDistance: Calculation of the driven distance in degrees. Type: Integer. Default: No default value.
    """

    if cancel:
        return
    #Calculation of speed values for positive speeds
    if speed > 0:
        if drivenDistance > 0:
            #Calculation and detection of brake speed
            if drivenDistance > brakeStartValue and speed > endspeed:
                subSpeedPerDegree = (maxspeed - endspeed) / accelerateDistance
                speed = speed - (drivenDistance - oldDrivenDistance) * subSpeedPerDegree
            #Acceleration during acceleration phase
            elif speed < maxspeed:
                addspeedPerDegree = (maxspeed - startspeed) / accelerateDistance

                speed = speed + (drivenDistance - oldDrivenDistance) * addspeedPerDegree
        #Calculation of speed values for positive speeds
        else:
            #Calculation and detection of brake speed
            if drivenDistance < brakeStartValue and speed > endspeed:
                subSpeedPerDegree = (maxspeed - endspeed) / accelerateDistance

                speed = speed - (drivenDistance - oldDrivenDistance) * subSpeedPerDegree
            #Acceleration during acceleration phase
            elif speed < maxspeed:
                addspeedPerDegree = (maxspeed - startspeed) / accelerateDistance

                speed = speed + (drivenDistance - oldDrivenDistance) * addspeedPerDegree
            #Calculation of speed values for positive speeds

    else:
        if drivenDistance > 0:
            #Calculation and detection of brake speed
            if drivenDistance > brakeStartValue and speed < endspeed:
                subSpeedPerDegree = (maxspeed - endspeed) / accelerateDistance
                speed = speed + (drivenDistance - oldDrivenDistance) * subSpeedPerDegree
            #Acceleration during acceleration phase
            elif speed > maxspeed:
                addspeedPerDegree = (maxspeed - startspeed) / accelerateDistance
                speed = speed - (drivenDistance - oldDrivenDistance) * addspeedPerDegree
        else:

            #Calculation and detection of brake speed
            if drivenDistance < brakeStartValue and speed < endspeed:
                subSpeedPerDegree = (maxspeed - endspeed) / accelerateDistance

                #speed = speed + (drivenDistance - oldDrivenDistance) * subSpeedPerDegree
            #Acceleration during acceleration phase


            elif speed > maxspeed:
                addspeedPerDegree = (maxspeed - startspeed) / accelerateDistance
                speed = speed - (drivenDistance - oldDrivenDistance) * addspeedPerDegree

    return speed

def breakFunction():
    return False

def driveMotor(rotations, speed, port):
    """
    Allows you to drive a small motor in parallel to driving with gyroStraightDrive

    Parameters
    -------------
    rotations: the rotations the motor turns

    speed: the speed at which the motor turns

    port: the motor used. Note: this cannot be the same motors as configured in the motor Drivebase
    """
    global runSmall
    global run_generator
    while runSmall:
        smallMotor = Motor(port)
        smallMotor.set_degrees_counted(0)

        loop_small = True
        while loop_small:
            drivenDistance = smallMotor.get_degrees_counted()
            smallMotor.start_at_power(speed)
            if (abs(drivenDistance) > abs(rotations) * 360):
                loop_small = False
            yield

        smallMotor.stop()
        runSmall = False
        run_generator = False

    yield

def pidCalculation(speed):
    #golbally sets PID values based on current speed of the robot, allows for fast and accurate driving
    global pRegler
    global iRegler
    global dRegler
    #Important note: These PID values are experimental and based on our design for the robot. You will need to adjust them manually. You can also set them statically as you can see below
    if speed > 0:
        pRegler = -0.17 * speed + 12.83
        iRegler = 0.0006 * speed - 0.015
        dRegler = 1.94 * speed - 51.9
        if pRegler < 3.2:
            pRegler = 3.2
    else:
        pRegler = 2
        iRegler = 0
        dRegler = 10

def pidCalculationLight(speed):
    #Sets the PID values for the lineFollower based on current speed. Allows for accurate and fast driving
    #Important note: these PID values are experimental and based on our design for the robot. You will need to adjust them. See above on how to do so
    global pReglerLight
    global dReglerLight

    pReglerLight = -0.04 * speed + 4.11
    dReglerLight = 0.98 * speed - 34.2
    #set hard bottom for d value, as otherwise the values don't work
    if dReglerLight < 5:
        dReglerLight = 5



"""
This is an introduction on how to use the driving programs. For accurate driving you will need to adjust the PID values to fit your
robot and speeds you go at. There are tutorials online on how to do this. (For example see: https://www.youtube.com/watch?v=AMBWV_HGYj4 )
We recommend that you try to align yourself both with lines but also mechanically for example with walls or mission tasks on the mat.
"""

def example1(): #How to use the gyro straight drive
    gyroStraightDrive(20, 20, 45, 20, 0.2, 0.8)
    """
    Most basic driving. The robot will go in a straight line for 20 centimeters if you adjusted the wheel circumference in the gyroStraighDrive.
    The robot will start moving at a speed of 20, accelerate until it reaches a speed of 45 after 20% of the distance travelled and will start
    slowing down back to 20 after 80% of the distance has been travelled.
    By default all over values are set to 0 to reduce the amount of code
    """
    hub.left_button.wait_until_pressed() #Waits to execute next function until user presses left button
    gyroStraightDrive(20,20,45,20,0.2, 0.8, "E", 25)
    """
    This is the same as before, but this time the robot stops when the light sensor on port E sees a light value lower than 25, in this case a black line.
    The port can be adapted to any light sensor, as can the value. If it is under 50, the robot will stop at light values lower than 50, if it is above 50
    then the robot looks for a light value greater than 50, for example a white line
    """
    hub.left_button.wait_until_pressed() #Waits to execute next function until user presses left button
    gyroStraightDrive(20, 20, 45, 20, 0.2, 0.8, 0,25, 1)
    """
    This is the same as the first gyroStraightDrive but the robot stops as soon as it sees a line, it doesn't matter on which port. As soon as the robot has
    stopped, it drives the corresponding motor until the other light sensor also sees the specified light value. Be careful here, as internal lag can cause
    the robot to overshoot the line, so make sure that you set your speed values accordingly
    """
    hub.left_button.wait_until_pressed() #Waits to execute next function until user presses left button
    gyroStraightDrive(20, 20, 45, 20, 0.2, 0.8, "E", 25, detectLineStart=0.6)
    """
    The robot will drive until it sees a line, as in the second gyroStraightDrive. However it only starts checking for the specified light value after 80% of the
    distance has been driven. This is very useful if you want to skip lines but still drive with one function
    """
    hub.left_button.wait_until_pressed() #Waits to execute next function until user presses left button
    gyroStraightDrive(20, 20, 45, 20, 0.2, 0.8, offset=20)
    """
    This is the same as the basic driving of the first gyroStraightDrive. However the robot won't go straight but aim for a target value of 20
    """
    hub.left_button.wait_until_pressed() #Waits to execute next function until user presses left button
    generator = driveMotor(5, 100, 'A') #Prepares parallel code execution to be run in next gyroStraightDrive
    gyroStraightDrive(20, 20, 45, 20, 0.2, 0.8,generator=generator)
    """
    This is the same as the basic driving of the first gyroStraightDrive. However while it drives the robot also turns the "A" motor for 5 rotations.
    Because of the weak single core CPU of the robot, this isn't natively supported and the calculation takes about half a second, during which
    the robot waits. this can be avoided by giving every single generator a unique name and letting them calculate before the actual program starts.
    """
    return

def example2(): #How to use the line follower
    lineFollower(20, 20, 40, 20, 0.2, 0.8, "left", "E")
    """
    This is the most basic line follower. The robot will follow the line for 20 centimeters if you adjusted the wheel circumference in the line follower.
    The robot will start moving at a speed of 20, accelerate until it reaches a speed of 45 after 20% of the distance travelled and will start
    slowing down back to 20 after 80% of the distance has been travelled.
    You have to specifiy the side of the line you want to go on, as this program keeps the robot on the edge of the line. This program is meant to be used
    when the edge of the line is black and white, though you can adjust the target light value as needed. The port specified, in this case "E" states which port
    the robot uses to follow the line.
    """
    hub.left_button.wait_until_pressed() #Waits to execute next function until user presses left button
    lineFollower(20, 20, 40, 20, 0.2, 0.8, "left", "E", "F", 25)
    """
    This is the same as the first lineFollower except for the ending condition. This time, the robot stops when the "F" light sensor sees a value below 25. This is
    exactly the same as it is on the gyroStraightDrive. The driveToLinePort cannot be the following port, as this port is already in use for following the line.
    """
    hub.left_button.wait_until_pressed() #Waits to execute next function until user presses left button
    lineFollower(20, 20, 40, 20, 0.2, 0.8, "left", "E", "F", 25, 0.6)
    """
    This line follower is again similar to the gyroStraightDrive. The robot starts looking for the line after 60% of the distance has been travelled.
    """
    #parallel motor execution isn't implemented yet in the line follower, this feature will be patched in later though
    return

def example3(): #How to use the gyro rotation
    gyroRotation(90, 20, 35, 20, 0.2, 0.8)
    """
    This is the most basic gyroRotation. The robot will turn clockwise 90°. It will start at a speed of 20 reach 35 after 20% of the angle has been reached and will 
    start slowing down to 20 after 80% of the angle haas been turned. Using this variant, the robot will turn on the spot.
    All other variables are set to be 0 to reduce the amount of code
    """
    gyroRotation(90, 20, 35, 20, 0.2, 0.8, "E", 30)
    """
    This is the same gyroRotation as in the first example. However the robot will also check for a light value less than 30 on the "E" light sensor. It will stop when 
    it has reached one of the ending conditions
    """
    gyroRotation(90, 20, 35, 20, 0.2, 0.8, variant=1)
    """
    This is the same gyroRotation as in the first example. However, rather than turning on the spot it will only rotate the outer motor to turn in a curve like shape
    """
    gyroRotation(90, 20, 35, 20, 0.2, 0.8, "E", 30, detectLineStart=0.8)
    """
    This is the same gyroRotation as in the second example. However the robot will only start checking for the light value after 80% of the angle has been reached. 
    This is useful when the robot also sweeps over areas of the same color but you don't want the robot to stop there
    """
    return

def example4(): #Write your own programs here
    
    return
#shows the battery colours in different colours on the console if the voltage is low and the battery should be charged
class bcolors:
    BATTERY = '\033[32m'
    BATTERY_LOW = '\033[31m'

    ENDC = '\033[0m'

pRegler = 2.75
iRegler = 0
dRegler = 70




pReglerLight = 1.6
iReglerLight = 0.009
dReglerLight = 16

accelerate = True

print(dir(util.log))

log.log_to_file("test")


#Battery voltage printout in console for monitoring charge
if battery.voltage() < 8000:
    print(bcolors.BATTERY_LOW + "battery voltage is to low: " + str(battery.voltage()) + bcolors.ENDC)
else:
    print(bcolors.BATTERY + "battery voltage: " + str(battery.voltage()) + bcolors.ENDC)



#User Interface in Program for competition and instant program loading
program = True
programselect = 1 #Set the attachment the selection program starts on
hub.status_light.on('blue')
hub.light_matrix.write(programselect)
while program:
    cancel = False
    #Program selection
    if hub.right_button.is_pressed(): #press right button to cycle through programs. cycling back isn't supported yet, but we are working on reallocating the buttons in the file system
        wait_for_seconds(0.15) #waiting prevents a single button press to be registered as multiple clicks
        programselect = programselect + 1
        hub.light_matrix.write(programselect) #show current selcted program
        hub.speaker.beep(85, 0.1) #give audio feedback for user

        if programselect == 1:
            hub.status_light.on('blue')
        elif programselect == 2:
            hub.status_light.on('black')
        elif programselect == 3:
            hub.status_light.on('white')
        elif programselect == 4:
            hub.status_light.on("red")
        if programselect == 5: #Add more slots as necessary, you will need to match the amount of slots to the starting slots
            programselect = 1
            hub.light_matrix.write(programselect)
            hub.status_light.on('blue')

    #Program start
    if hub.left_button.is_pressed(): #press left button to start the selected program
        hub.speaker.beep(85, 0.1)
        if programselect == 1: #starts first program
            hub.light_matrix.show_image("DUCK")
            example1()
            programselect = 2
            hub.status_light.on("black")
            hub.light_matrix.write(programselect)
        elif programselect == 2: #starts second program
            hub.light_matrix.show_image("DUCK")
            example2()
            programselect = 3
            hub.status_light.on("white")
            hub.light_matrix.write(programselect)
        elif programselect == 3: #starts third program
            hub.light_matrix.show_image("DUCK")
            example3()
            hub.light_matrix.write(programselect)
        elif programselect == 4: #starts fourth program
            hub.light_matrix.show_image("DUCK")
            example4()
            hub.light_matrix.write(programselect) #add more slots for programs as you wish
programselect=0 #reset variable to prevent bugs with multiple runs

sys.exit("ended program successfully")
