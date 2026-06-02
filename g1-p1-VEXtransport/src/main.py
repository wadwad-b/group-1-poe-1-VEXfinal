# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Brenden D'Souza, Harsh Patel, Michael Martini                #
# 	Created:      6/2/2026, 9:14:41 AM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

#----------------- Robot Configuration Code -----------------#
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)  # Right drivetrain
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)    # Left drivetrain
# Set the leftMotor to reversed so that when driving, it moves in the same direction as the right motor

drivetrain = DriveTrain(leftMotor, rightMotor)                  # Start both motors simultaneously
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)   # Liftarm motor
inertial_1 = Inertial(Ports.PORT5)                              # Inertial sensor
bumpSwitch = Bumper(brain.three_wire_port.a)                    # Bumper switch
#-------------------------------------------------------------#

#-----------------------Helper Functions----------------------#
def bump():
    """
    Hold the program's execution until the button is pressed
    """
    while bumpSwitch.pressing() == False:
        wait(10, MSEC) # Debounce the button (10 ms)
    
        brain.screen.set_cursor(1, 1) # Place cursor in upper left corner
        brain.screen.print("Press the button to start the program")
        pass

    brain.screen.clear_line(1) # Clear the text on row 1
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Program executed")
    wait(1, SECONDS) # Wait 1 second before proceeding

def inertialCalibration():
    """
    Calibrate the inertial sensor
    A wait time of 2 seconds is required
    This function should be called at the start of the program's execution
    """

    brain.screen.clear_screen() # Clear the brain's screen
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Calibrating the inertial sensor")
    brain.screen.set_cursor(2, 1)
    brain.screen.print("Don't move the robot!")
    inertial_1.calibrate() # Calibrate the inertial sensor

    wait(2, SECONDS) # Wait for the calibration to complete

    brain.screen.clear_line(1) # Clear the text on row 1
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Inertial calibration completed")

def testInertial():
    """
    Test the inertial sensor by having it display heading and total rotation data
    Pressing the bump switch will end the test
    """

    brain.screen.clear_screen()
    while bumpSwitch.pressing() == False:
        wait(10, MSEC) # Debounce the button (10 ms)
        brain.screen.set_cursor(5, 1)
        brain.screen.print("Heading: " + str(inertial_1.heading()))
        brain.screen.set_cursor(6, 1)
        brain.screen.print("Total Rotation: " + str(inertial_1.rotation()))

        brain.screen.set_cursor(8, 1)
        brain.screen.print("Press the button to exit")
    
    brain.screen.clear_row(8)
    brain.screen.set_cursor(8, 1)
    brain.screen.print("Inertial test terminated")

def driveStraightData(e):
    """
    1. Report position, rotation, and error values to the screen while driving
    2. Parameter: e = error value (setpoint - rotation)
    """

    brain.screen.set_cursor(1, 1)
    brain.screen.print("Position: " + str(leftMotor.position()))    # Return the current motor count

    brain.screen.set_cursor(2, 1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation()))   # Return the current rotation

    brain.screen.set_cursor(3, 1)
    brain.screen.print("Error: " + str(e))                          # Return the error

def stopMotors():
    """
    Stop both motors at the same time
    """
    drivetrain.stop()   # Stop both motors
    wait(0.5, SECONDS)  # Wait 0.5 seconds for the system to stabilize

def accelerate(currentVelocity, targetVelocity, step):
    """
    Gradually accelerate the motors to the target velocity by increasing the velocity by the acceleration step 
    """
    currentVelocity += step
    if abs(currentVelocity) > abs(targetVelocity):
        currentVelocity = targetVelocity

    return currentVelocity


def driveStraight(distance, setpoint, targetVelocity):
    """
    1. distance = distance to travel in inches
    2. setpoint = 0-degrees
    3. targetVelocity = the target velocity of the motors (+) => forward, (-) => backward
    """

    motorVelocity = 0;

    inertial_1.reset_rotation() # Reset the rotation before each driving action

    # Set stopping mode for the motors

    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    kP = 0.37   # Proportional constant for driving straight (used to calculate correction term to maintain course)
                # If too small, correction will occur to slowly
                # If too large, overcorrection will occur
                # Determine best value by iteratively testing

    wheelDiameter = 4   # Wheel diameter = 4 inches

    # Calculate the distance in terms of encoder ticks (1 tick = 1 degree of rotation)
    # Distance (ticks) = (Distance in inches / Wheel Circumference) * 360
    wheelCircumference = wheelDiameter * math.pi
    distance = (distance / wheelCircumference) * 360

    # Reset the motor encoder count to zero before drivinge
    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    # Drive forward if target velocity > 0
    if targetVelocity > 0:
        # while loop to track the distance traveled
        while leftMotor.position() < distance:
            error = setpoint - inertial_1.rotation()    # Calculate error
            correction = kP * error                     # Motor velocity correction

            # Correct motor velocities
            # If error > 0 (setpoint > rotation) => drifting left
            # If error < 0 (setpoint < rotation) => drifting right
            if motorVelocity < targetVelocity:
                motorVelocity = accelerate(motorVelocity, targetVelocity, 0.1) # Accelerate
            leftMotor.set_velocity(motorVelocity + correction, PERCENT)
            rightMotor.set_velocity(motorVelocity - correction, PERCENT)

            # Spin motors
            drivetrain.drive(FORWARD)

            driveStraightData(error) # Display data on the brain's screen
    
        # Stop motors when the desired distance is reached
        stopMotors()
    
    # Drive straight in reverse if target velocity < 0
    else:

        distance *= -1 # Make distance positive for reverse driving
        # while loop to track the distance traveled
        while leftMotor.position() > distance:
            error = setpoint - inertial_1.rotation()    # Calculate error
            correction = kP * error                     # Motor velocity correction

            # Correct motor velocities
            # If error > 0 (setpoint > rotation) => drifting left
            # If error < 0 (setpoint < rotation) => drifting right
            if motorVelocity > targetVelocity:
                motorVelocity = accelerate(motorVelocity, targetVelocity, -0.1) # Accelerate
            leftMotor.set_velocity(motorVelocity + correction, PERCENT)
            rightMotor.set_velocity(motorVelocity - correction, PERCENT)

            # Spin motors
            drivetrain.drive(FORWARD)

            driveStraightData(error) # Display data on the brain's screen
    
        # Stop motors when the desired distance is reached
        stopMotors()

def turnData(turnError, derivative):
    """
    Print the current heading, turning error, and derivative values
    """

    brain.screen.set_cursor(1, 1)
    brain.screen.print("Heading: " + str(inertial_1.heading())) # Return the current heading
    
    brain.screen.set_cursor(2, 1)
    brain.screen.print("Error: " + str(abs(turnError)))         # Return the current turning error

    brain.screen.set_cursor(3, 1)
    brain.screen.print("Derivative: " + str(abs(derivative)))   # Return the current derivative

def pointTurn(setpoint):
    """
    1. Perform a point turn using the inertial sensor and proportional and derivative control
    2. Argument: desired heading (setpoint)
    """

    brain.screen.clear_screen() # Clear the brain's screen


    # Set stopping mode for turn
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    # Calculate the difference between setpoint and current heading
    difference = setpoint - inertial_1.heading()

    # Want to minimize the amount of turn required

    if setpoint > inertial_1.heading():     
        if abs(difference) <= 180:          # Turn CW
            clockwise = True
        else:                               # Turn CCW
            clockwise = False 
    else: 
        if abs(difference) <= 180:          # Turn CCW
            clockwise = False
        else:                               # Turn CW
            clockwise = True  

    # Define kP and kD values for the CW and CCW turns
    if clockwise:       # Values if clockwise
        kP = 0.04
        kD = 0.08
    else:               # Values if counterclockwise
        kP = 0.045
        kD = 0.016

    # Define maxiumm velocity and previous error terms
    maxVelocity = 50        # Units: %
    previousError = 0.0     # Error from previous iteration of the control loop

    while True:
        turnError = setpoint - inertial_1.heading()
        derivative = turnError - previousError

        # Stop the motors and exit the control loop when the error and derivative terms are sufficiently small to ensure setpoint was reached without oscillation
        if abs(turnError) < 1 and abs(derivative) < 0.2:
            stopMotors()    # Stop motors
            break           # Leave the loop

        # Proportional and derivative correction calculations
        turnCorrection = (kP * turnError) + (kD * derivative)

        # Limit the corrective term to make sure we don't exceed the maximum velocity

        if abs(turnCorrection) > 1:
            turnCorrection = 1

        turnVelocity = turnCorrection * maxVelocity

        # Set the motor velocities to spin in the correction directions
        if clockwise:
            leftMotor.set_velocity(turnVelocity, PERCENT)
            rightMotor.set_velocity(-turnVelocity, PERCENT)
        else:
            leftMotor.set_velocity(-turnVelocity, PERCENT)
            rightMotor.set_velocity(turnVelocity, PERCENT)
        
        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        turnData(turnError, derivative) # Print heading, error, and derivative values

        previousError = turnError # Update previous error term       

        wait(20, MSEC) # Small delay to prevent overloading the brain with data

def liftArm(motorVelocity, liftAngle):
    # Configure the motor to hold its position
    liftMotor.set_stopping(HOLD)

    liftMotor.set_velocity(motorVelocity, PERCENT) # Set lift arm motor velocity

    gearRatio = 5 # 60T to 12T
    motorAngularDisplacement = liftAngle * gearRatio # Calculate the motor axle's required displacement

    # Spin motor forward for the given angular displacement in degrees
    liftMotor.spin_for(FORWARD, motorAngularDisplacement, DEGREES)
    wait(0.5, SECONDS)


#-------------------Define main() Function---------------------#
def main():
    """
    The main() function is the function that will be executed by the brain
    """
    bump()                      # Call bump() to execute the program
    inertialCalibration()       # Calibrate the inertial sensor

    driveStraight(94, 0, 100)   # Test drive straight function (distance in inches, setpoint, target velocity in %)

main()