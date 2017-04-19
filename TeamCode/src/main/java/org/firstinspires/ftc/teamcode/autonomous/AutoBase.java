package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainRobotBase;
import org.firstinspires.ftc.teamcode.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.hardware.NiFTColorSensor;
import org.firstinspires.ftc.teamcode.hardware.NiFTGyroSensor;
import org.firstinspires.ftc.teamcode.hardware.NiFTRangeSensor;
import org.firstinspires.ftc.teamcode.threads.NiFTFlow;


//For added simplicity while coding autonomous with the new FTC system. Utilizes inheritance and polymorphism.
public abstract class AutoBase extends MainRobotBase
{
    /******** SENSOR STUFF ********/

    /**** Color Sensors (3) ****/
    protected NiFTColorSensor option1ColorSensor, option2ColorSensor, bottomColorSensor, particleColorSensor; //Must have different I2C addresses.
    protected boolean option1Red, option2Red, option1Blue, option2Blue;

    protected void updateColorSensorStates ()
    {
        //Threshold is currently 2, but this could be changed.
        option1Blue = option1ColorSensor.sensor.blue () >= 2;
        option1Red = option1ColorSensor.sensor.red () >= 1 && !option1Blue; //Since blue has an annoying tendency to see red and blue color values.
        option2Blue = option2ColorSensor.sensor.blue () >= 2;
        option2Red = option2ColorSensor.sensor.red () >= 1 && !option2Blue;
    }


    /**** Gyro ****/
    protected NiFTGyroSensor gyroscope;

    //Used to turn to a specified heading, and returns the difference between the desired angle and the actual angle achieved.
    protected enum TurnMode
    {
        LEFT, RIGHT, BOTH
    }

    //This battery factor, when updated, remains updated for all future turns so that the robot does not have to start changing it again.
    protected void turnToHeading (int desiredHeading, TurnMode mode, long maxTime) throws InterruptedException
    {
        //Create a turn process console.
        NiFTConsole.ProcessConsole turnConsole = new NiFTConsole.ProcessConsole ("Turning");

        double turnCoefficient = 0.0042, turnIntercept = 0.2, initialTurnIntercept = turnIntercept;

        gyroscope.setDesiredHeading (desiredHeading);

        //Get the startTime so that we know when to end.
        long startTime = System.currentTimeMillis ();
        int priorHeading = gyroscope.getValidGyroHeading ();
        long lastCheckedTime = startTime;

        int currentHeading = priorHeading;
        //Adjust as fully as possible but not beyond the time limit.
        while (opModeIsActive () && (System.currentTimeMillis () - startTime < maxTime || Math.abs (currentHeading - desiredHeading) >= 10))
        {
            //Verify that the heading that we thought was perfectly on point actually is on point.
            if (gyroscope.getOffFromHeading () == 0)
            {
                hardBrake (100);
                //Verify that it really is at the correct heading (like never happens ever) and if it really was, then try something else.
                if (gyroscope.getOffFromHeading () == 0)
                    break;
            }

            /*** Verify that we are turning ****/
            currentHeading = gyroscope.getValidGyroHeading ();

            //Protection against stalling, increases power if no observed heading change in last fraction of a second.
            if (System.currentTimeMillis () - lastCheckedTime >= 300 && (System.currentTimeMillis () - startTime) > 1000)
            {
                int headingChange = Math.abs (priorHeading - currentHeading);
                //Don't start increasing power at the very start of the turn before the robot has had time to accelerate.
                if (headingChange <= 1)
                    turnIntercept += 0.06;
                else if (headingChange >= 7 && turnIntercept > initialTurnIntercept)
                    turnIntercept -= 0.03;

                //Update other variables.
                priorHeading = currentHeading;
                lastCheckedTime = System.currentTimeMillis ();
            }

            /**** turn at some power. ****/
            //Turn at a speed proportional to the distance from the ideal heading.
            int thetaFromHeading = desiredHeading - currentHeading;

            double turnPower = Math.signum (thetaFromHeading) * (Math.abs (thetaFromHeading) * turnCoefficient + turnIntercept);

            //Set clipped powers.
            if (mode != TurnMode.RIGHT)
                leftDrive.setDirectMotorPower (Range.clip (turnPower, -1, 1));
            if (mode != TurnMode.LEFT)
                rightDrive.setDirectMotorPower (-1 * Range.clip (turnPower, -1, 1));

            idle ();
            //Output required data.
            turnConsole.updateWith (
                    "Turning to heading " + gyroscope,
                    "Current heading = " + currentHeading,
                    "Turn Power is " + turnPower,
                    "Turn intercept is " + turnIntercept,
                    "I have " + (maxTime - (System.currentTimeMillis () - startTime)) + "ms left.",
                    "Turn coefficient = " + turnCoefficient,
                    "Min turn speed = " + turnIntercept
            );
        }

        turnConsole.destroy ();

        hardBrake (100);
    }


    /**** Encoders ****/
    //Since this method takes a half-second or so to complete, try to run it as little as possible.
    protected void initializeAndResetEncoders () throws InterruptedException
    {
        leftDrive.resetEncoder ();
        rightDrive.resetEncoder ();
    }

    private int getEncoderPosition () throws InterruptedException
    {
        return (int) ((leftDrive.encoderMotor.getCurrentPosition () + rightDrive.encoderMotor.getCurrentPosition ()) / 2.0);
    }

    private long lastCheckTime = 0;
    private double previousPosition = 0;

    private double calculateEncoderAdjustment () throws InterruptedException
    {
        int drivenDistance = getEncoderPosition ();

        if ((System.currentTimeMillis () - lastCheckTime) >= 100)
        {
            if (Math.abs (drivenDistance - previousPosition) <= 2)
                return 0.05; //Suggest an increase in movement power if we haven't moved very far.

            previousPosition = drivenDistance;
            lastCheckTime = System.currentTimeMillis ();
        }

        return 0; //Don't suggest that we increase movement power.
    }

    /**** Range Sensor(s) ****/
    protected NiFTRangeSensor frontRangeSensor, sideRangeSensor;

    protected void driveUntilDistanceFromObstacle (double stopDistance, double minPower) throws InterruptedException
    {
        //Required variables.
        double lastValidDistance = 150 - stopDistance;
        double stopAndMoveAtMinPowerDist = 42;

        double distanceFromStop = lastValidDistance;
        while (distanceFromStop > stopAndMoveAtMinPowerDist)
        {
            //Scrub the input to get a valid result (255 is the default invalid value of the range sensors).
            double perceivedDistanceFromObstacle = frontRangeSensor.ultrasonicDistCM ();
            if (perceivedDistanceFromObstacle >= 255)
                perceivedDistanceFromObstacle = lastValidDistance;
            else
                lastValidDistance = perceivedDistanceFromObstacle;

            //Calculate the distance until stopping.
            distanceFromStop = perceivedDistanceFromObstacle - stopDistance;

            //Calculate the new movement power based on this result.
            //The (movement power - initial movement power) expression incorporates encoder adjustments in the event that the bot is not moving.
            //Linear trend downwards as we approach the obstacle.
            movementPower = distanceFromStop * 0.0067 + minPower;

            //Only use encoders.
            manuallyApplySensorAdjustments (true, true, false);
        }

        NiFTFlow.pauseForMS (100);
        stopDriving ();

        //Drive at min power the rest of the way.
        startDrivingAt (minPower);

        while (distanceFromStop > 0)
        {
            double perceivedDistanceFromObstacle = frontRangeSensor.sensor.cmUltrasonic ();
            if (perceivedDistanceFromObstacle >= 255)
                perceivedDistanceFromObstacle = lastValidDistance;
            else
                lastValidDistance = perceivedDistanceFromObstacle;

            //Calculate the distance until stopping.
            distanceFromStop = perceivedDistanceFromObstacle - stopDistance;

            //Only use encoders.
            manuallyApplySensorAdjustments (true, true, false);
        }

        NiFTFlow.pauseForMS (100);
        stopDriving ();
    }

    /******** MOVEMENT POWER CONTROL ********/
    //Used to set drive move power initially.
    protected double movementPower = 0;
    protected void startDrivingAt (double movementPower)
    {
        this.movementPower = movementPower;

        leftDrive.setDirectMotorPower (movementPower);
        rightDrive.setDirectMotorPower (movementPower);

        encoderDriveSpeedBoost = 0;
    }

    //Stops all drive motors.
    protected void stopDriving ()
    {
        leftDrive.setDirectMotorPower (0);
        rightDrive.setDirectMotorPower (0);
    }

    //Stops all drive motors and pauses for a moment
    protected void hardBrake (long msDelay) throws InterruptedException
    {
        stopDriving ();
        NiFTFlow.pauseForMS (msDelay);
    }

    private double encoderDriveSpeedBoost = 0;
    protected void manuallyApplySensorAdjustments (boolean... adjustments) throws InterruptedException
    {
        if (adjustments != null)
        {
            //Will be used multiple times.
            int movementPowerSign = (int) (Math.signum (movementPower));

            //Calculate the required sensor adjustments based on the parameters.
            if (adjustments.length >= 2 && adjustments[1])
                encoderDriveSpeedBoost += calculateEncoderAdjustment ();
            else if (encoderDriveSpeedBoost > 0)
                encoderDriveSpeedBoost = 0;

            double actualMovementPower = movementPower + movementPowerSign * encoderDriveSpeedBoost;

            //For each result, positive favors left side and negative the right side.
            double gyroAdjustment = 0;
            if (adjustments.length >= 1 && adjustments[0])
            {
                int offFromHeading = gyroscope.getOffFromHeading ();
                gyroAdjustment = Math.signum (offFromHeading) * (Math.abs (offFromHeading) * .006 + .22);
            }

            double rangeSensorAdjustment = 0;
            if (adjustments.length >= 3 && adjustments[2])
            {
                double rangeSensorReading = sideRangeSensor.sensor.cmUltrasonic ();
                if (rangeSensorReading >= 50)
                    rangeSensorAdjustment = 0;
                else
                {
                    //Desired range sensor values.
                    double offFromDistance = rangeSensorReading - 15;

                    //Change motor powers based on offFromHeading.
                    rangeSensorAdjustment = Math.signum (offFromDistance) * (Math.abs (offFromDistance) * 0.03);
                }
            }

            double totalAdjustmentFactor = movementPowerSign * gyroAdjustment + rangeSensorAdjustment;

            //Set resulting movement powers based on calculated values.  Can be over one since this is fixed later.
            rightDrive.setDirectMotorPower (actualMovementPower * (1 - totalAdjustmentFactor));
            leftDrive.setDirectMotorPower (actualMovementPower * (1 + totalAdjustmentFactor));
        }

        idle ();
    }


    /******** DRIVING METHODS ********/
    protected enum TerminationType {BOTTOM_ALPHA, RANGE_DIST, ENCODER_DIST}

    protected void drive (TerminationType terminationType, double stopVal, double movementPower, boolean... adjustments) throws InterruptedException
    {
        this.movementPower = movementPower;

        int initialDrivePosition = getEncoderPosition ();
        int movementPowerSign = (int) (Math.signum (movementPower));

        leftDrive.setDirectMotorPower (movementPower);
        rightDrive.setDirectMotorPower (movementPower);

        encoderDriveSpeedBoost = 0;

        //Start adjusting and checking for termination.
        boolean shouldTerminate = false;
        while (!shouldTerminate)
        {
            //Run adjustment methods.
            if (adjustments != null)
                manuallyApplySensorAdjustments (true, adjustments.length >= 1 && adjustments[0], adjustments.length >= 2 && adjustments[1]);

            NiFTFlow.pauseForSingleFrame ();

            //Check if the drive should terminate.
            switch (terminationType)
            {
                case BOTTOM_ALPHA:
                    shouldTerminate = bottomColorSensor.sensor.alpha () >= stopVal;
                    break;

                case RANGE_DIST:
                    shouldTerminate = frontRangeSensor.ultrasonicDistCM () < stopVal;
                    break;

                case ENCODER_DIST:
                    shouldTerminate = (initialDrivePosition + movementPowerSign * stopVal) * movementPowerSign >= getEncoderPosition () * movementPowerSign;
                    break;
            }
        }

        hardBrake (100);
    }


    /******** INITIALIZATION ********/
    //Initialize everything required in autonomous that isn't initialized in RobotBase (sensors)
    @Override
    protected void initializeOpModeSpecificHardware () throws InterruptedException
    {
        //The range sensors are especially odd to initialize, and will often require a robot power cycle.
        NiFTConsole.outputNewSequentialLine ("Validating Front Range Sensor...");
        frontRangeSensor = new NiFTRangeSensor ("Front Range Sensor", 0x90);
        NiFTConsole.appendToLastSequentialLine (frontRangeSensor.returningValidOutput () ? "OK!" : "FAILED!");

        NiFTConsole.outputNewSequentialLine ("Validating Side Range Sensor...");
        sideRangeSensor = new NiFTRangeSensor ("Back Range Sensor", 0x10);
        NiFTConsole.appendToLastSequentialLine (sideRangeSensor.returningValidOutput () ? "OK!" : "FAILED!");

        //Initialize color sensors.
        NiFTConsole.outputNewSequentialLine ("Fetching Color Sensors...");
        option1ColorSensor = new NiFTColorSensor ("Option 1 Color Sensor", 0x4c, false);
        option2ColorSensor = new NiFTColorSensor ("Option 2 Color Sensor", 0x5c, false);
        bottomColorSensor = new NiFTColorSensor ("Bottom Color Sensor", 0x3c, true);
        particleColorSensor = new NiFTColorSensor ("particleColorSensor", 0x6c, true);
        NiFTConsole.appendToLastSequentialLine ("OK!");

        //Initialize gyroscope.
        NiFTConsole.outputNewSequentialLine("Calibrating Gyroscope...");
        gyroscope = new NiFTGyroSensor ("Gyroscope"); //Calibrates immediately.
        NiFTConsole.appendToLastSequentialLine ("OK!");
    }


    /******** CHILD CLASS INHERITANCE ********/
    //All child classes should have special instructions.
    protected abstract void driverStationSaysGO () throws InterruptedException;
}