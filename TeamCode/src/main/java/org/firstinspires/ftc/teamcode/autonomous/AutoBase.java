package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainRobotBase;
import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.hardware.NiFTColorSensor;
import org.firstinspires.ftc.teamcode.niftc.hardware.NiFTGyroSensor;
import org.firstinspires.ftc.teamcode.niftc.hardware.NiFTRangeSensor;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;


//For added simplicity while coding autonomous with the new FTC system. Utilizes inheritance and polymorphism.
public abstract class AutoBase extends MainRobotBase
{
    /*------ SENSOR STUFF ------*/

    /**** Color Sensors (3) ****/
    protected NiFTColorSensor option1ColorSensor, option2ColorSensor, bottomColorSensor, particleColorSensor;

    protected boolean option1Red, option2Red, option1Blue, option2Blue;
    protected void updateColorSensorStates ()
    {
        final int redThreshold = 2, blueThreshold = 3;
        option1Blue = option1ColorSensor.sensor.blue () >= blueThreshold;
        option1Red = option1ColorSensor.sensor.red () >= redThreshold;
        option2Blue = option2ColorSensor.sensor.blue () >= blueThreshold;
        option2Red = option2ColorSensor.sensor.red () >= redThreshold;
    }

    /*--- Gyro ---*/
    protected NiFTGyroSensor gyroscope;

    //Used to turn to a specified heading, and returns the difference between the desired angle and the actual angle achieved.
    protected enum TurnMode { LEFT, RIGHT, BOTH }
    protected void turnToHeading (int desiredHeading, TurnMode mode, long maxTime) throws InterruptedException
    {
        //Set gyro desired heading for future calls.
        gyroscope.setDesiredHeading (desiredHeading);

        //Create a turn process console.
        NiFTConsole.ProcessConsole turnConsole = new NiFTConsole.ProcessConsole ("Turning");

        //Required variables.
        double turnBoost = 0.15;
        final double turnCoefficient = 0.0035, initialTurnIntercept = turnBoost;

        //Get the start time so that we know when to end.
        long turnStartTime = System.currentTimeMillis ();

        //Get initial values for the automatic boost increase/decrease.
        int lastOffFromHeading = gyroscope.getOffFromHeading (), currentOffFromHeading = lastOffFromHeading;
        long lastCheckedTime = turnStartTime;

        //While the time limit is not up or the heading is still unacceptably far off.
        while (System.currentTimeMillis () - turnStartTime < maxTime || currentOffFromHeading >= 10)
        {
            //Get the current off from heading.
            currentOffFromHeading = gyroscope.getOffFromHeading ();

            //Verify that the heading that we thought was perfectly on point actually is on point.
            if (currentOffFromHeading == 0)
            {
                hardBrake (100);

                //Obtain new value and check again.
                if (gyroscope.getOffFromHeading () == 0)
                    break;
            }

            //Protection against stalling, increases power if no observed heading change in last fraction of a second.
            if (System.currentTimeMillis () - lastCheckedTime >= 300 && (System.currentTimeMillis () - turnStartTime) > 1000)
            {
                int changeFromLastBoostCheck = Math.abs (lastOffFromHeading - currentOffFromHeading);
                //Don't start increasing power at the very start of the turn before the robot has had time to accelerate.
                if (changeFromLastBoostCheck <= 1)
                    turnBoost += 0.06;
                else if (changeFromLastBoostCheck >= 7 && turnBoost > initialTurnIntercept)
                    turnBoost -= 0.03;

                //Update other variables.
                lastOffFromHeading = currentOffFromHeading;
                lastCheckedTime = System.currentTimeMillis ();
            }

            //Turn at a speed proportional to the distance from the ideal heading.
            double turnPower = Math.signum (currentOffFromHeading) * (Math.abs (currentOffFromHeading) * turnCoefficient + turnBoost);

            //Set clipped powers.
            if (mode != TurnMode.RIGHT)
                leftDrive.setDirectMotorPower (Range.clip (turnPower, -1, 1));
            if (mode != TurnMode.LEFT)
                rightDrive.setDirectMotorPower (-1 * Range.clip (turnPower, -1, 1));

            //Output required data.
            turnConsole.updateWith (
                    "Turning to heading " + gyroscope.getDesiredHeading (),
                    "Off by " + currentOffFromHeading + " degrees",
                    "Turn Power is " + turnPower,
                    "Turn boost is " + turnBoost,
                    "I have " + (maxTime - (System.currentTimeMillis () - turnStartTime)) + "ms left."
            );

            //Pause for a frame so that the program can exit if need be.
            NiFTFlow.pauseForSingleFrame ();
        }

        //Remove the console.
        turnConsole.destroy ();

        //Brake to make sure we are stationary.
        hardBrake (100);
    }

    /*--- Encoders ---*/
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

    /*--- Range Sensor(s) ---*/
    protected NiFTRangeSensor frontRangeSensor, sideRangeSensor;


    /*------- MOVEMENT POWER CONTROL -------*/
    //Used to set drive move power initially.
    private double movementPower = 0;
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


    /*------- DRIVING METHODS -------*/
    protected enum TerminationType {BOTTOM_ALPHA, RANGE_DIST, ENCODER_DIST}
    protected void drive (TerminationType terminationType, double stopVal, double movementPower, boolean... adjustments) throws InterruptedException
    {
        NiFTConsole.ProcessConsole driveConsole = new NiFTConsole.ProcessConsole ("Driving");

        //Get values required for termination.
        int movementPowerSign = (int) (Math.signum (movementPower));
        int desiredEncoderPosition = terminationType == TerminationType.ENCODER_DIST ? (int) (getEncoderPosition () + movementPowerSign * stopVal) : 0;

        //Start moving.
        startDrivingAt (movementPower);

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
                    double currentRangeVal = frontRangeSensor.validDistCM (255);
                    shouldTerminate = currentRangeVal < stopVal;

                    driveConsole.updateWith (
                            "Current dist " + currentRangeVal,
                            "Stopping at dist " + stopVal
                    );

                    break;

                case ENCODER_DIST:
                    /*
                     * At 1000 trying to go to 300
                     * power = -0.22 and dist = 700
                     * initial = 1000
                     * left = -1 * (1000 + -1 * 700) = -300
                     * right = -1 * 1000
                     * so -300 <= -1000 false
                     *
                     * At -1000 trying to get to -1300
                     * power = -0.22 and dist = 300
                     * initial = -1000
                     * left = -1 * (-1000 + -1 * 300) = 1300
                     * right = -1000 * -1
                     * so 1300 <= 1000 false
                     *
                     * At 1000 trying to get to 1300
                     * power = +1 and dist = 300
                     * initial = 1000
                     * left = 1 * (1000 + 1 * 300) = 1300
                     * right = 1 * 1000 = 1000
                     * so 1300 <= 1000
                     */

                    int currentEncoderPosition = getEncoderPosition ();
                    shouldTerminate = desiredEncoderPosition * movementPowerSign <= currentEncoderPosition * movementPowerSign;

                    driveConsole.updateWith (
                            "Driving to drive position " + desiredEncoderPosition,
                            "Current drive position = " + currentEncoderPosition
                    );
                    break;
            }
        }

        //Become stationary.
        hardBrake (100);

        driveConsole.destroy ();
    }


    /*------ INITIALIZATION -------*/
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


    /*------- CHILD CLASS INHERITANCE -----*/
    //All child classes should have special instructions.
    protected abstract void driverStationSaysGO () throws InterruptedException;
}