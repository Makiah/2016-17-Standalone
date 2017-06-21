package org.firstinspires.ftc.teamcode.autonomous.maintypes;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.OnAlliance;
import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTComplexTask;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

public abstract class BeaconAuto extends AutoBase implements OnAlliance
{
    //The task which will be used to control harvesting particles while we drive.
    private class PickUpAndAutoRejectParticles extends NiFTComplexTask
    {
        public PickUpAndAutoRejectParticles()
        {
            super("Particle Pick Up Task");
        }

        private int pickedUpParticles = 0;

        @Override
        protected void onDoTask () throws InterruptedException
        {
            //Set initial harvester power.
            harvester.setDirectMotorPower (.5);
            flywheels.setDirectMotorPower (-.4);

            //Start the task which actually looks at the color of the sensors.
            while (true)
            {
                if (getAlliance () == Alliance.BLUE)
                {
                    if (particleColorSensor.sensor.blue () > 12)
                        pickedUpParticles++;
                    else if (particleColorSensor.sensor.red () > 12)
                    {
                        harvester.setDirectMotorPower (-1);
                        NiFTFlow.pauseForMS (1500);
                        harvester.setDirectMotorPower (.5);
                    }
                }
                else
                {
                    if (particleColorSensor.sensor.red () > 12)
                        pickedUpParticles++;
                    else if (particleColorSensor.sensor.blue () > 12)
                    {
                        harvester.setDirectMotorPower (-1);
                        NiFTFlow.pauseForMS (1500);
                        harvester.setDirectMotorPower (.5);
                    }
                }

                //Pause for a frame.
                NiFTFlow.yieldForFrame ();
            }
        }

        @Override
        protected void onQuitTask ()
        {
            NiFTConsole.outputNewSequentialLine("Got onQuitTask()");
            harvester.setDirectMotorPower(0);
            flywheels.setDirectMotorPower(0);
        }

        public int getPickedUpParticles()
        {
            return pickedUpParticles;
        }
    }

    //Color sensor updates.
    private boolean option1Red, option2Red, option1Blue, option2Blue;
    private void updateColorSensorStates ()
    {
        final int redThreshold = 1, blueThreshold = 2;
        option1Blue = option1ColorSensor.sensor.blue () >= blueThreshold;
        option1Red = option1ColorSensor.sensor.red () >= redThreshold && !option1Blue;
        option2Blue = option2ColorSensor.sensor.blue () >= blueThreshold;
        option2Red = option2ColorSensor.sensor.red () >= redThreshold && !option2Blue;
    }

    final private boolean lightDebug = true;
    private PickUpAndAutoRejectParticles pickUpTask;

    //Called after runOpMode() has finished initializing by BaseFunctions.
    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        //The power at which the robot will attempt to drive to ensure accuracy.
        final double BEACON_DP = 0.2;

        //Results in a coefficient of 1 if doing blue, and -1 for red.
        final boolean onBlueAlliance = (getAlliance () == Alliance.BLUE);
        final int autonomousSign = (onBlueAlliance ? 1 : -1);

        /*------- STEP 1: SHOOT, DRIVE, TURN TO BE PARALLEL WITH WALL --------*/

        //Start the flywheels so that PID has time to adjust them.
        NiFTConsole.outputNewSequentialLine ("Starting flywheels...");
        flywheels.setRPS (18.2);
        flywheels.setPIDStatus (true);

        //Drive until we are just far enough from the cap ball to score reliably.
        NiFTConsole.outputNewSequentialLine ("Driving forward to the cap ball to score...");
        drive (TerminationType.RANGE_DIST, 43, .4, true);

        //Shoot the balls into the center vortex.
        NiFTConsole.outputNewSequentialLine ("Shooting balls into center vortex...");
        harvester.setDirectMotorPower (1);

        //Wait for the balls to shoot out.
        NiFTFlow.pauseForMS (2200);

        //Stop flywheels and harvester.
        flywheels.setRPS (0);
        flywheels.setPIDStatus (false);
        harvester.setDirectMotorPower (0);

        //Turn to face the wall directly.
        NiFTConsole.outputNewSequentialLine ("Turning to face wall at an angle...");
        turnToHeading (73 * autonomousSign, TurnMode.BOTH, 3000);

        //Start the collection task.
        pickUpTask = new PickUpAndAutoRejectParticles ();
        pickUpTask.run();

        //Drive to the wall and stop once a little ways away.
        NiFTConsole.outputNewSequentialLine ("Driving to the wall...");
        drive(TerminationType.RANGE_DIST, (onBlueAlliance ? 30 : 33), .3, true);

        //If on red, then turn the collecting off.
        if (!onBlueAlliance) {
            stopPickUpTask();
        }

        //Turn back to become parallel with the wall.
        NiFTConsole.outputNewSequentialLine ("Turning to become parallel to the wall...");
        turnToHeading (onBlueAlliance ? 0 : -180, TurnMode.BOTH, onBlueAlliance ? 2500 : 3000);

        drive(TerminationType.ENCODER_DIST, onBlueAlliance ? 170 : 100, BEACON_DP * autonomousSign, true, true, true);
        NiFTFlow.pauseForMS(500);

        //For each of the two beacons.
        for (int currentBeacon = 1; currentBeacon <= 2; currentBeacon++)
        {
            /*------- STEP 2: FIND AND CENTER SELF ON BEACON --------*/
            //Don't drive as far on the first beacon since we know that we're already pretty close.
            NiFTConsole.outputNewSequentialLine("Centering self on white line...");
            centerSelfOnWhiteLine(BEACON_DP * autonomousSign, currentBeacon == 1 ? 600 : 800);

            //DEBUGGING, output
            blinkLights();

            /*------- SEE THE BEACON --------*/
            NiFTConsole.outputNewSequentialLine ("Extending to become close to the wall...");
            double distFromWall = sideRangeSensor.validDistCM (20, 2000); //Make sure valid.
            int gapBetweenWallAndSensorApparatus = 10;
            long extensionTime = (long) Range.clip(((distFromWall - gapBetweenWallAndSensorApparatus) * 67), 0, 3000);
            rightButtonPusher.setToUpperLim ();
            NiFTFlow.pauseForMS (extensionTime);
            rightButtonPusher.setServoPosition (0.5);

            /*------- STEP 3: PRESS AND VERIFY THE BEACON!!!!! -------*/

            NiFTConsole.outputNewSequentialLine ("Ahoy there!  Beacon spotted!  Option 1 is " + (option1Blue ? "blue" : "red") + " and option 2 is " + (option2Blue ? "blue" : "red"));

            //While the beacon is not completely blue (this is the verification step).
            int failedAttempts = 0; //The robot tries different drive lengths for each trial.
            updateColorSensorStates (); //Has to know the initial colors.
            initializeAndResetEncoders (); //Does this twice in total to prevent time loss.


            boolean pressedAtLeastOnce = false;
            //As long as the colors don't align to what they should be.
            while ((onBlueAlliance ? (!(option1Blue && option2Blue)) : (!(option1Red && option2Red))) || !pressedAtLeastOnce)
            {
                //Output what the robot sees.
                NiFTConsole.outputNewSequentialLine ("Beacon is not completely red/blue, attempting to press the correct color!");

                //The possible events that could occur upon either verification or first looking at the beacon.
                double drivePower;
                int driveDistance;
                if (option1Blue && option2Red)
                {
                    NiFTConsole.outputNewSequentialLine ("Chose option 1");
                    //Use the option 1 button pusher.
                    drivePower = BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 90 : 60) + 20 * failedAttempts;
                    pressedAtLeastOnce = true;
                }
                else if (option1Red && option2Blue)
                {
                    NiFTConsole.outputNewSequentialLine ("Chose option 2");
                    //Use the option 2 button pusher.
                    drivePower = -BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 130 : 110) + 20 * failedAttempts;
                    pressedAtLeastOnce = true;
                }
                else if (onBlueAlliance ? (option1Red && option2Red) : (option1Blue && option2Blue))
                {
                    NiFTConsole.outputNewSequentialLine ("Neither option is the correct color, toggling beacon!");
                    //Toggle beacon.
                    drivePower = BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 90 : 60) + 20 * failedAttempts;
                    pressedAtLeastOnce = true;
                }
                else
                {
                    NiFTConsole.outputNewSequentialLine ("Can't see the beacon clearly, so doing the shimmy!");
                    failedAttempts = -1; //This will be incremented and returned to 0, fear not.
                    driveDistance = 100;
                    drivePower = 0.35;
                }

                //Drive based on state determined previously.
                drive (TerminationType.ENCODER_DIST, driveDistance, drivePower);

                //If this is a reset run then try again.
                if (failedAttempts != -1)
                {
                    //Run the continuous rotation servo out to press, then back in.
                    rightButtonPusher.setToUpperLim ();
                    NiFTFlow.pauseForMS (gapBetweenWallAndSensorApparatus * 67); //Those cm we excluded previously.
                    rightButtonPusher.setToLowerLim ();
                    NiFTFlow.pauseForMS (gapBetweenWallAndSensorApparatus * 67 - 300);
                    rightButtonPusher.setServoPosition (.5);
                }

                //Drive back to re-center on the white line.
                centerSelfOnWhiteLine (-1 * Math.signum (drivePower) * BEACON_DP, driveDistance * 2);

                blinkLights();

                //Wait for a moment if we couldn't see the line so that the drivers know that we aren't just jittering around.
                if (failedAttempts == -1)
                    NiFTFlow.pauseForMS(1000);

                //Update the number of trials completed so that we know the new drive distance and such.
                failedAttempts++;

                //Update beacon states to check loop condition.
                updateColorSensorStates ();
            }

            //Output success.
            NiFTConsole.outputNewSequentialLine ("Success!  Beacon " + currentBeacon + " is completely red/blue.");

            //Retract servo.
            rightButtonPusher.setServoPosition(0);
            NiFTFlow.pauseForMS(gapBetweenWallAndSensorApparatus * 67);
            rightButtonPusher.setServoPosition(0.5);

            //Drive to the next line/a little ways away from the beacon for the turn.
            drive(TerminationType.ENCODER_DIST, currentBeacon == 1 ? 1500 : 200, 0.42 * autonomousSign, true, true, true);
            if (currentBeacon == 1) {
                NiFTFlow.pauseForMS(500);
                blinkLights();
            }
        }

        /*------- STEP 3.5: SHOOT THE PARTICLES IF WE PICKED ANY UP ------*/
        if (onBlueAlliance)
            stopPickUpTask();


        /*------- STEP 4: PARK AND KNOCK OFF THE CAP BALL -------*/

        NiFTConsole.outputNewSequentialLine ("Knocking the cap ball off of the pedestal...");
        rightButtonPusher.setToLowerLim ();
        turnToHeading (onBlueAlliance ? 30 : -210, TurnMode.BOTH, 3000);
        rightButtonPusher.setServoPosition (0.5);
        drive(TerminationType.ENCODER_DIST, 3000, -autonomousSign, true); //SPRINT TO THE CAP BALL TO PARK
    }

    private void stopPickUpTask() throws InterruptedException
    {
        harvester.setDirectMotorPower(0);
        flywheels.setDirectMotorPower(0);
        NiFTFlow.pauseForMS(100);
        if (pickUpTask != null) {
            //Stop the task.
            pickUpTask.stop();
        }
    }

    private void centerSelfOnWhiteLine(double initialPower, int tooLongDistance) throws InterruptedException
    {
        //Start driving.
        startDrivingAt (initialPower);

        //While we haven't gotten on the line and seen distinct colors (can't vary).
        int movementPowerSign = (int) (Math.signum (initialPower));
        int desiredEncoderPosition = (getEncoderPosition () + movementPowerSign * tooLongDistance);
        boolean sawWhiteLine = false;
        //Drive that given distance.
        while (!(desiredEncoderPosition * movementPowerSign <= getEncoderPosition() * movementPowerSign))
        {
            if (bottomColorSensor.sensor.alpha() > 4)
            {
                sawWhiteLine = true;
                break;
            }
            manuallyApplySensorAdjustments(true, true, true);
        }

        //Stop once centered on the beacon.
        hardBrake (200);

        //Make sure that we have centered, otherwise do it again at a lower power and a lower distance in the opp direction.
        if (!sawWhiteLine && bottomColorSensor.sensor.alpha() < 4)
            centerSelfOnWhiteLine (-1 * (initialPower * 4.0/5.0), (int) (tooLongDistance * 5.0/6.0)); //Recursion at its finest :P
    }

    private void blinkLights() throws InterruptedException
    {
        if (!lightDebug) return;
        lights.setPower(.5);
        NiFTFlow.pauseForMS(800);
        lights.setPower(0);
    }
}
