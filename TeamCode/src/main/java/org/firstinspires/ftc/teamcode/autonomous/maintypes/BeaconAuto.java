package org.firstinspires.ftc.teamcode.autonomous.maintypes;

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
            harvester.setDirectMotorPower (.8);
            flywheels.setDirectMotorPower (-.4);

            //Start the task which actually looks at the color of the sensors.
            while (true)
            {
                if (getAlliance () == Alliance.BLUE)
                {
                    if (particleColorSensor.sensor.blue () > 3)
                        pickedUpParticles++;
                    else if (particleColorSensor.sensor.red () > 3)
                    {
                        harvester.setDirectMotorPower (-1);
                        NiFTFlow.pauseForMS (1500);
                        harvester.setDirectMotorPower (.8);
                    }
                }
                else
                {
                    if (particleColorSensor.sensor.red () > 3)
                        pickedUpParticles++;
                    else if (particleColorSensor.sensor.blue () > 3)
                    {
                        harvester.setDirectMotorPower (-1);
                        NiFTFlow.pauseForMS (1500);
                        harvester.setDirectMotorPower (.8);
                    }
                }

                //Pause for a frame.
                NiFTFlow.yieldForFrame ();
            }
        }

        @Override
        protected void onQuitTask ()
        {
            harvester.setDirectMotorPower (0);
            flywheels.setDirectMotorPower (0);
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
        final int redThreshold = 1, blueThreshold = 3;
        option1Blue = option1ColorSensor.sensor.blue () >= blueThreshold;
        option1Red = option1ColorSensor.sensor.red () >= redThreshold && !option1Blue;
        option2Blue = option2ColorSensor.sensor.blue () >= blueThreshold;
        option2Red = option2ColorSensor.sensor.red () >= redThreshold && !option2Blue;
    }

    //Called after runOpMode() has finished initializing by BaseFunctions.
    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        //The power at which the robot will attempt to drive to ensure accuracy.
        final double BEACON_DP = 0.25;

        //Results in a coefficient of 1 if doing blue, and -1 for red.
        final boolean onBlueAlliance = (getAlliance () == Alliance.BLUE);
        final int autonomousSign = (onBlueAlliance ? 1 : -1);

        /*------- STEP 1: SHOOT, DRIVE, TURN TO BE PARALLEL WITH WALL --------*/

        //Start the flywheels so that PID has time to adjust them.
        NiFTConsole.outputNewSequentialLine ("Starting flywheels...");
        flywheels.setRPS (18.2);
        flywheels.startPIDTask ();

        //Drive until we are just far enough from the cap ball to score reliably.
        NiFTConsole.outputNewSequentialLine ("Driving forward to the cap ball to score...");
        drive (TerminationType.RANGE_DIST, 43, .3, true);

        //Shoot the balls into the center vortex.
        NiFTConsole.outputNewSequentialLine ("Shooting balls into center vortex...");
        harvester.setDirectMotorPower (1);

        //Wait for the balls to shoot out.
        NiFTFlow.pauseForMS (2200);

        //Stop flywheels and harvester.
        flywheels.setRPS (0);
        flywheels.stopPIDTask ();
        harvester.setDirectMotorPower (0);

        //Turn to face the wall directly.
        NiFTConsole.outputNewSequentialLine ("Turning to face wall at an angle...");
        turnToHeading (73 * autonomousSign, TurnMode.BOTH, 3000);

        //Start the collection task.
        PickUpAndAutoRejectParticles pickUpTask = new PickUpAndAutoRejectParticles ();
        pickUpTask.run();

        //Drive to the wall and stop once a little ways away.
        NiFTConsole.outputNewSequentialLine ("Driving to the wall...");
        drive(TerminationType.RANGE_DIST, (onBlueAlliance ? 33 : 36), .3, true);

        //Turn back to become parallel with the wall.
        NiFTConsole.outputNewSequentialLine ("Turning to become parallel to the wall...");
        turnToHeading (onBlueAlliance ? 0 : -180, TurnMode.BOTH, 3000);

        //Extend pusher so that we are right up next to the wall.
        NiFTConsole.outputNewSequentialLine ("Extending to become close to the wall...");
        double distFromWall = sideRangeSensor.validDistCM (20, 2000); //Make sure valid.
        int gapBetweenWallAndSensorApparatus = 10;
        long extensionTime = (long) ((distFromWall - gapBetweenWallAndSensorApparatus) * 67);
        rightButtonPusher.setToUpperLim ();
        NiFTFlow.pauseForMS (extensionTime);
        rightButtonPusher.setServoPosition (0.5);

        //For each of the two beacons.
        for (int currentBeacon = 1; currentBeacon <= 2; currentBeacon++)
        {
            /*------- STEP 2: FIND AND CENTER SELF ON BEACON --------*/
            centerSelfOnWhiteLine (BEACON_DP * autonomousSign, 3000);

            /*------- STEP 3: PRESS AND VERIFY THE BEACON!!!!! -------*/

            NiFTConsole.outputNewSequentialLine ("Ahoy there!  Beacon spotted!  Option 1 is " + (option1Blue ? "blue" : "red") + " and option 2 is " + (option2Blue ? "blue" : "red"));

            //While the beacon is not completely blue (this is the verification step).
            int failedAttempts = 0; //The robot tries different drive lengths for each trial.
            updateColorSensorStates (); //Has to know the initial colors.
            initializeAndResetEncoders (); //Does this twice in total to prevent time loss.

            while (onBlueAlliance ? (!(option1Blue && option2Blue)) : (!(option1Red && option2Red)))
            {
                NiFTConsole.outputNewSequentialLine ("Beacon is not completely blue, attempting to press the correct color!");

                //The possible events that could occur upon either verification or first looking at the beacon.
                double drivePower;
                int driveDistance;
                if (option1Blue && option2Red)
                {
                    NiFTConsole.outputNewSequentialLine ("Chose option 1");
                    //Use the option 1 button pusher.
                    drivePower = BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 90 : 60) + 20 * failedAttempts;
                }
                else if (option1Red && option2Blue)
                {
                    NiFTConsole.outputNewSequentialLine ("Chose option 2");
                    //Use the option 2 button pusher.
                    drivePower = -BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 130 : 110) + 20 * failedAttempts;
                }
                else if (option1Blue ? (option1Red && option2Red) : (option1Blue && option2Blue))
                {
                    NiFTConsole.outputNewSequentialLine ("Neither option is the correct color, toggling beacon!");
                    //Toggle beacon.
                    drivePower = BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 90 : 60) + 20 * failedAttempts;
                }
                else
                {
                    failedAttempts = -1; //This will be incremented and returned to 0, fear not.
                    NiFTConsole.outputNewSequentialLine ("Can't see the beacon clearly, so extending!");
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

                //Start driving.
                centerSelfOnWhiteLine (-1 * Math.signum (drivePower) * BEACON_DP, 2500);

                //Update the number of trials completed so that we know the new drive distance and such.
                failedAttempts++;

                //Update beacon states to check loop condition.
                updateColorSensorStates ();
            }

            NiFTConsole.outputNewSequentialLine ("Success!  Beacon " + currentBeacon + " is completely blue.");

            //Drive a bit forward from the white line to set up for the next step.
            int distToDrive;
            if (currentBeacon == 1)
                distToDrive = 800;
            else
                distToDrive = 200;

            //Drive to the next line/a little ways away from the beacon for the turn.
            drive(TerminationType.ENCODER_DIST, distToDrive, 0.42 * autonomousSign, true);
        }


        /*------- STEP 3.5: SHOOT THE PARTICLES IF WE PICKED ANY UP ------*/

        //Stop the task.
        pickUpTask.stop();
        if (pickUpTask.getPickedUpParticles () >= 1)
            NiFTConsole.outputNewSequentialLine ("I would have shot balls here, but that's not coded yet :(");


        /*------- STEP 4: PARK AND KNOCK OFF THE CAP BALL -------*/

        NiFTConsole.outputNewSequentialLine ("Knocking the cap ball off of the pedestal...");
        turnToHeading (onBlueAlliance ? 36 : -216, TurnMode.BOTH, 2000);
        drive(TerminationType.ENCODER_DIST, 3000, -autonomousSign, true); //SPRINT TO THE CAP BALL TO PARK
    }

    private void centerSelfOnWhiteLine(double initialPower, long tooLongDelay) throws InterruptedException
    {
        //Start driving.
        startDrivingAt (initialPower);
        //While we haven't gotten on the line and seen distinct colors (can't vary).
        long startTime = System.currentTimeMillis ();
        boolean tookTooLong = false;
        while (bottomColorSensor.sensor.alpha() < 4)
        {
            //it might miss the beacon.
            if ((System.currentTimeMillis () - startTime) > 2500)
            {
                tookTooLong = true;
                break;
            }
            manuallyApplySensorAdjustments (true, true);
        }

        //Stop once centered on the beacon.
        hardBrake (100);

        if (tookTooLong || bottomColorSensor.sensor.alpha() < 4)
            centerSelfOnWhiteLine (-(initialPower - 0.05), (long) (tooLongDelay * 4.0/5.0)); //Recursion at its finest :P
    }
}
