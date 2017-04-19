package org.firstinspires.ftc.teamcode.autonomous.maintypes;

import org.firstinspires.ftc.teamcode.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.OnAlliance;
import org.firstinspires.ftc.teamcode.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.threads.NiFTFlow;

public abstract class BeaconAuto extends AutoBase implements OnAlliance
{
    //Called after runOpMode() has finished initializing by BaseFunctions.
    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        final double BEACON_DP = 0.27;

        //Results in a coefficient of 1 if doing blue, and -1 for red.
        boolean onBlueAlliance = (getAlliance () == Alliance.BLUE);
        int autonomousSign = (onBlueAlliance ? 1 : -1);

        /******** STEP 1: SHOOT, DRIVE, TURN TO BE PARALLEL WITH WALL ********/

        //Start the flywheels so that PID has time to adjust them.
        flywheels.setRPS (25);
        flywheels.startPIDTask ();

        //Drive until we are just far enough from the cap ball to score reliably.
        NiFTConsole.outputNewSequentialLine ("Driving forward to the cap ball to score...");
        drive (TerminationType.RANGE_DIST, 41, BEACON_DP, true, false);

        //Shoot the balls into the center vortex.
        NiFTConsole.outputNewSequentialLine ("Shooting balls into center vortex...");
        harvester.startPIDTask ();
        harvester.setRPS (5);
        NiFTFlow.pauseForMS (2200);

        //Stop flywheels and harvester.
        flywheels.setRPS (0);
        harvester.setRPS (0);
        flywheels.stopPIDTask ();
        harvester.stopPIDTask ();

        //Turn to face the wall directly.
        NiFTConsole.outputNewSequentialLine ("Turning to face wall at an angle...");
        turnToHeading (73 * autonomousSign, TurnMode.BOTH, 3000);

        //Drive to the wall and stop once a little ways away.
        NiFTConsole.outputNewSequentialLine ("Driving to the wall...");
        drive(TerminationType.RANGE_DIST, (onBlueAlliance ? 36 : 39), BEACON_DP, true);

        //Turn back to become parallel with the wall.
        NiFTConsole.outputNewSequentialLine ("Turning to become parallel to the wall...");
        turnToHeading (onBlueAlliance ? 0 : -180, TurnMode.BOTH, 3000);

        //For each of the two beacons.
        for (int currentBeacon = 1; currentBeacon <= 2; currentBeacon++)
        {
            /******** STEP 2: FIND AND CENTER SELF ON BEACON ********/
            //Set movement speed.
            startDrivingAt (0.35 * autonomousSign);

            NiFTConsole.ProcessConsole beaconViewerConsole = new NiFTConsole.ProcessConsole ("Beacon Viewer");
            beaconViewerConsole.updateWith ("Looking for beacon " + currentBeacon);

            boolean aboutToSeeWhiteLine = false;
            int currentAlpha = bottomColorSensor.sensor.alpha ();
            while (currentAlpha <= 4)
            {
                //Update alpha.
                currentAlpha = bottomColorSensor.sensor.alpha ();

                //Update color sensor states.
                updateColorSensorStates ();

                //Check if we are about to see the white line and slow down if true.
                if (!aboutToSeeWhiteLine && (onBlueAlliance ? (option1Red || option1Blue) : (option2Red || option2Blue)))
                {
                    hardBrake (100); //Useful in order to slow down.
                    startDrivingAt (BEACON_DP * autonomousSign);
                    aboutToSeeWhiteLine = true;
                }

                //adjustDirectionBasedOnColorSensors ();
                manuallyApplySensorAdjustments (true, true, true);

                beaconViewerConsole.updateWith (
                        "Alpha is " + currentAlpha,
                        "About to see white line = " + aboutToSeeWhiteLine
                );
            }
            beaconViewerConsole.destroy ();
            //Stop once centered on the beacon.
            hardBrake (150);


            /******** STEP 3: PRESS AND VERIFY THE BEACON!!!!! ********/

            NiFTConsole.ProcessConsole buttonPressingConsole = new NiFTConsole.ProcessConsole ("Button Pressing");
            buttonPressingConsole.updateWith (
                    "Ahoy there!  Beacon spotted!  Option 1 is " + (option1Blue ? "blue" : "red") + " and option 2 is " + (option2Blue ? "blue" : "red")
            );

            //Extend the button pusher and the sensors until two distinct colors are seen.
            long startTime = System.currentTimeMillis ();
            rightButtonPusher.setToUpperLim ();
            while (!((option1Blue && option2Red) || (option2Blue && option1Red))) //Two distinct beacon colors.
                NiFTFlow.pauseForSingleFrame ();
            rightButtonPusher.setServoPosition (.5); //Stop extending.
            long timeTakenToSeeDistinctColors = System.currentTimeMillis () - startTime;

            //While the beacon is not completely blue (this is the verification step).
            int failedAttempts = 0; //The robot tries different drive lengths for each trial.
            updateColorSensorStates (); //Has to know the initial colors.
            initializeAndResetEncoders (); //Does this twice in total to prevent time loss.

            while (onBlueAlliance ? (!(option1Blue && option2Blue)) : (!(option1Red && option2Red)))
            {
                buttonPressingConsole.updateWith ("Beacon is not completely blue, attempting to press the correct color!");

                //The possible events that could occur upon either verification or first looking at the beacon.
                double drivePower;
                int driveDistance;
                if (option1Blue && option2Red)
                {
                    buttonPressingConsole.updateWith ("Chose option 1");
                    //Use the option 1 button pusher.
                    drivePower = BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 120 : 90) + 20 * failedAttempts;
                }
                else if (option1Red && option2Blue)
                {
                    buttonPressingConsole.updateWith ("Chose option 2");
                    //Use the option 2 button pusher.
                    drivePower = -BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 180 : 160) + 20 * failedAttempts;
                }
                else if (option1Blue ? (option1Red && option2Red) : (option1Blue && option2Blue))
                {
                    buttonPressingConsole.updateWith ("Neither option is the correct color, toggling beacon!");
                    //Toggle beacon.
                    drivePower = BEACON_DP * autonomousSign;
                    driveDistance = (onBlueAlliance ? 120 : 90) + 20 * failedAttempts;
                }
                else
                {
                    failedAttempts = -1; //This will be incremented and returned to 0, fear not.
                    buttonPressingConsole.updateWith ("Can't see the beacon clearly, so double checking!");
                    drivePower = 0.35;
                    driveDistance = 100;
                }

                //Drive based on state determined previously.
                drive (TerminationType.ENCODER_DIST, driveDistance, drivePower);

                //Press the button (which uses the range sensor but pass in time taken)
                pressButton (timeTakenToSeeDistinctColors);

                //Drive back to the white line.
                startDrivingAt (Math.signum (drivePower) * BEACON_DP);
                while (bottomColorSensor.sensor.alpha () <= 4)
                    manuallyApplySensorAdjustments (true, true);
                hardBrake (100);

                //Update the number of trials completed so that we know the new drive distance and such.
                failedAttempts++;

                //Update beacon states to check loop condition.
                updateColorSensorStates ();
            }

            buttonPressingConsole.destroy ();

            NiFTConsole.outputNewSequentialLine ("Success!  Beacon " + currentBeacon + " is completely blue.");

            //Retract button pusher.
            rightButtonPusher.setToLowerLim ();
            startTime = System.currentTimeMillis ();
            while (System.currentTimeMillis () - startTime < timeTakenToSeeDistinctColors)
                NiFTFlow.pauseForSingleFrame ();
            rightButtonPusher.setServoPosition (.5); //Stop extending.

            //Drive a bit forward from the white line to set up for the next step.
            drive(TerminationType.ENCODER_DIST, 3000, -0.42 * autonomousSign, true); //SPRINT TO THE CAP BALL TO PARK
        }


        /******** STEP 4: PARK AND KNOCK OFF THE CAP BALL ********/

        //Dash backward to the ramp afterward.
        NiFTConsole.outputNewSequentialLine ("Knocking the cap ball off of the pedestal...");
        turnToHeading (36 * autonomousSign - (onBlueAlliance ? 0 : 180), TurnMode.BOTH, 2000);
        drive(TerminationType.ENCODER_DIST, 3000, -1.0 * autonomousSign, true); //SPRINT TO THE CAP BALL TO PARK

    }

    private void pressButton (long timeTakenToSeeDistinctColorsInitially) throws InterruptedException
    {
        //Determine the length to push the pusher out based on the distance from the wall.
        double distanceFromWall = sideRangeSensor.ultrasonicDistCM ();
        if (distanceFromWall >= 255)
        {
            //Possible that this was a misreading.
            NiFTFlow.pauseForSingleFrame ();
            distanceFromWall = sideRangeSensor.ultrasonicDistCM ();
            if (distanceFromWall >= 255) //It can't actually be 255.
                distanceFromWall = 20;
        }
        double extendLength = distanceFromWall * 67;
        NiFTConsole.outputNewSequentialLine ("Extending the button pusher for " + extendLength + " ms.");

        //Run the continuous rotation servo out to press, then back in.
        rightButtonPusher.setToUpperLim ();
        NiFTFlow.pauseForMS ((long) (extendLength - timeTakenToSeeDistinctColorsInitially));
        rightButtonPusher.setToLowerLim ();
        NiFTFlow.pauseForMS ((long) (extendLength - 100 - timeTakenToSeeDistinctColorsInitially));
        rightButtonPusher.setServoPosition (.5);
    }
}
