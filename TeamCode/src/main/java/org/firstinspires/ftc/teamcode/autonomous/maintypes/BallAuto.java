package org.firstinspires.ftc.teamcode.autonomous.maintypes;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.OnAlliance;
import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTComplexTask;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

public abstract class BallAuto extends AutoBase implements OnAlliance
{
    //The instance to which we will reference when stopping and starting the pick up task.
    private PickUpAndAutoRejectParticles pickUpTask;

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

    private boolean getCapBall = true;
    private boolean parkOnCenterVortex = false;

    @Override
    protected void driverStationSaysINITIALIZE () throws InterruptedException
    {
        NiFTConsole.ProcessConsole settingsConsole = new NiFTConsole.ProcessConsole ("Current Settings");

        long delay = 0;
        long lastDelayIncrementTime = 0;

        //Get input stuff for delay, etc.
        while (!opModeIsActive ()) //While start not pressed
        {
            if (System.currentTimeMillis () - lastDelayIncrementTime >= 400)
            {
                //Delay variables.
                if (gamepad1.dpad_up || gamepad2.dpad_up)
                    delay += 1000;
                else if (gamepad1.dpad_down || gamepad2.dpad_down)
                    delay -= 1000;

                delay = (long) (Range.clip (delay, 0, 30000));

                lastDelayIncrementTime = System.currentTimeMillis ();
            }

            if (gamepad1.y || gamepad2.y)
                getCapBall = false;

            if (gamepad1.x || gamepad2.x)
                parkOnCenterVortex = true;

            settingsConsole.updateWith (
                    "Delay (DPAD) is " + delay,
                    "Getting cap ball (Y) = " + getCapBall,
                    "Parking on Center Vortex (X) = " + parkOnCenterVortex,
                    "Ramming alliance robot = true"
            );

            idle ();
        }

        NiFTFlow.pauseForMS (delay);
    }

    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        lights.setPower(.5);

        final boolean onBlueAlliance = (getAlliance() == Alliance.BLUE);
        final int autonomousSign = (onBlueAlliance ? 1 : -1);

        flywheels.setPIDStatus (true);
        flywheels.setRPS (18.2);

        //Drive to the cap ball.
        NiFTConsole.outputNewSequentialLine ("Driving to shooting position.");
        drive (TerminationType.RANGE_DIST, 40, .27);

        //Shoot the balls into the center vortex.
        NiFTConsole.outputNewSequentialLine ("Shooting balls into center vortex...");
        harvester.setDirectMotorPower (1);
        NiFTFlow.pauseForMS (2200);
        harvester.setDirectMotorPower (0);
        flywheels.setPIDStatus (false);
        flywheels.setRPS (0);

        if (parkOnCenterVortex)
        {
            NiFTConsole.outputNewSequentialLine ("Parking on center vortex.");
            drive (TerminationType.ENCODER_DIST, 1400, 0.5);
            return; //End prematurely
        }

        if (getCapBall)
        {
            //Drive the remainder of the distance.
            NiFTConsole.outputNewSequentialLine ("Knock the cap ball off of the pedestal.");
            drive (TerminationType.ENCODER_DIST, 1800, 0.5);

            //Turn to face the ramp from the position that we drove.
            NiFTConsole.outputNewSequentialLine ("Turning to the appropriate heading.");
            turnToHeading (110 * autonomousSign, TurnMode.BOTH, 3000);
        } else
        {
            //Turn to face the ramp from the position that we drove.
            NiFTConsole.outputNewSequentialLine ("Turning to the appropriate heading.");
            turnToHeading (70 * autonomousSign, TurnMode.BOTH, 3000);
        }

        pickUpTask = new PickUpAndAutoRejectParticles();
        pickUpTask.run();

        //Drive until we reach the appropriate position.
        NiFTConsole.outputNewSequentialLine ("Drive to the ramp, stopping upon bottom color sensor reaches the blue region on the ramp.");
        startDrivingAt (0.6);

        long startDriveTime = System.currentTimeMillis (); //Max time at 6 seconds.
        while ((System.currentTimeMillis () - startDriveTime) < 6000)
            manuallyApplySensorAdjustments (true);

        stopDriving ();

        stopPickUpTask();
    }
}