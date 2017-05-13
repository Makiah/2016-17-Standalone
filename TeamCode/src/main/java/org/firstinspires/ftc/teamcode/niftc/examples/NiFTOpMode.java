package org.firstinspires.ftc.teamcode.niftc.examples;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.niftc.NiFTBase;
import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.hardware.NiFTMotorController;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

/**
 * This class creates an example opMode which would very simply make the robot move forward.  Not super advanced or helpful right now, but just copy this file and change it around to do other cool stuff.
 */
public class NiFTOpMode extends NiFTBase
{
    protected NiFTMotorController leftDrive, rightDrive;

    /**
     * Override this method in order to initialize all the hardware you need.  I suggest that you also include console output so that the user sees what you are doing with the code while you grab hardware and calibrate gyroscopes.
     */
    @Override
    protected void initializeHardware()
    {
        //Example params.
        NiFTConsole.outputNewSequentialLine ("Getting drive motors...");
        leftDrive = new NiFTMotorController ("Left Drive", "backLeft", "frontLeft")
                .setMotorDirection (DcMotorSimple.Direction.REVERSE)
                .setAdjustmentSensitivity (.00004);
        rightDrive = new NiFTMotorController ("Right Drive", "backRight", "frontRight");
        NiFTConsole.appendToLastSequentialLine ("OK!");
        //Now the user sees on the DS console "Getting drive motors...OK!"
    }

    /**
     * This code runs during the period in which the driver hits INIT and the play button.  This runs after initializing the hardware.  (Note that this still runs even if the user hits the play button before all hardware has been initialized.)
     */
    @Override
    protected void driverStationSaysINITIALIZE()
    {
        //This isn't necessary since their constructors already include a reset call, just an example.
        leftDrive.resetEncoder ();
        rightDrive.resetEncoder ();
    }

    /**
     * Starts the actual code upon start being pressed.
     */
    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        //Enable PID, which is a new AsyncTask which runs independently (how cool is that?!)
        leftDrive.setPIDStatus (true);
        rightDrive.setPIDStatus (true);

        //Start moving for 10 seconds.
        leftDrive.setRPS (3);
        rightDrive.setRPS (3);

        NiFTFlow.pauseForMS (10000);
    }

    /**
     * Now when stop is pressed, we want to stop the tasks and stop the motors.
     */
    @Override
    protected void driverStationSaysSTOP ()
    {
        leftDrive.setRPS (0);
        rightDrive.setRPS (0);

        leftDrive.setPIDStatus (false);
        rightDrive.setPIDStatus (false);
    }
}
