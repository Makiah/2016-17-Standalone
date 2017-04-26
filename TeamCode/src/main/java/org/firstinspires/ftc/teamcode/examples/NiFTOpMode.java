package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.NiFTBase;
import org.firstinspires.ftc.teamcode.hardware.NiFTMotorController;
import org.firstinspires.ftc.teamcode.threads.NiFTFlow;

public class NiFTOpMode extends NiFTBase
{
    protected NiFTMotorController leftDrive, rightDrive;
    @Override
    protected void initializeHardware()
    {
        //Example params.
        leftDrive = new NiFTMotorController ("Left Drive", "backLeft", "frontLeft")
                .setMotorDirection (DcMotorSimple.Direction.REVERSE)
                .setAdjustmentSensitivity (.00004);
        rightDrive = new NiFTMotorController ("Right Drive", "backRight", "frontRight");
    }

    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        //Enable PID, which is a new AsyncTask which runs independently (how cool is that?!)
        leftDrive.startPIDTask ();
        rightDrive.startPIDTask ();

        //Start moving for 10 seconds.
        leftDrive.setRPS (3);
        rightDrive.setRPS (3);

        NiFTFlow.pauseForMS (10000);
    }

    @Override
    protected void driverStationSaysSTOP ()
    {
        leftDrive.setRPS (0);
        rightDrive.setRPS (0);

        leftDrive.stopPIDTask ();
        rightDrive.stopPIDTask ();
    }
}
