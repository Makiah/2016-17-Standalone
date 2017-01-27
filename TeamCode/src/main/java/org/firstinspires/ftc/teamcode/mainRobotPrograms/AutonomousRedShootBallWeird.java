package org.firstinspires.ftc.teamcode.mainRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous - Red Corner Edition", group = "Autonomous Group")

public class AutonomousRedShootBallWeird extends _AutonomousBase
{
    //Called after runOpMode() has finished initializing.
    protected void driverStationSaysGO() throws InterruptedException
    {
        driveForTime(-0.5, 1300);

        flywheels.setPower(0.4);
        sleep(2000);
        harvester.setPower(1.0);

        sleep(4000);

        harvester.setPower(0);
        flywheels.setPower(0);

        driveForTime(-0.5, 800);

        turnToHeading(800, TurnMode.BOTH); //Doesn't use gyro.
        turnToHeading(-800, TurnMode.BOTH);
        driveForTime(-0.3, 1000);
    }
}