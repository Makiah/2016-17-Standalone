package org.firstinspires.ftc.teamcode.mainRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Autonomous - Blue Edition", group = "Autonomous Group")
//@Disabled

public class AutonomousBlue extends AutonomousBase {
    //Autonomous code for the Blue alliance

    //Called after runOpMode() has finished initializing.
    protected void driverStationSaysGO()
    {
        sleep(0);
        drive(0.8, 1500);
        turn(0.3, -110);
        drive(1, 2000);
        harvester.setPower(0.5);
        sleep(3000);
    }
}