package org.firstinspires.ftc.teamcode.driverchoices;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.maintypes.ResetAuto;


public class DriveBackward extends ResetAuto
{
    @Override
    protected int getSign ()
    {
        return -1;
    }
}