package org.firstinspires.ftc.teamcode.niftc.examples;

import org.firstinspires.ftc.teamcode.niftc.NiFTBase;
import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTComplexTask;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

/**
 * Using multiple tasks is an incredibly important part of programming a sophisticated robot.
 *
 * Here are implementations of two different tasks which can be started to accomplish different functionality (feel free to experiment with different tasks!)
 */
public class MultipleTasks extends NiFTBase
{
    private final class DriveRandomly extends NiFTComplexTask
    {
        @Override
        protected void onDoTask () throws InterruptedException
        {
            //Do something.
            while (true)
            {
                NiFTConsole.outputNewSequentialLine ("Updated task 1!");
                //Do another something.
                NiFTFlow.pauseForMS (1000);
            }
        }
    }

    private final class DriveRandomlyP2 extends NiFTComplexTask
    {
        @Override
        protected void onDoTask () throws InterruptedException
        {
            //Do something.
            while (true)
            {
                NiFTConsole.outputNewSequentialLine ("Updated task 2!");
                //Do another something.
                NiFTFlow.pauseForMS (1500);
            }
        }

        @Override
        protected void onQuitTask ()
        {
            //Will execute once completed.
        }
    }

    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        new DriveRandomlyP2 ().run();
        new DriveRandomly ().run();

        while (true)
        {
            NiFTConsole.outputNewSequentialLine ("Updated main thread!");
            NiFTFlow.pauseForMS (40);
        }
        //Both tasks are automatically killed at the end of the program (once stop pressed)
    }
}
