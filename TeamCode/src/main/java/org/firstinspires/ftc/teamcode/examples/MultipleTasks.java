package org.firstinspires.ftc.teamcode.examples;

import org.firstinspires.ftc.teamcode.NiFTBase;
import org.firstinspires.ftc.teamcode.threads.NiFTFlow;
import org.firstinspires.ftc.teamcode.threads.NiFTTask;

public class MultipleTasks extends NiFTBase
{
    private final class DriveRandomly extends NiFTTask
    {
        @Override
        protected void onDoTask () throws InterruptedException
        {
            //Do something.
            while (true)
            {
                //Do another something.
                NiFTFlow.pauseForMS (30);
            }
        }
    }

    private final class DriveRandomlyP2 extends NiFTTask
    {
        @Override
        protected void onDoTask () throws InterruptedException
        {
            //Do something.
            while (true)
            {
                //Do another something.
                NiFTFlow.pauseForMS (20);
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
            NiFTFlow.pauseForMS (40);
        }
        //Both tasks are automatically killed at the end of the program (once stop pressed)
    }
}
