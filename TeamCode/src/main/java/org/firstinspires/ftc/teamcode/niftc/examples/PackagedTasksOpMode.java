package org.firstinspires.ftc.teamcode.niftc.examples;

import org.firstinspires.ftc.teamcode.niftc.NiFTBase;
import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTSimpleTask;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTTaskPackage;

public class PackagedTasksOpMode extends NiFTBase
{
    /**
     * Each class is an example of how related tasks can be packaged with one another.
     */
    private class RightPIDUpdater extends NiFTSimpleTask
    {
        public RightPIDUpdater()
        {
            super("Right PID Updater");
        }

        @Override
        protected long onContinueTask () throws InterruptedException
        {
            NiFTConsole.outputNewSequentialLine ("Updating left PID...");

            //Now the method would attempt to run.

            return 30; //Wait 30 milliseconds before running again.
        }
    }
    private class LeftPIDUpdater extends NiFTSimpleTask
    {
        public LeftPIDUpdater ()
        {
            super("Left PID Task");
        }

        @Override
        protected long onContinueTask () throws InterruptedException
        {
            NiFTConsole.outputNewSequentialLine ("Updating right PID...");

            //Rebuild console here.

            return 50; //Wait 50 milliseconds for some reason (we're just TESTING chill out)
        }
    }

    @Override
    protected void driverStationSaysGO () throws InterruptedException
    {
        //Create both updater tasks.
        RightPIDUpdater rightPIDUpdater = new RightPIDUpdater ();
        LeftPIDUpdater leftPIDUpdater = new LeftPIDUpdater ();

        //Create a new NiFTTaskPackage with these components.
        NiFTTaskPackage taskPackage =
                new NiFTTaskPackage ("Drive PID Updates", leftPIDUpdater, rightPIDUpdater);

        //Start the PID updater.
        taskPackage.startTaskUpdater ();

        //Pause for a while
        NiFTFlow.pauseForMS (2000);

        //Stop PID updater
        taskPackage.stopTaskUpdater ();
    }
}