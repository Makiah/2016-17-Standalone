package org.firstinspires.ftc.teamcode.niftc.console;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.niftc.NiFTBase;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTComplexTask;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

import java.util.ArrayList;

/**
 * The NiFTConsole is an easy way to visualize a large number of tasks in parallel without having to rely on super quick vision or updating techniques.  It also supports sequential steps in the same window.
 *
 * This console uses a task to update its content so that it isn't jumpy when displayed on the driver station.
 */
public class NiFTConsole
{
    /**
     * Resets the entire console with empty content.
     */
    public static void reset ()
    {
        //Initialize required components.
        sequentialConsoleData = new ArrayList<> ();
        privateProcessConsoles = new ArrayList<> ();

        consoleUpdaterInstance = null;

        //Will stop when stop requested.
        startConsoleUpdater ();
    }

    /*-- USE TO OUTPUT DATA IN A SLIGHTLY BETTER WAY THAT LINEAR OP MODES PROVIDE --*/
    private static ArrayList<String> sequentialConsoleData; //Lines being added and removed.

    public static void outputNewSequentialLine (String newLine)
    {
        final int maxSequentialLines = 13;

        //Add new line at beginning of the lines.
        sequentialConsoleData.add (0, newLine);
        //If there is more than 5 lines there, remove one.
        if (sequentialConsoleData.size () > maxSequentialLines)
            sequentialConsoleData.remove (maxSequentialLines);
    }

    public static void appendToLastSequentialLine (String toAppend)
    {
        String result = sequentialConsoleData.get (0) + toAppend;
        sequentialConsoleData.remove (0);
        sequentialConsoleData.add (0, result);
    }

    /**
     * To get a private process console, create a new NiFTConsole.ProcessConsole(<name here>) and then run updateWith() to provide new content.
     */
    private static ArrayList<ProcessConsole> privateProcessConsoles;

    public static class ProcessConsole
    {
        private final String processName;
        private String[] processData;

        public ProcessConsole (String processName)
        {
            this.processName = processName;
            processData = new String[0];

            privateProcessConsoles.add (this);
        }

        public void updateWith (String... processData)
        {
            this.processData = processData;
        }

        public void destroy ()
        {
            privateProcessConsoles.remove (this);
        }

        public void revive ()
        {
            privateProcessConsoles.add (this);
        }
    }

    /**
     * The task which updates the console at a fairly slow rate but your eye can't tell the difference.
     */
    private static class ConsoleUpdater extends NiFTComplexTask
    {
        @Override
        protected void onDoTask () throws InterruptedException
        {
            while (true)
            {
                rebuildConsole ();
                NiFTFlow.pauseForMS (50);
            }
        }
    }

    private static ConsoleUpdater consoleUpdaterInstance;

    /**
     * Creates a new console updater instance and runs it.
     */
    public static void startConsoleUpdater ()
    {
        if (consoleUpdaterInstance == null)
        {
            consoleUpdaterInstance = new ConsoleUpdater ();
            consoleUpdaterInstance.run();
        }
    }

    /**
     * Nullifies a new console instance and stops it.
     */
    public static void stopConsoleUpdater ()
    {
        if (consoleUpdaterInstance != null)
        {
            consoleUpdaterInstance.stop ();
            consoleUpdaterInstance = null;
        }
    }

    /**
     * Rebuilds the whole console (call minimally, allow the task to take care of it.)
     */
    public static void rebuildConsole ()
    {
        final Telemetry mainTelemetry = NiFTBase.opModeInstance.telemetry;

        if (mainTelemetry != null)
        {
            //Clear all lines.
            mainTelemetry.update ();

            //Add all private console data.
            for (ProcessConsole pConsole : privateProcessConsoles)
            {
                mainTelemetry.addLine ("----- " + pConsole.processName + " -----");

                for (String line : pConsole.processData)
                    mainTelemetry.addLine (line);

                mainTelemetry.addLine ("");
            }

            mainTelemetry.addLine ("----- Sequential Data -----");
            for (String line : sequentialConsoleData)
            {
                mainTelemetry.addLine (line);
            }

            //Refresh the console with this new data.
            mainTelemetry.update ();
        }
        //Otherwise it just gets queued in the ArrayList.
    }
}
