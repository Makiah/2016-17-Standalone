package org.firstinspires.ftc.teamcode.niftc.threads;

import org.firstinspires.ftc.teamcode.niftc.NiFTBase;

/**
 * This class is super important to running the program while making sure to check the user requested state of the program.  While running any wait method, the program will run essentially an idle() statement to check to see whether a stop was requested, and throw an InterruptedException in that event.
 *
 * Avoid sleep() statements since I don't think that they include stopRequested() checks.   NiFTFlow.pauseForMS is better instead.
 */
public class NiFTFlow
{
    /**
     * This method "waits" for a given number of seconds by running yieldForFrame() as long as necessary.
     *
     * @param ms the milliseconds that the program should wait for.
     * @throws InterruptedException the exception which indicates that the program needs to stop.
     */
    public static void pauseForMS(long ms) throws InterruptedException
    {
        long startTime = System.currentTimeMillis ();
        while (System.currentTimeMillis () - startTime <= ms)
            yieldForFrame ();
    }

    /**
     * This method quickly checks to see whether the program needs to stop before allowing other threads to run.
     *
     * @throws InterruptedException
     */
    public static void yieldForFrame () throws InterruptedException
    {
        if (NiFTBase.opModeInstance.isStopRequested())
            throw new InterruptedException();

        Thread.yield();
    }
}
