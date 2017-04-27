package org.firstinspires.ftc.teamcode.niftc.threads;

import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;

/**
 * NiFTSimpleTasks run in a single thread context, and are cycled through one by one in order to avoid counting toward the thread limit.
 */
public abstract class NiFTSimpleTask
{
    public static NiFTTaskPackage generalTaskPackage;

    public final String taskName;
    protected NiFTConsole.ProcessConsole processConsole;
    public NiFTSimpleTask ()
    {
        this ("Unnamed Simple NiFTComplexTask");
    }
    public NiFTSimpleTask (String taskName)
    {
        this.taskName = taskName;
    }

    public void activate()
    {
        if (processConsole == null)
            processConsole = new NiFTConsole.ProcessConsole (taskName);
    }
    public void deactivate()
    {
        if (processConsole != null)
        {
            processConsole.destroy ();
            processConsole = null;
        }
    }

    //The long returned indicates the amount of time to wait before running the task again.
    public long nextRunTime = 0;
    protected abstract long onContinueTask () throws InterruptedException;
}