package org.firstinspires.ftc.teamcode.niftc.threads;

import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;

/**
 * NiFTSimpleTasks run in a single thread context, and are cycled through one by one in order to avoid counting toward the thread limit.
 */
public abstract class NiFTSimpleTask
{
    public final String taskName;
    protected NiFTConsole.ProcessConsole processConsole;
    /**
     * Disable this task by changing outputtingData to false :P
     */
    public boolean active = false;
    public NiFTSimpleTask (NiFTTaskPackage taskPackage)
    {
        this (taskPackage, "Unnamed Simple NiFTComplexTask");
    }
    public NiFTSimpleTask (NiFTTaskPackage taskPackage, String taskName)
    {
        this.taskName = taskName;

        taskPackage.add(this);

        processConsole = new NiFTConsole.ProcessConsole (taskName);
    }

    /**
     * Don't worry about setting this value, it is set by the task package in which this resides.
     */
    public long nextRunTime = 0;
    protected abstract long onUpdate () throws InterruptedException;
}