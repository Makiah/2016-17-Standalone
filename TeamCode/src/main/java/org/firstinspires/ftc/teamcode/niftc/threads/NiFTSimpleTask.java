package org.firstinspires.ftc.teamcode.niftc.threads;

/**
 * NiFTSimpleTasks run in a single thread context, and are cycled through one by one in order to avoid counting toward the thread limit.
 */
public abstract class NiFTSimpleTask
{
    //Constructor.
    public final String taskName;
    public NiFTSimpleTask ()
    {
        this ("Unnamed Simple NiFTComplexTask");
    }
    public NiFTSimpleTask (String taskName)
    {
        this.taskName = taskName;
    }

    //The long returned indicates the amount of time to wait before running the task again.
    public long nextRunTime = 0;
    protected abstract long onContinueTask () throws InterruptedException;
}