package org.firstinspires.ftc.teamcode.niftc.threads;

import android.os.AsyncTask;

import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;

/**
 * NiFTComplexTask is an easier method of working with AsyncTasks, which provides a convenient process console and a bunch of other functionality to the table for an opmode which requires a bunch of advanced tooling.
 *
 * Warning: Moto Gs (and probably also ZTEs) can only run something like 5 tasks in parallel, and have to queue the rest to run once the other tasks are completed.  KEEP THIS IN MIND, since if one task is inexplicably not running this is probably why.
 */
public abstract class NiFTComplexTask extends AsyncTask <Void, Void, Void>
{
    public final String taskName;
    public final NiFTConsole.ProcessConsole processConsole;
    /**
     * Creates a task with a given name and a console with that same name.
     */
    public NiFTComplexTask ()
    {
        this("Unnamed NiFT Task");
    }
    public NiFTComplexTask (String taskName)
    {
        this.taskName = taskName;
        processConsole = new NiFTConsole.ProcessConsole (taskName);
    }

    /**
     * Runs the onDoTask() method while catching potential InterruptedExceptions, which indicate that the user has requested a stop which was thrown in NiFTFlow.
     *
     * Runs the onQuitAndDestroyConsole() method on catching an InterruptedException, which destroys the process console and ends the program.
     *
     * @param params can be safely ignored.
     */
    @Override
    protected final Void doInBackground (Void... params)
    {
        try
        {
            onDoTask ();
        }
        catch (InterruptedException e) //Upon stop requested by NiFTFlow
        {
            NiFTConsole.outputNewSequentialLine (taskName + " task was stopped!");
        }
        finally
        {
            onQuitAndDestroyConsole ();
        }

        return null;
    }

    /**
     * When the stop() method is called, the doInBackground method halts and onCancelled is called, which causes console destruction and task end.
     */
    @Override
    protected final void onCancelled ()
    {
        onQuitAndDestroyConsole ();
    }

    /**
     * Inherit this method in child classes to actually accomplish something during your task.
     *
     * @throws InterruptedException
     */
    protected abstract void onDoTask () throws InterruptedException;

    /**
     * Used solely in this class, used to destroy the created process console and THEN
     * run the desired onCompletion method.
     */
    private void onQuitAndDestroyConsole ()
    {
        onQuitTask ();
        processConsole.destroy ();
    }

    /**
     * Override this method if you want to do something when your task ends (regardless of whether it was cancelled or finished on its own).
     */
    protected void onQuitTask () {}

    /**
     * run() attempts to run the program in a try-catch block, and in the event of an
     * error, stops the attempt and returns an error to the user.
     */
    public final void run()
    {
        try
        {
            this.executeOnExecutor (AsyncTask.THREAD_POOL_EXECUTOR);
        }
        catch (Exception e)
        {
            NiFTConsole.outputNewSequentialLine ("Uh oh! " + taskName + " can't run!");
            NiFTConsole.outputNewSequentialLine (e.getMessage ());
            NiFTConsole.outputNewSequentialLine ("Proceeding normally.");
        }
    }
    /**
     * Stop attempts to cancel the given task, and reports an error if it cannot.
     */
    public final void stop()
    {
        try
        {
            this.cancel (true);
        }
        catch (Exception e)
        {
            NiFTConsole.outputNewSequentialLine ("Uh oh! " + taskName + " couldn't be stopped!");
            NiFTConsole.outputNewSequentialLine (e.getMessage ());
            NiFTConsole.outputNewSequentialLine ("Proceeding normally.");
        }
    }
}
