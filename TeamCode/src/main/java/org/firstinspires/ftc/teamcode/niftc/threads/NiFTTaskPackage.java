package org.firstinspires.ftc.teamcode.niftc.threads;

import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;

import java.util.ArrayList;

/**
 * Used to divvy up SimpleTasks into certain groups.
 */
public class NiFTTaskPackage
{
    /**
     * Where they would generally go.
     */
    public static NiFTTaskPackage generalTaskPool;

    /**
     * Might as well include a name for the group of tasks!
     */
    public final String groupName;
    private NiFTConsole.ProcessConsole processConsole;
    public NiFTTaskPackage (String groupName)
    {
        this(groupName, null);
    }
    public NiFTTaskPackage(String groupName, NiFTSimpleTask... tasks)
    {
        this.groupName = groupName;

        //Populate task list.
        for (NiFTSimpleTask task : tasks)
            add(task);
    }

    /**
     * Place tasks in here, which will be run by the complex task class nested in this class.
     */
    private ArrayList<NiFTSimpleTask> taskList = new ArrayList<> ();
    public void add(NiFTSimpleTask simpleTask)
    {
        taskList.add (simpleTask);
        simpleTask.activate ();
    }
    public void remove(NiFTSimpleTask simpleTask)
    {
        taskList.remove (simpleTask);
        simpleTask.deactivate ();
    }

    /**
     * The SimpleTaskUpdater is a ComplexTask which takes up one AsyncTask spot but runs more than one task.
     */
    private class SimpleTaskUpdater extends NiFTComplexTask
    {
        public SimpleTaskUpdater(String taskName)
        {
            super(taskName);
        }

        @Override
        protected void onDoTask () throws InterruptedException
        {
            while (true)
            {
                //Cycle through whole list of tasks and run all that can be run.
                for (int i = 0; i < taskList.size (); i++)
                {
                    //Run if possible.
                    NiFTSimpleTask task = taskList.get(i);
                    if (task.nextRunTime < System.currentTimeMillis ())
                        task.nextRunTime = task.onContinueTask () + System.currentTimeMillis ();
                }

                //Exit program if stop requested, otherwise yield to other threads.
                NiFTFlow.yieldForFrame ();
            }
        }
    }
    private SimpleTaskUpdater taskUpdaterInstance = null;
    public void startTaskUpdater()
    {
        if (taskUpdaterInstance == null)
        {
            taskUpdaterInstance = new SimpleTaskUpdater (groupName + " Task");
            taskUpdaterInstance.run ();
        }
    }
    public void stopTaskUpdater()
    {
        if (taskUpdaterInstance != null)
        {
            taskUpdaterInstance.stop ();
            taskUpdaterInstance = null;
        }
    }
}
