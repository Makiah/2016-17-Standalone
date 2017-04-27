package org.firstinspires.ftc.teamcode.niftc.threads;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Used to divvy up SimpleTasks into certain groups.
 */
public class NiFTTaskPackage
{
    /**
     * Might as well include a name for the group of tasks!
     */
    public final String groupName;
    public NiFTTaskPackage (String groupName)
    {
        this(groupName, null);
    }
    public NiFTTaskPackage(String groupName, NiFTSimpleTask... tasks)
    {
        this.groupName = groupName;

        //Populate task list.
        if (tasks != null)
            taskList.addAll(Arrays.asList (tasks));
    }

    /**
     * Place tasks in here, which will be run by the complex task class nested in this class.
     */
    public ArrayList<NiFTSimpleTask> taskList = new ArrayList<> ();

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
