package org.firstinspires.ftc.teamcode.niftc.threads;

import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/**
 * Used to divvy up SimpleTasks into certain groups based on their functionality so that they don't count toward the 5 complex tasks limit.
 */
public class NiFTTaskPackage
{
    /**
     * Might as well include a name for the group of tasks!
     */
    public final String groupName;
    private ArrayList<NiFTSimpleTask> taskList;
    public NiFTTaskPackage (String groupName)
    {
        this(groupName, null);
    }
    public NiFTTaskPackage(String groupName, NiFTSimpleTask... tasks)
    {
        NiFTConsole.outputNewSequentialLine("Created task package " + groupName + "...");
        this.groupName = groupName;

        //Populate task list.
        taskList = new ArrayList<> ();
        add(tasks);

        //Start updating packages.
        startPackage ();
    }

    /**
     * Place tasks in here, which will be run by the complex task class nested in this class.
     */
    public void add(NiFTSimpleTask... simpleTasks)
    {
        if (simpleTasks != null)
            taskList.addAll(Arrays.asList (simpleTasks));
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
                    if (task.active && task.nextRunTime < System.currentTimeMillis ())
                        task.nextRunTime = task.onUpdate () + System.currentTimeMillis ();
                }

                //Exit program if stop requested, otherwise yield to other threads.
                NiFTFlow.yieldForFrame ();
            }
        }
    }
    private SimpleTaskUpdater taskUpdaterInstance = null;
    public void startPackage ()
    {
        if (taskUpdaterInstance == null)
        {
            taskUpdaterInstance = new SimpleTaskUpdater (groupName + " Task");
            taskUpdaterInstance.run ();
        }
    }
    public void stopPackage ()
    {
        if (taskUpdaterInstance != null)
        {
            taskUpdaterInstance.stop ();
            taskUpdaterInstance = null;
        }
    }
}
