package org.firstinspires.ftc.teamcode.niftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.music.NiFTMusic;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

/**
 * NiFTBase is the class from which all user OpModes should inherit.  With advanced error handling, it takes care of the scenarios in which the user requests an early stop, fails to take an error into account, etc.
 */
public abstract class NiFTBase extends LinearOpMode
{
    /**
     * Useful for other files which require custom initialization steps or components from this op mode which they cannot otherwise obtain.
     */
    public static LinearOpMode opModeInstance;

    /**
     * runOpMode() is the method called by LinearOpMode to start the program, but is really low-level.  What this method does is split the sequence into a set of steps which every autonomous program should include, while also observing errors and either stopping the code or outputting them based on their severity.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode () throws InterruptedException
    {
        try
        {
            //Classes such as NiFTMusic require this so that they can get the context they require.
            opModeInstance = this;
            NiFTConsole.reset ();

            //REQUIRED in child classes.
            initializeHardware ();

            //May be used in different programs.
            driverStationSaysINITIALIZE ();

            //Wait for the start button to be pressed.
            waitForStart ();

            //This is where the child classes mainly differ in their instructions.
            driverStationSaysGO ();
        }
        catch (InterruptedException e) {} //If this is caught, then the user requested program stop.
        catch (Exception e) //If this is caught, it wasn't an InterruptedException and wasn't requested, so the user is notified.
        {
            NiFTConsole.outputNewSequentialLine ("UH OH!  An error was just thrown!");
            NiFTConsole.outputNewSequentialLine (e.getMessage ());
            NiFTConsole.outputNewSequentialLine ("Will end upon tapping stop...");

            //Wait until stop is requested.
            try
            {
                while (true)
                    NiFTFlow.pauseForSingleFrame ();
            }
            catch (InterruptedException e2) {} //The user has read the message and stops the program.
        }
        finally //Occurs after all possible endings.
        {
            NiFTMusic.quit ();
            driverStationSaysSTOP ();
        }
    }

    /**
     * Each method below are the main methods with which you can run your OpModes.  This enables quick and easy access to required methods.  Edit at will, but be careful to avoid messing with the exceptions too much.
     *
     * Those which are abstract must be overridden in child classes with actual code.  Those which are not are optionally modifiable.
     */
    protected void initializeHardware () throws InterruptedException {};
    protected void driverStationSaysINITIALIZE () throws InterruptedException {}
    protected abstract void driverStationSaysGO () throws InterruptedException;
    protected void driverStationSaysSTOP () {}
}