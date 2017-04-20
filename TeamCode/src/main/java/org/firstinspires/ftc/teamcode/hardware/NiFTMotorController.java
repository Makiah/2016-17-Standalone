package org.firstinspires.ftc.teamcode.hardware;

import android.os.AsyncTask;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.threads.NiFTAsyncTask;
import org.firstinspires.ftc.teamcode.threads.NiFTFlow;

//My own take on PID, not great but it works
public class NiFTMotorController
{
    //Only certain motors have encoders on them, so the linkedMotor object is implemented.
    public final String name;
    public final DcMotor encoderMotor, linkedMotor;

    //Initialization steps
    public NiFTMotorController (String name, String encoderMotorName)
    {
        this (name, encoderMotorName, null);
    }

    public NiFTMotorController (String name, String encoderMotorName, String linkedMotorName)
    {
        this.name = name;

        this.encoderMotor = NiFTInitializer.initialize (DcMotor.class, encoderMotorName);
        if (linkedMotorName != null)
            this.linkedMotor = NiFTInitializer.initialize (DcMotor.class, linkedMotorName);
        else
            this.linkedMotor = null;

        resetEncoder ();
    }

    //Initial conversion factor, will be changed a LOT through the course of the program.
    private double rpsConversionFactor = .25;

    public double getRPSConversionFactor () //Primarily for debugging.
    {
        return rpsConversionFactor;
    }

    public NiFTMotorController setRPSConversionFactor (double rpsConversionFactor)
    {
        //Bounds are 0.005 to 5, since greater than that would have to be a glitch.
        this.rpsConversionFactor = Range.clip (rpsConversionFactor, 0.005, 5);
        return this;
    }

    //Motor type
    public enum MotorType
    {
        NeverRest40 (1120), NeverRest20 (1120), NeverRest3P7 (45);

        public final int encoderTicksPerRevolution;

        MotorType (int encoderTicksPerRevolution)
        {
            this.encoderTicksPerRevolution = encoderTicksPerRevolution;
        }
    }

    private MotorType motorType = MotorType.NeverRest40;

    public NiFTMotorController setMotorType (MotorType motorType)
    {
        this.motorType = motorType;
        return this;
    }

    public NiFTMotorController setMotorDirection (DcMotorSimple.Direction direction)
    {
        encoderMotor.setDirection (direction);
        if (linkedMotor != null)
            linkedMotor.setDirection (direction);

        return this;
    }

    //Adjustment sensitivity.
    private double sensitivity = .00002;

    public NiFTMotorController setAdjustmentSensitivity (double sensitivity)
    {
        this.sensitivity = sensitivity;
        return this;
    }

    //Bounds for adjustment
    private double sensitivityBound = .5;

    public NiFTMotorController setAdjustmentSensitivityBounds (double sensitivityBound)
    {
        this.sensitivityBound = sensitivityBound;
        return this;
    }

    //Refresh rate.
    private long refreshRate = 50;

    public NiFTMotorController setRefreshRate (long refreshRate)
    {
        this.refreshRate = refreshRate;
        return this;
    }

    /***** ENCODER MANAGEMENT *****/
    public void resetEncoder ()
    {
        boolean doneSuccessfully = false;
        long additionalTime = 0;
        while (!doneSuccessfully)
        {
            try
            {
                encoderMotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
                NiFTFlow.pauseForMS (100 + additionalTime);
                doneSuccessfully = true;
            } catch (Exception e)
            {
                if (e instanceof InterruptedException)
                    return;

                additionalTime += 20;
            }
        }

        doneSuccessfully = false;
        additionalTime = 0;
        while (!doneSuccessfully)
        {
            try
            {
                encoderMotor.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                NiFTFlow.pauseForMS (100 + additionalTime);
                doneSuccessfully = true;
            } catch (Exception e)
            {
                if (e instanceof InterruptedException)
                    return;

                additionalTime += 20;
            }
        }

        doneSuccessfully = false;
        while (!doneSuccessfully)
        {
            try
            {
                encoderMotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                NiFTFlow.pauseForMS (100 + additionalTime);
                doneSuccessfully = true;
            }
            catch (InterruptedException e)
            {
                return;
            }
            catch (Exception e)
            {
                additionalTime += 20;
            }
        }
    }

    /******* THREADING *********/
    private final class PIDTask extends NiFTAsyncTask
    {
        private final NiFTConsole.ProcessConsole pidConsole;

        public PIDTask ()
        {
            super(name + " PID");
            pidConsole = new NiFTConsole.ProcessConsole (name + " PID Console");
        }

        @Override
        protected void onBeginTask () throws InterruptedException
        {
            while (true)
            {
                if (System.currentTimeMillis () - lastAdjustTime >= refreshRate)
                {
                    updateMotorPowerWithPID ();

                    pidConsole.updateWith (
                            "Current RPS conversion = " + rpsConversionFactor,
                            "Expected = " + getExpectedTicksSinceUpdate (),
                            "Actual = " + getActualTicksSinceUpdate ()
                    );
                }

                NiFTFlow.pauseForMS (30);
            }
        }

        @Override
        protected void onQuitTask ()
        {
            pidConsole.destroy ();
        }
    }

    private PIDTask pidInstance;

    public void startPIDTask ()
    {
        if (pidInstance == null)
        {
            pidInstance = new PIDTask ();
            pidInstance.run ();
        }
    }

    public void stopPIDTask ()
    {
        if (pidInstance != null)
        {
            pidInstance.stop();
            pidInstance = null;
        }
    }

    /******* PID STUFF *********/
    private double desiredRPS = 0;

    //Stored for each startEasyTask.
    private int previousMotorPosition;
    private long lastAdjustTime = 0;

    private double expectedTicksPerSecond;

    public void setRPS (double givenRPS)
    {
        //Will soon be modified by PID.
        desiredRPS = givenRPS;
        updateMotorPowers ();

        recordLastState ();

        expectedTicksPerSecond = motorType.encoderTicksPerRevolution * desiredRPS;
    }

    private double expectedTicksSinceUpdate, actualTicksSinceUpdate;

    public double getExpectedTicksSinceUpdate ()
    {
        return expectedTicksSinceUpdate;
    }

    public double getActualTicksSinceUpdate ()
    {
        return actualTicksSinceUpdate;
    }

    private void updateMotorPowerWithPID ()
    {
        if (lastAdjustTime != 0)
        {
            expectedTicksSinceUpdate = expectedTicksPerSecond * ((System.currentTimeMillis () - lastAdjustTime) / 1000.0);

            actualTicksSinceUpdate = encoderMotor.getCurrentPosition () - previousMotorPosition;

            //Sensitivity is the coefficient below, and bounds are .5 and -.5 so that momentary errors don't result in crazy changes.
            setRPSConversionFactor (rpsConversionFactor + Math.signum (desiredRPS) * Range.clip (((expectedTicksSinceUpdate - actualTicksSinceUpdate) * sensitivity), -sensitivityBound, sensitivityBound));

            updateMotorPowers ();
        }

        recordLastState ();
    }

    private void recordLastState ()
    {
        previousMotorPosition = encoderMotor.getCurrentPosition ();
        lastAdjustTime = System.currentTimeMillis ();
    }

    private void updateMotorPowers ()
    {
        //Set the initial power which the PID will soon modify.
        double desiredPower = Range.clip (desiredRPS * rpsConversionFactor, -1, 1);
        encoderMotor.setPower (desiredPower);
        if (linkedMotor != null)
            linkedMotor.setPower (desiredPower);
    }

    //Used rarely but useful when required.
    public void setDirectMotorPower (double power)
    {
        double actualPower = Range.clip (power, -1, 1);
        encoderMotor.setPower (actualPower);
        if (linkedMotor != null)
            linkedMotor.setPower (actualPower);
    }
}
