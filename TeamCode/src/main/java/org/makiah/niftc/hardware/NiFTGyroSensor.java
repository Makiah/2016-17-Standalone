package org.makiah.niftc.hardware;

import com.qualcomm.robotcore.hardware.GyroSensor;

import org.makiah.niftc.threads.NiFTFlow;

/**
 * Encapsulates the gyro sensor in an easy to access set of methods.
 */
public class NiFTGyroSensor
{
    public final GyroSensor sensor;

    public NiFTGyroSensor (String gyroSensorName) throws InterruptedException
    {
        this.sensor = NiFTInitializer.initialize (GyroSensor.class, gyroSensorName);

        calibrate (true);
    }

    /**
     * Calibrates the gyro and optionally zeroes the heading.
     *
     * @param zeroHeading if true, it zeroes the heading as well.
     * @throws InterruptedException
     */
    public void calibrate(boolean zeroHeading) throws InterruptedException
    {
        //Pause to prevent odd errors in which it says it's configured but is actually LYING.
        NiFTFlow.pauseForMS (1000);

        //Wait for gyro to finish calibrating.
        while (sensor.isCalibrating())
            NiFTFlow.pauseForMS (50);

        //Zero gyro heading.
        if (zeroHeading)
            zeroHeading();
    }

    //Just resets the gyro.
    public void zeroHeading() throws InterruptedException
    {
        NiFTFlow.pauseForMS (400);
        sensor.resetZAxisIntegrator();
        NiFTFlow.pauseForMS (400);
    }

    /**
     * OK SO
     * The gyro output is weird.  This makes it easier to access these values, by providing positive values if turning left and negative values if turning right.
     * However, if we turn to heading 190, then we will have an error as it tries to calculate getting to this position and can't get a value greater than 180 (190 is -170 with this method)
     * So this method incorporates the desired heading and the current heading and the inability of the robot to turn super fast to return calculation-valid values.
     */
    public int getValidGyroHeading()
    {
        //Get the heading.
        int heading = sensor.getHeading ();

        //Determine the actual heading on a logical basis (which makes sense with the calculations).
        if (heading > 180 && heading <= 360)
            heading -= 360;

        //What this does is enable the 180 degree turn to be effectively made without resulting in erratic movement.
        if (desiredHeading > 160 && heading < 0)
            heading += 360;
        else if (desiredHeading < -160 && heading > 0)
            heading -= 360;

        return heading;
    }

    //Set this value before attempting a turn with the method below.
    private int desiredHeading = 0;
    public void setDesiredHeading(int desiredHeading)
    {
        this.desiredHeading = desiredHeading;
    }
    public int getDesiredHeading() { return desiredHeading; }

    public int getOffFromHeading()
    {
        return desiredHeading - getValidGyroHeading ();
    }
}
