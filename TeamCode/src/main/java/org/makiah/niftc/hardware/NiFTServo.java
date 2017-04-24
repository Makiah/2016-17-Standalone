package org.makiah.niftc.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Since servos often are constrained to a certain range of movement, this class includes this data in an easier to observe format, which is more effectively encapsulated.
 */
public class NiFTServo
{
    public final Servo servo;
    private final double upperLimit, lowerLimit;
    private double position;

    /**
     * Initializes a given servo with the name provided, and the bounds of movement.
     */
    public NiFTServo (String servoName, double lowerLimit, double initialPosition, double upperLimit)
    {
        this.servo = NiFTInitializer.initialize (Servo.class, servoName);
        this.lowerLimit = Range.clip(lowerLimit, 0, 1);
        this.upperLimit = Range.clip(upperLimit, 0, 1);
        this.position = Range.clip(initialPosition, lowerLimit, upperLimit);
        updateServoPosition ();
    }
    public NiFTServo (String servoName, double initialPosition)
    {
        this(servoName, 0, initialPosition, 1);
    }

    //Methods which are used to get/set servo position.
    public void setToUpperLim()
    {
        setServoPosition (upperLimit);
    }
    public void setToLowerLim()
    {
        setServoPosition (lowerLimit);
    }
    public void setServoPosition(double desiredPosition)
    {
        position = Range.clip(desiredPosition, lowerLimit, upperLimit);
        updateServoPosition ();
    }
    private void updateServoPosition()
    {
        servo.setPosition (position);
    }
    public double getServoPosition()
    {
        return position;
    }
}
