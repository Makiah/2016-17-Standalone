package org.firstinspires.ftc.teamcode.driverchoices;

import java.io.Console;
import java.util.HashMap;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ImprovedOpModeBase;
import org.firstinspires.ftc.teamcode.debugging.ConsoleManager;

@TeleOp(name="TankBot Drive", group="TankBot Group")

public class TankBotDriveAround extends ImprovedOpModeBase
{
    private static final double MOTOR_CORRECTION_FACTOR = -0.1;
    private static final double TRIGGER_MIN_POSITION = 0.5;
    private static final double TURRET_SPEED = 0.005;

    private enum Motors
    {
        LEFT, RIGHT;
    }

    private Servo turret;
    private DcMotor lMotor, rMotor;

    /* Called on initialization (once) */
    protected void initializeHardware() throws InterruptedException
    {
        turret = initialize(Servo.class, "Turret");
        lMotor = initialize(DcMotor.class, "Left Motor");
        rMotor = initialize(DcMotor.class, "Right Motor");

        lMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /* Teleop controls. */
    protected void driverStationSaysGO() throws InterruptedException
    {
        double lPower, rPower;
        boolean backwards = false;
        double currentTurretPosition = 0.5;

        while (opModeIsActive())
        {
            /* Going backwards? */
            if (!backwards)
            {
                lPower = gamepad1.left_stick_y;
                rPower = gamepad1.right_stick_y;
            } else
            {
                lPower = -gamepad1.right_stick_y;
                rPower = -gamepad1.left_stick_y;
            }

            /* Control and set speed. */
            setPower(scaleInput(Range.clip(lPower, -1, 1)), Motors.LEFT);
            setPower(scaleInput(Range.clip(rPower, -1, 1)), Motors.RIGHT);

            if(canPress("a"))
            {
                backwards = !backwards;
            }
            if(canPress("y"))
            {
                MediaPlayerWrapper.playAudio(MediaPlayerWrapper.DownloadedSongs.IMPERIAL_MARCH);
            }

            /* Turret position */
            if (turret != null)
            {
                if (gamepad1.right_trigger > TRIGGER_MIN_POSITION)
                {
                    currentTurretPosition -= TURRET_SPEED;
                }
                else if (gamepad1.left_trigger > TRIGGER_MIN_POSITION)
                {
                    currentTurretPosition += TURRET_SPEED;
                }

                turret.setPosition(Range.clip(currentTurretPosition, 0, 1));
            }

            /* Data output. */
            ConsoleManager.outputConstantDataToDrivers(
                new String[] {"Right power = " + rPower, "Music playing = " + (MediaPlayerWrapper.isPlaying())}
            );

            idle();
        }
    }

    protected void driverStationSaysSTOP()
    {
        MediaPlayerWrapper.stopAudio();
        setPower(0, Motors.LEFT);
        setPower(0, Motors.RIGHT);
    }

    protected void setPower(double power, Motors motorID)
    {
        (motorID.equals(Motors.LEFT) ? lMotor : rMotor).setPower(power);
    }

    /**
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    private double scaleInput(double dVal)
    {
        double[] scaleArr = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        /* Get corresponding index. */
        int i = ((int)(dVal * 16.0)) % 16;
        i = i < 0 ? -i : i;

        return dVal < 0 ? -scaleArr[i] : scaleArr[i];
    }

    /**
     * Checks if enough time has passed that a button can be pressed, and if
     * that button is being pressed at all.
     *
     * Todo: A debounce may be more effective.
     * Todo: Extrememly time expensive (use more variables).
     */
    private boolean canPress(String button) // Todo: Figure out the real class.
    {
        try
        {
            /* The game button and the last time it was pressed. */
            final HashMap<String, Long> lastPressed = new HashMap<>(2); // Todo: Shouldn't be created more than once. May need to move to a field.
            boolean beingPressed = (boolean)gamepad1.getClass().getDeclaredField(button).get(gamepad1); // TODO: Eww...

            if(beingPressed)
            {
            /* As buttons are pressed, they are added to the map so their times can be tracked.*/
                if(!lastPressed.containsKey(button)) // TODO: Probaby can be simplified with native methods.
                {
                    lastPressed.put(button, System.currentTimeMillis());
                }

                if((System.currentTimeMillis() - lastPressed.get(button)) > 500)
                {
                    lastPressed.put(button, System.currentTimeMillis())
                    return true;
                }
                return false;
            }
        } catch (NoSuchFieldException e)
        {
            ConsoleManager.outputNewLineToDrivers("No such button found.");
        } catch (IllegalAccessException e)
        {
            ConsoleManager.outputNewLineToDrivers("An error while attempting to get the button, I think.");
        }
    }
}
