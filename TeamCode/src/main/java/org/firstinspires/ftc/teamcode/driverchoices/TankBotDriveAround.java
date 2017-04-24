package org.firstinspires.ftc.teamcode.driverchoices;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.makiah.niftc.NiFTBase;
import org.makiah.niftc.hardware.NiFTInitializer;
import org.makiah.niftc.music.NiFTMusic;

//Add the teleop to the op mode register.
@TeleOp(name="TankBot Drive", group="TankBot Group")

public class TankBotDriveAround extends NiFTBase
{
    /*** CONFIGURE ALL ROBOT ELEMENTS HERE ***/
    //Drive motors (they are lists because it helps when we add on new motors.
    private DcMotor leftMotor, rightMotor;
    private Servo turret;
    private double motorCorrectionFactor = -0.1;

    // Called on initialization (once)
    protected void initializeHardware() throws InterruptedException
    {
        /*----------------- DRIVING MOTORS ---------------*/
        leftMotor = NiFTInitializer.initialize(DcMotor.class, "Left Motor");
        rightMotor = NiFTInitializer.initialize(DcMotor.class, "Right Motor");
        leftMotor.setDirection (DcMotorSimple.Direction.REVERSE);

        turret = NiFTInitializer.initialize(Servo.class, "Turret");
    }

    private void setRightPower(double power)
    {
        rightMotor.setPower (power * (1 - motorCorrectionFactor));
    }

    private void setLeftPower(double power)
    {
        leftMotor.setPower (power * (1 + motorCorrectionFactor));
    }

    //All teleop controls are here.
    protected void driverStationSaysGO() throws InterruptedException
    {
        double leftPower, rightPower;
        boolean backwards = false;
        long lastTimeBackTogglePressed = System.currentTimeMillis(),
                lastTimeMusicTogglePressed = System.currentTimeMillis ();
        double currentTurretPosition = 0.5;

        //Keep looping while opmode is active (waiting a hardware cycle after all of this is completed, just like loop())
        while (opModeIsActive())
        {
            /*-------------------- CONTROLLER #1 ---------------------*/
            /*-------------- Direction Toggle -------------*/
            if (!backwards)
            { // Driving forward
                leftPower = gamepad1.left_stick_y;
                rightPower = gamepad1.right_stick_y;
            } else { // Driving backward
                leftPower = -gamepad1.right_stick_y;
                rightPower = -gamepad1.left_stick_y;
            }

            /*------------- Motor Speed Control -------------*/
            rightPower = Range.clip(rightPower, -1, 1);
            leftPower = Range.clip(leftPower, -1, 1);

            // Write the values to the motors.  Scale the robot in order to startEasyTask the robot more effectively at slower speeds.
            setLeftPower (scaleInput(leftPower));
            setRightPower (scaleInput(rightPower));

            //Toggle direction
            if (gamepad1.a && (System.currentTimeMillis() - lastTimeBackTogglePressed) > 500)
            {
                backwards = !backwards;
                lastTimeBackTogglePressed = System.currentTimeMillis();
            }

            if (gamepad1.y && (System.currentTimeMillis () - lastTimeMusicTogglePressed) > 500)
            {
                if (NiFTMusic.playing())
                    NiFTMusic.quit ();
                else
                    NiFTMusic.play (NiFTMusic.AudioFiles.IMPERIAL_MARCH);
            }

            /*-------------- Turret Position --------------*/
            if (turret != null)
            {
                if (gamepad1.right_trigger > 0.5)
                    currentTurretPosition -= 0.005;
                else if (gamepad1.left_trigger > 0.5)
                    currentTurretPosition += 0.005;

                currentTurretPosition = Range.clip(currentTurretPosition, 0, 1);
                turret.setPosition(currentTurretPosition);
            }

            /*---------------- END OF LOOP ---------------*/

            idle();

        }
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */

    private double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}