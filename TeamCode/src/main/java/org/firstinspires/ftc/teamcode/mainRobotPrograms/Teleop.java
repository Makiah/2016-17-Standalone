package org.firstinspires.ftc.teamcode.mainRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

//Add the teleop to the op mode register.
@TeleOp(name="Teleop - Badass Mode", group="Teleop Group")
//@Disabled

public class Teleop extends _RobotBase
{
    //Not really required, just initialize everything that needs to be implemented in teleop.
    protected Servo leftSensorServo, rightSensorServo;
    protected final double RIGHT_SERVO_CLOSED = 1.0, LEFT_SERVO_CLOSED = 1.0;

    //All teleop controls are here.
    protected void driverStationSaysGO() throws InterruptedException
    {
        //Audio Control Variables

        //Normal mode variables
        double leftPower, rightPower;
        boolean backwards = false;
        double lastTimeToggleDirectionPressed = 0;

        //Other motor variables

        //Keep looping while opmode is active (waiting a hardware cycle after all of this is completed, just like loop()).
        while (opModeIsActive())
        {
            /******************** DRIVING CONTROL ********************/
            //Driving Toggle
            if (!backwards)
            { // Driving forward
                leftPower = -gamepad1.right_stick_y;
                rightPower = -gamepad1.left_stick_y;
            } else { // Driving backward
                leftPower = gamepad1.left_stick_y;
                rightPower = gamepad1.right_stick_y;
            }

            // clip the right/left values so that the values never exceed +/- 1
            rightPower = Range.clip(rightPower, -1, 1);
            leftPower = Range.clip(leftPower, -1, 1);

            // Write the values to the motors.  Scale the robot in order to run the robot more effectively at slower speeds.
            for (DcMotor lMotor : leftDriveMotors)
                lMotor.setPower(scaleInput(leftPower));
            for (DcMotor rMotor : rightDriveMotors)
                rMotor.setPower(scaleInput(rightPower));

            //Wait a second before switching to backwards again (can only toggle once every second).
            if (gamepad1.back && (System.currentTimeMillis() - lastTimeToggleDirectionPressed) > 1000)
            {
                backwards = !backwards; // Switch driving direction
                lastTimeToggleDirectionPressed = System.currentTimeMillis();
                outputNewLineToDriverStation("Toggled drive mode to " + (backwards ? "backwards" : "forwards"));
            }

            /******************** OTHER MOTORS ********************/

            //Harvester (hopefully just this simple)
            if (gamepad2.b)
                harvester.setPower(-0.5);
            else if (gamepad2.a)
                harvester.setPower(0.5);
            else
                harvester.setPower(0);

            //Pusher
            if (gamepad2.dpad_left)
                pusher.setPower(-0.3);
            else if (gamepad2.dpad_right)
                pusher.setPower(0.3);
            else
                pusher.setPower(0);

            idle();

            outputConstantLinesToDriverStation(
                    new String[]
                    {
                            "Current distance is " + opticalDistanceSensor.getLightDetected()
                    }
            );

            /******************** END OF LOOP ********************/
        }
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */

    double scaleInput(double dVal)
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

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}