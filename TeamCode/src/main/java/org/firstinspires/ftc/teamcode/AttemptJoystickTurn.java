package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.niftc.NiFTBase;
import org.firstinspires.ftc.teamcode.niftc.console.NiFTConsole;
import org.firstinspires.ftc.teamcode.niftc.hardware.NiFTInitializer;
import org.firstinspires.ftc.teamcode.niftc.threads.NiFTFlow;

@Autonomous(name="Attempt Joystick Turning", group="Test Bot")
public class AttemptJoystickTurn extends NiFTBase
{
    /*** CONFIGURE ALL ROBOT ELEMENTS HERE ***/
    //Drive motors (they are lists because it helps when we add on new motors.
    protected DcMotor toTurn;

    private void setMotorMode(DcMotor motor, DcMotor.RunMode runMode)
    {
        boolean doneSuccessfully = false;
        long additionalTime = 0;
        while (!doneSuccessfully)
        {
            try
            {
                motor.setMode (runMode);
                NiFTFlow.pauseForMS (100 + additionalTime);
                doneSuccessfully = true;
            }
            catch (Exception e)
            {
                NiFTConsole.outputNewSequentialLine("ERROR");
                if (e instanceof InterruptedException)
                    return;

                additionalTime += 20;
            }
        }
    }

    private int previousVal = 0;
    private int getDesiredJoystickHeading() {
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
            return previousVal;
        }

        double desiredHeading = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) * 180.0 / Math.PI - 90;

        if (desiredHeading < 0) {
            desiredHeading = 360 - Math.abs(desiredHeading); // if like -90 or something
        }

        int desiredHeadingInt = (int)(desiredHeading);

        previousVal = desiredHeadingInt;

        return desiredHeadingInt;
    }

    protected final void driverStationSaysGO() throws InterruptedException {
        final int encoderRevolution = 1600;

        NiFTConsole.outputNewSequentialLine("Started");

        toTurn = NiFTInitializer.initialize(DcMotor.class, "Left Motor");

        // Used to set the encoder to the appropriate position.
        setMotorMode(toTurn, DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorMode(toTurn, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(toTurn, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        NiFTConsole.ProcessConsole processConsole = new NiFTConsole.ProcessConsole("Position Console");
        while (!gamepad1.x) {
            int currentPosition = toTurn.getCurrentPosition();
            int desiredMotorPosition = (int)(getDesiredJoystickHeading() / 360.0 * encoderRevolution);

            processConsole.updateWith("X=" + gamepad1.left_stick_x + " Y=" + -gamepad1.left_stick_y + " so degree is " + getDesiredJoystickHeading());

            // Figure out whether we should continue the current rotation or backtrack (whichever is fastest).
            int positionSign = (int)(Math.signum(currentPosition));
            if (Math.abs(currentPosition - (desiredMotorPosition + encoderRevolution * positionSign)) > Math.abs(currentPosition - desiredMotorPosition)) {
                desiredMotorPosition += encoderRevolution * positionSign;
            }

            // Update the motor power if we're off of the desired position by some threshold.
            final int offFromDesired = desiredMotorPosition - currentPosition;
            if (Math.abs(offFromDesired) > 10) {
                toTurn.setPower(Math.signum(offFromDesired) * (.05 + Math.abs(offFromDesired) * .0005));
            } else {
                toTurn.setPower(0);
            }

            NiFTFlow.yieldForFrame();
        }

        toTurn.setPower(0);
        NiFTConsole.outputNewSequentialLine("Done");
        NiFTFlow.pauseForMS(2000);
    }
}