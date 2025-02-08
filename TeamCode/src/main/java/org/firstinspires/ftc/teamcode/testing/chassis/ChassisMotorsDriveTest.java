package org.firstinspires.ftc.teamcode.testing.chassis;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CHASSIS | Driving Test", group = "$$$$ Chassis")
public class ChassisMotorsDriveTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        // hardware map
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // motor directions (reverse right motors)
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // other variables
        double driveDirectionFactor = 1.0;

        //------------------------------------------------------------------------------------------------
        // Start Button
        //------------------------------------------------------------------------------------------------

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        //------------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            // read current positions
            int leftFrontTicksActual = leftFrontDrive.getCurrentPosition();
            int leftBackTicksActual = leftBackDrive.getCurrentPosition();
            int rightFrontTicksActual = rightFrontDrive.getCurrentPosition();
            int rightBackTicksActual = rightBackDrive.getCurrentPosition();

            // toggle backwards mode to use the pusher
            // --------------------------------------------------------------------------------------------
            if (gamepad1.back || gamepad2.back) {
                driveDirectionFactor = -1.0;
            } else if (gamepad1.start || gamepad2.start) {
                driveDirectionFactor = 1.0;
            }


            // turbo mode speed overrides with left and right stick buttons
            // --------------------------------------------------------------------------------------------
            double turboOverrideLeftStick = 0.25;
            double turboOverrideRightStick = 0.25;
            // max wheel speed override
            if (gamepad1.left_stick_button || gamepad2.left_stick_button) {
                turboOverrideLeftStick = 0.60;
            }
            if (gamepad1.right_stick_button || gamepad2.right_stick_button) {
                turboOverrideRightStick = 0.60;
            }

            // start of code from ftc robot controller external examples
            // --------------------------------------------------------------------------------------------

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = 0;
            double lateral = 0;
            double yaw = 0;

            // gamepad 1
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                // Note: pushing stick forward gives negative value
                axial = -gamepad1.left_stick_y * turboOverrideLeftStick * driveDirectionFactor;
            }
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                lateral = gamepad1.left_stick_x * turboOverrideLeftStick * driveDirectionFactor;
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                yaw = gamepad1.right_stick_x * turboOverrideRightStick;
            }

            // gamepad 2
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                // Note: pushing stick forward gives negative value
                axial = -gamepad2.left_stick_y * turboOverrideLeftStick * driveDirectionFactor;
            }
            if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                lateral = gamepad2.left_stick_x * turboOverrideLeftStick * driveDirectionFactor;
            }
            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                yaw = gamepad2.right_stick_x * turboOverrideRightStick;
            }

            // D-Pad overrides for the same joystick functions as above
            // --------------------------------------------------------------------------------------------
            // TODO: adjust max speed between 0.0 and 1.0
            double precisionMax = 0.20;
            if (!(gamepad1.dpad_up && gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_right)) {
                if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    axial = precisionMax * driveDirectionFactor;
                }
                if (gamepad1.dpad_down || gamepad2.dpad_down) {
                    axial = -precisionMax * driveDirectionFactor;
                }
                if (gamepad1.dpad_left || gamepad2.dpad_left) {
                    lateral = -precisionMax * driveDirectionFactor;
                }
                if (gamepad1.dpad_right || gamepad2.dpad_right) {
                    lateral = precisionMax * driveDirectionFactor;
                }
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // update telemetry data
            telemetry.addLine("Use the left and right sticks to dirve.");
            telemetry.addLine("Pressing the sticks will enable turbo.");
            telemetry.addLine("Pressing BACK makes pusher forward control.");
            telemetry.addLine("Pressing START makes claw forward control.");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addData("Driving Direction", driveDirectionFactor < 0 ? "!!! PUSHER FORWARD !!!" : "!!! CLAW FORWARD !!!");
            telemetry.addLine(String.format("Turbo Override: L[%4.2f] R[%4.2f]", turboOverrideLeftStick, turboOverrideRightStick));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("PWR: [%4.2f]-<>-[%4.2f]", leftFrontPower, rightFrontPower));
            telemetry.addLine(String.format("PWR: [%4.2f]-<>-[%4.2f]", leftBackPower, rightBackPower));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("TICKS: [%d]-<>-[%d]", leftFrontTicksActual, rightFrontTicksActual));
            telemetry.addLine(String.format("TICKS: [%d]-<>-[%d]", leftBackTicksActual, rightBackTicksActual));
            telemetry.update();

            idle();

        }

    }
}
