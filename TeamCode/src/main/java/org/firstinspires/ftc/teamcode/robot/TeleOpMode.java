package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Robot: Into the Deep", group = "Robot")
public class TeleOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // main wheel drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // scissor lift drive, extension drive, and claw servo
    private DcMotor scissorDrive = null;
    private DcMotor extensionDrive = null;
    private Servo clawServo = null;

    @Override
    public void runOpMode() {

        // *******************************************************************************************
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // *******************************************************************************************

        // main wheel drive motor hardware names
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // scissor drive, claw server, and extend / retract
        scissorDrive = hardwareMap.get(DcMotor.class, "scissorDrive");
        extensionDrive = hardwareMap.get(DcMotor.class, "extensionDrive");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // assign wheel motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // assign scissor, extension, and claw directions
        scissorDrive.setDirection(DcMotor.Direction.FORWARD);
        extensionDrive.setDirection(DcMotor.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // *******************************************************************************************
        // Wait for the game to start (driver presses START)
        // *******************************************************************************************
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // *******************************************************************************************
            // SECTION 1: Main Joystick Robot Driving
            // *******************************************************************************************

            // turbo mode speed overrides with left and right stick buttons
            // --------------------------------------------------------------------------------------------

            double turboOverrideLeftStick = 0.5;
            double turboOverrideRightStick = 0.5;
            // max wheel speed override
            if (gamepad1.left_stick_button) {
                turboOverrideLeftStick = 1.0;
            }
            if (gamepad1.right_stick_button) {
                turboOverrideRightStick = 1.0;
            }

            // start of code from ftc robot controller external examples
            // --------------------------------------------------------------------------------------------

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y * turboOverrideLeftStick;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x * turboOverrideLeftStick;
            double yaw = gamepad1.right_stick_x * turboOverrideRightStick;

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

            // *******************************************************************************************
            // SECTION 2: D-Pad Precision Robot Driving
            // *******************************************************************************************

            // TODO: adjust max speed between 0.0 and 1.0
            double precisionMax = 0.25;

            if (gamepad1.dpad_up && gamepad1.dpad_right) {
                // Move diagonally forward-right
                leftFrontPower = precisionMax;    // Front left wheel spins forward
                rightFrontPower = 0;              // Front right wheel stops
                leftBackPower = 0;                // Back left wheel stops
                rightBackPower = precisionMax;    // Back right wheel spins forward
            } else if (gamepad1.dpad_up && gamepad1.dpad_left) {
                // Move diagonally forward-left
                leftFrontPower = 0;                // Front left wheel stops
                rightFrontPower = precisionMax;    // Front right wheel spins forward
                leftBackPower = precisionMax;      // Back left wheel spins forward
                rightBackPower = 0;                // Back right wheel stops
            } else if (gamepad1.dpad_down && gamepad1.dpad_right) {
                // Move diagonally backward-right
                leftFrontPower = -precisionMax;    // Front left wheel spins backward
                rightFrontPower = 0;               // Front right wheel stops
                leftBackPower = 0;                 // Back left wheel stops
                rightBackPower = -precisionMax;    // Back right wheel spins backward
            } else if (gamepad1.dpad_down && gamepad1.dpad_left) {
                // Move diagonally backward-left
                leftFrontPower = 0;                // Front left wheel stops
                rightFrontPower = -precisionMax;   // Front right wheel spins backward
                leftBackPower = -precisionMax;     // Back left wheel spins backward
                rightBackPower = 0;                // Back right wheel stops
            } else if (gamepad1.dpad_up) {
                // Move forward
                leftFrontPower = precisionMax;
                rightFrontPower = precisionMax;
                leftBackPower = precisionMax;
                rightBackPower = precisionMax;
            } else if (gamepad1.dpad_down) {
                // Move backward
                leftFrontPower = -precisionMax;
                rightFrontPower = -precisionMax;
                leftBackPower = -precisionMax;
                rightBackPower = -precisionMax;
            } else if (gamepad1.dpad_left) {
                // Move left
                leftFrontPower = precisionMax;   // Front left wheel spins forward
                rightFrontPower = -precisionMax; // Front right wheel spins backward
                leftBackPower = -precisionMax;   // Back left wheel spins backward
                rightBackPower = precisionMax;   // Back right wheel spins forward
            } else if (gamepad1.dpad_right) {
                // Move right
                leftFrontPower = -precisionMax;  // Front left wheel spins backward
                rightFrontPower = precisionMax;  // Front right wheel spins forward
                leftBackPower = precisionMax;    // Back left wheel spins forward
                rightBackPower = -precisionMax;  // Back right wheel spins backward
            }

            // *******************************************************************************************
            // SECTION 3: Raise and Lower Scissor Lift
            // *******************************************************************************************

            // TODO: adjust max between 0.0 and 1.0
            double scissorDriveUpMax = 0.50;
            double scissorDriveDownMax = 0.50;
            double scissorDrivePower = 0.0;
            double scissorPosition = scissorDrive.getCurrentPosition();

            if (gamepad1.right_trigger > 0.1) {
                scissorDrivePower = gamepad1.right_trigger * scissorDriveUpMax;
            } else if (gamepad1.left_trigger > 0.1) {
                scissorDrivePower = -gamepad1.left_trigger * scissorDriveDownMax;
            }

            // *******************************************************************************************
            // SECTION 4: Extend and Retract Claw
            // *******************************************************************************************

            // TODO: adjust max between 0.0 and 1.0
            double extensionDriveFwdMax = 1.0;
            double extensionDriveBwdMax = 1.0;
            double extensionDrivePower = 0;
            double extensionPosition = extensionDrive.getCurrentPosition();
            if (gamepad1.right_bumper) {
                extensionDrivePower = extensionDriveFwdMax;
            } else if (gamepad1.left_bumper) {
                extensionDrivePower = -extensionDriveBwdMax;
            }

            // *******************************************************************************************
            // SECTION 5: Open and Close Claw
            // *******************************************************************************************

            // *******************************************************************************************
            // SECTION X: Write Outputs
            // *******************************************************************************************

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
            leftBackDrive.setPower(leftBackPower);

            // Power to scissor lift, extension
            scissorDrive.setPower(scissorDrivePower);
            extensionDrive.setPower(extensionDrivePower);

            // *******************************************************************************************
            // SECTION X: Telemetry
            // *******************************************************************************************

            // Show the elapsed game time and wheel power.
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine(String.format("Stick Override: L[%4.2f] R[%4.2f]", turboOverrideLeftStick, turboOverrideRightStick));
            telemetry.addLine(String.format("[%s]----[%s]", FormatPower(leftFrontPower, "%4.2f"), FormatPower(rightFrontPower, "%4.2f")));
            telemetry.addLine(String.format("[%s]----[%s]", FormatPower(leftBackPower, "%4.2f"), FormatPower(rightBackPower, "%4.2f")));
            telemetry.addLine(String.format("Scissor Lift: [%.0f]", scissorPosition));
            telemetry.addLine(String.format("Claw Extension: [%.0f]", extensionPosition));

            // Update telemetry
            telemetry.update();

        }
    }

    private String FormatPower(double leftFrontPower, String formatString) {
        String power = String.format(formatString, Math.abs(leftFrontPower));
        String direction = "";
        if (leftFrontPower > 0.01) {
            direction = "\u25B2"; // Up arrow
        } else if (leftFrontPower < -0.01) {
            direction = "\u25BC"; // Down arrow
        } else {
            direction = "\uFE00"; // No movement
        }
        return direction + power;
    }
}

