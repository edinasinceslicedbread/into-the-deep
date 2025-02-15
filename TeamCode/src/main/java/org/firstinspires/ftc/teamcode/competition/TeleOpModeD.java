package org.firstinspires.ftc.teamcode.competition;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ctrl.ArmControllerV2;

@TeleOp(name = "$$$ TELE-D (SIGMA)", group = "$$$")
public class TeleOpModeD extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------

    // chassis
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // scissor
    private DcMotorEx scissorDrive = null;
    private TouchSensor scissorLoSensor = null;

    // claw
    private Servo clawServo = null;
    ColorSensor colorSensor = null;    // Hardware Device Object

    // arm
    private ArmControllerV2 armController = new ArmControllerV2();

    @SuppressLint("DefaultLocale")
    @Override

    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        // chassis
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // scissor
        scissorDrive = hardwareMap.get(DcMotorEx.class, "scissorDrive");
        scissorLoSensor = hardwareMap.get(TouchSensor.class, "scissorLoSensor");

        // claw
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // arm
        armController.initialize(hardwareMap);

        //------------------------------------------------------------------------------------------------
        // Advanced Setup
        //------------------------------------------------------------------------------------------------

        // chassis motor directions (reverse right motors)
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // other variables
        double driveDirectionFactor = 1.0;
        double clawOpenPosition = 0.50;
        double clawClosePosition = 0.05;

        //------------------------------------------------------------------------------------------------
        // Start Button
        //------------------------------------------------------------------------------------------------

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // arm and claw reset
        armController.reset();
        clawServo.setPosition(clawOpenPosition);

        //------------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------------
            // Chassis Control
            //------------------------------------------------------------------------------------------------

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
            double turboOverrideAxial  = 0.25;       // default maximum axial speed
            double turboOverrideLateral = 0.35;       // default maximum axial speed
            double turboOverrideTurning = 0.25;      // default maximum yaw speed
            // max wheel speed override
            if (gamepad1.left_stick_button || gamepad2.left_stick_button) {
                turboOverrideAxial = 0.50;
                turboOverrideLateral = 0.60;
            }
            if (gamepad1.right_stick_button || gamepad2.right_stick_button) {
                turboOverrideTurning = 0.50;
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
                axial = -gamepad1.left_stick_y * turboOverrideAxial * driveDirectionFactor;
            }
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                lateral = gamepad1.left_stick_x * turboOverrideLateral * driveDirectionFactor;
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                yaw = gamepad1.right_stick_x * turboOverrideTurning;
            }

            // gamepad 2
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                // Note: pushing stick forward gives negative value
                axial = -gamepad2.left_stick_y * turboOverrideAxial * driveDirectionFactor;
            }
            if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                lateral = gamepad2.left_stick_x * turboOverrideLateral * driveDirectionFactor;
            }
            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                yaw = gamepad2.right_stick_x * turboOverrideTurning;
            }

            // D-Pad overrides for the same joystick functions as above
            // --------------------------------------------------------------------------------------------
            double dpadAxialMax = 0.20;
            double dpadLateralMax = 0.30;

            // gamepad 1
            boolean dpadButtons1 = gamepad1.dpad_up && gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_right;
            boolean dpadButtons2 = gamepad2.dpad_up && gamepad2.dpad_down && gamepad2.dpad_left && gamepad2.dpad_right;
            if (!(dpadButtons1 || dpadButtons2)) {
                if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    axial = dpadAxialMax * driveDirectionFactor;
                }
                if (gamepad1.dpad_down || gamepad2.dpad_down) {
                    axial = -dpadAxialMax * driveDirectionFactor;
                }
                if (gamepad1.dpad_left || gamepad2.dpad_left) {
                    lateral = -dpadLateralMax * driveDirectionFactor;
                }
                if (gamepad1.dpad_right || gamepad2.dpad_right) {
                    lateral = dpadLateralMax * driveDirectionFactor;
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

            //------------------------------------------------------------------------------------------------
            // Scissor Control
            //------------------------------------------------------------------------------------------------

            // read position and limit switch
            int scissorTicksActual = scissorDrive.getCurrentPosition();
            boolean scissorLoPressed = scissorLoSensor.isPressed();

            // no power without triggers
            double scissorPower = 0;

            // right trigger moves scissor up to limited value
            if (gamepad1.right_trigger > 0.05) {
                scissorPower = gamepad1.right_trigger * 0.8;
            } else if (gamepad2.right_trigger > 0.05) {
                scissorPower = gamepad2.right_trigger * 0.8;
            }

            // left trigger moves scissor down to limited value
            if (gamepad1.left_trigger > 0.05) {
                scissorPower = -gamepad1.left_trigger * 0.4;
            } else if (gamepad2.left_trigger > 0.05) {
                scissorPower = -gamepad2.left_trigger * 0.4;
            }

            // upper limit position override
            if (scissorPower > 0.0 && scissorTicksActual > 7900.0) {
                scissorPower = 0;
            }

            // lower limit switch override
            if (scissorPower < 0.0 && scissorLoPressed) {
                scissorPower = 0;
            }

            // output the power
            scissorDrive.setPower(scissorPower);

            //------------------------------------------------------------------------------------------------
            // Claw Control
            //------------------------------------------------------------------------------------------------
            // left bumper opens
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                clawServo.setPosition(clawOpenPosition);
            }

            // right bumper closes
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                clawServo.setPosition(clawClosePosition);
            }

            // claw compensation
            if (gamepad1.x || gamepad2.x) {
                clawServo.setPosition(clawOpenPosition);
            }

            //------------------------------------------------------------------------------------------------
            // arm Control
            //------------------------------------------------------------------------------------------------
            armController.update(runtime, gamepad1, gamepad2);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the armController.shoulder.");
            telemetry.addLine(String.format("X -> S:[%s] E:[%s] Home Position", armController.shoulderState.atHomePos, armController.elbowState.atHomePos));
            telemetry.addLine(String.format("Y -> S:[%s] E:[%s] Chamber", armController.shoulderState.atChamberPos, armController.elbowState.atChamberPos));
            telemetry.addLine(String.format("Y -> S:[%s] E:[%s] Basket", armController.shoulderState.atBasketPos, armController.elbowState.atBasketPos));
            telemetry.addLine(String.format("A -> S:[%s] E:[%s] Stop Motion", armController.shoulderController.atGoal(), armController.elbowController.atGoal()));
            telemetry.addLine(String.format("B -> S:[%s] E:[%s] Pick at Submersible", armController.shoulderState.atExtendPos, armController.elbowState.atExtendPos));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Power: FFW[%4.2f] PID[%4.2f]", armController.shoulderState.motorPowerFFW, armController.shoulderState.motorPowerPID));
            telemetry.addLine(String.format("Shoulder Target: GOAL[%d] SP[%d]", (int) Math.round(armController.shoulderController.getGoal().position), (int) Math.round(armController.shoulderController.getSetpoint().position)));
            telemetry.addLine(String.format("Shoulder Status: PWR[%4.2f] TICKS[%d]", armController.shoulderState.motorPower, armController.shoulderState.currentPosActual));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Shoulder Error: POS[%4.2f] VEL[%4.2f]", armController.shoulderController.getPositionError(), armController.shoulderController.getVelocityError()));
            telemetry.addLine(String.format("At: Goal[%s] SetPoint[%s]", armController.shoulderController.atGoal(), armController.shoulderController.atSetpoint()));
            telemetry.addLine(String.format("Control State: [%d]", armController.armControlState));
            telemetry.update();

        }
    }

}

