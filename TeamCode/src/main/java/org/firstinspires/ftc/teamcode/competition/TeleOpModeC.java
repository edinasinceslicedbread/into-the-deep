package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ctrl.ArmController;
import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;


@TeleOp(name = "$$$ TELE-C (BETA)", group = "$$$")
@Disabled
public class TeleOpModeC extends LinearOpMode {

    // scissor lift constantszoom
    static final double SCISSOR_MIN_POS = 1000;    // Minimum scissor lift encoder position
    static final double SCISSOR_MAX_POS = 10000;     // Maximum scissor lift encoder position

    // claw extension constants
    static final double EXTENSION_MIN_POS = -3000;  // Minimum claw extension encoder position
    static final double EXTENSION_MAX_POS = 5000;   // Maximum claw extension encoder position

    // claw gripper constants
    static final int CYCLE_MS = 50;             // period of each cycle
    static final double CLAW_MIN_POS = 0.0;     // Minimum rotational position
    static final double CLAW_MAX_POS = 1.0;     // Maximum rotational position

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // main wheel drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // scissor lift drive, extension drive, and claw servo
    private DcMotor scissorDrive = null;
    private Servo clawServo = null;
    private DcMotorEx elbowMotor = null;
    private DcMotor shoulderDrive = null;
    private ColorRangeSensor colorSensor;

    private RisingEdgeTrigger elbowTriggerUp = new RisingEdgeTrigger();
    private RisingEdgeTrigger elbowTriggerDown = new RisingEdgeTrigger();

    // digital limit switches
    private TouchSensor scissorLimitLo = null;

    //Source code from Arm Controller
    ArmController armController = new ArmController();
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
        shoulderDrive = hardwareMap.get(DcMotor.class, "shoulderDrive");
        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowDrive");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        // digital limit switches
        scissorLimitLo = hardwareMap.get(TouchSensor.class, "scissorLoSensor");

        // assign wheel motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // assign scissor, extension, and claw directions
        scissorDrive.setDirection(DcMotor.Direction.FORWARD);

        // outside the while loop, set initial claw servo position
        double clawServoPosition = CLAW_MIN_POS; // Start at half position

        // outside the while loop, set homing mode false
        double driveDirectionFactor = 1.0;

        boolean lockSet;
        lockSet = true;
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
            // SECTION 1: toggle backwards mode to use the pusher
            // *******************************************************************************************

            if (gamepad1.back||gamepad2.back) {
                if (lockSet = true) {
                    driveDirectionFactor = -1.0;
                    lockSet = false;
                } else if (lockSet = false) {
                    driveDirectionFactor = 1.0;
                    lockSet = true;
                }
            }
            // *******************************************************************************************
            // SECTION 2: Main Joystick Robot Driving
            // *******************************************************************************************

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
                axial = -gamepad1.left_stick_y * turboOverrideLeftStick * driveDirectionFactor;  // Note: pushing stick forward gives negative value
            }
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                lateral = gamepad1.left_stick_x * turboOverrideLeftStick * driveDirectionFactor;
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                yaw = gamepad1.right_stick_x * turboOverrideRightStick; // * driveDirectionFactor;
            }

            // gamepad 2
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                axial = -gamepad2.left_stick_y * turboOverrideLeftStick * driveDirectionFactor;  // Note: pushing stick forward gives negative value
            }
            if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                lateral = gamepad2.left_stick_x * turboOverrideLeftStick * driveDirectionFactor;
            }
            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                yaw = gamepad2.right_stick_x * turboOverrideRightStick; // * driveDirectionFactor;
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

            // *******************************************************************************************
            // SECTION 3: Raise and Lower Scissor Lift
            // *******************************************************************************************

            // read encoder feedback position and limit switch status
            double scissorEncoderCounts = scissorDrive.getCurrentPosition();
            boolean scissorLimitLoOn = scissorLimitLo.isPressed();
            boolean scissorLimitHiOn = (scissorEncoderCounts >= 10100);


            // TODO: adjust max speeds between 0.0 and 1.0
            double scissorUpOverride = 1.0;
            double scissorDownOverride = 0.75;
            double scissorDrivePower = 0.0;

            // Gamepad 1 triggers
            if (gamepad1.right_trigger > 0.01 && scissorLimitHiOn == false) {
                scissorDrivePower = 1.0;
            } else if (gamepad1.left_trigger > 0.01 && scissorLimitLoOn == false) {
                scissorDrivePower = -0.75;
            }

            //Gamepad 2 triggers
            if (gamepad2.right_trigger > 0.01 && scissorLimitHiOn == false) {
                scissorDrivePower = scissorDrivePower * scissorUpOverride;
            } else if (gamepad2.left_trigger > 0.01 && scissorLimitLoOn == false) {
                scissorDrivePower = -scissorDrivePower * scissorDownOverride;
            }

            // *******************************************************************************************
            // SECTION 4: Open and Close Claw
            // *******************************************************************************************
            if (gamepad1.x || gamepad2.x == true) {
                clawServo.setPosition(0.30);
            } else if (gamepad1.x || gamepad2.x == true) {
                clawServo.setPosition(0.05);
            }

            // *******************************************************************************************
            // SECTION 5: Color Sensor
            // *******************************************************************************************
            int blue = colorSensor.blue();
            int red = colorSensor.red();
            int green = colorSensor.green();

            double color = 0;

            if (blue > red && blue > green && blue > 25) {
                color = 1;
            }
            if (red > green && red > blue && red > 40) {
                color = 2;
            }
            if (red > 60 && green > 50 && blue < 40) {
                color = 4;
            }

            // *******************************************************************************************
            // SECTION 6: Elbow and Shoulder Motors
            // *******************************************************************************************

            //Refer to Sourced Code Near End of Defines

            // *******************************************************************************************
            // SECTION 7: Write Outputs
            // *******************************************************************************************

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
            leftBackDrive.setPower(leftBackPower);

            // Power to scissor lift, extension
            scissorDrive.setPower(scissorDrivePower);

            clawServo.setPosition(clawServoPosition);

            // *******************************************************************************************
            // SECTION 7: Telemetry
            // *******************************************************************************************

            if (color == 1) {
                telemetry.addData("Blue", true);
            }
            if (color == 2) {
                telemetry.addData("Red", true);
            }
            if (color == 3) {
                telemetry.addData("Green", true);
            }
            if (color == 4) {
                telemetry.addData("Yellow", true);
            }

            telemetry.update();
            // Update telemetry
            telemetry.update();

            // idle time for servo
            sleep(CYCLE_MS);
            idle();

        }
    }

    private String MotorPower(double leftFrontPower) {

        // \u25A0: Square
        // \u25B2: Up arrow
        // \u25BC: Down arrow
        // \u25B6: Right triangle
        // \u25C0: Left triangle

        String power = String.format("%4.2f", Math.abs(leftFrontPower));
        String direction = "";
        if (leftFrontPower > 0.01) {
            direction = "\u25B2"; // Up arrow
        } else if (leftFrontPower < -0.01) {
            direction = "\u25BC"; // Down arrow
        } else {
            direction = "\u25A0 "; // No movement
        }
        return direction + power;
    }
}

