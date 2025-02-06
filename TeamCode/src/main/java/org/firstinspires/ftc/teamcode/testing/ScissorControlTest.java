package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "TEST | Scissor | Control Test", group = "$$$$")
public class ScissorControlTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Parameters
    //------------------------------------------------------------------------------------------------
    public static class ShoulderParams {
        public double holdingPower = 0.2;
        public double ticksPerDegree = 1440.0 / 360.0;

        // control parameters
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        // feed forward factor
        public double kF = 0.0;

    }

    public static class ElbowParams {
        public double ticksPerDegree = 288.0 / 360.0;

        // control parameters
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        // feed forward factor
        public double kF = 0.0;

    }

    public static ShoulderParams SHOULDER_PARAMS = new ShoulderParams();
    public static ElbowParams ELBOW_PARAMS = new ElbowParams();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------

    // hardware devices
    private DcMotorEx shoulderDrive = null;
    private DcMotorEx elbowDrive = null;
    private DcMotorEx scissorDrive = null;
    private TouchSensor scissorLoSensor = null;
    private Servo clawServo = null;
    private ColorRangeSensor colorSensor;

    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        // lift arm
        int shoulderTicksTarget = 0, elbowTicksTarget = 0, liftArmState = 0;
        double shoulderVelocityTarget = 0;
        double shoulderStartSeconds = 0.0, elbowStartSeconds = 0.0;
        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");

        // scissor lift
        scissorDrive = hardwareMap.get(DcMotorEx.class, "scissorDrive");
        scissorLoSensor = hardwareMap.get(TouchSensor.class, "scissorLoSensor");

        // claw and color sensor
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        //------------------------------------------------------------------------------------------------
        // Hardware Config
        //------------------------------------------------------------------------------------------------
        shoulderDrive.setDirection(DcMotorEx.Direction.REVERSE);
        shoulderDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulderDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        PIDController shoulderController = new PIDController(SHOULDER_PARAMS.kP, SHOULDER_PARAMS.kI, SHOULDER_PARAMS.kD);

        elbowDrive.setDirection(DcMotorEx.Direction.REVERSE);
        elbowDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elbowDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        PIDController elbowController = new PIDController(ELBOW_PARAMS.kP, ELBOW_PARAMS.kI, ELBOW_PARAMS.kD);

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


            // **********************************************************
            // scissor control
            // **********************************************************

            // read position and limit switch
            int scissorEncoderCounts = scissorDrive.getCurrentPosition();
            boolean scissorLoPressed = scissorLoSensor.isPressed();

            // move scissor by pressing buttons
            double scissorPower = 0;
            if (gamepad1.right_trigger > 0.05) {
                scissorPower = gamepad1.right_trigger;
            }
            if (gamepad1.left_trigger > 0.05) {
                scissorPower = -gamepad1.left_trigger;
            }

            // enforce scissor max limits
            if (scissorPower < -0.1) {
                scissorPower = -0.1;
            }
            if (scissorPower > 0.5) {
                scissorPower = 0.5;
            }

            // upper limit
            if (scissorEncoderCounts >= 10000) {
                scissorPower = 0;
            }

            // lower limit
            if (scissorPower <= 0 && scissorLoPressed) {
                scissorPower = 0;
            }

            scissorDrive.setPower(scissorPower);

            // **********************************************************
            // color sensor
            // **********************************************************
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

            if (gamepad1.a == true) {
                clawServo.setPosition(0.30);
            } else if (gamepad1.y == true) {
                clawServo.setPosition(0.05);
            }


            // **********************************************************
            // update telemetry data
            // **********************************************************
            telemetry.addData("Run Time", runtime.toString());

            telemetry.addLine(String.format("red=%d, green=%d, blue=%d", red, green, blue));
            telemetry.addData("Color", color);
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

        }

    }
}

