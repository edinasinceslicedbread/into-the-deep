package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "LiftArmsAsseblyTest B", group = "Testing")
public class LiftArmsAssemblyTestB extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    // scissor lift drive, shoulder, elbow and claw
    private DcMotorEx shoulderDrive = null;
    private DcMotorEx elbowDrive = null;
    private Servo clawServo = null;
    private ColorRangeSensor colorSensor;

    // rising edge triggers example
    private RisingEdgeTrigger shoulderTriggerUp = new RisingEdgeTrigger();
    private RisingEdgeTrigger shoulderTriggerDown = new RisingEdgeTrigger();
    private RisingEdgeTrigger elbowTriggerUp = new RisingEdgeTrigger();
    private RisingEdgeTrigger elbowTriggerDown = new RisingEdgeTrigger();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        // scissor drive, claw server, and extend / retract
        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "clawColorSensor");

        // assign scissor, extension, and claw directions
        shoulderDrive.setDirection(DcMotor.Direction.REVERSE);
        elbowDrive.setDirection(DcMotor.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);

        int shoulderTarget = shoulderDrive.getCurrentPosition();
        shoulderDrive.setTargetPosition(shoulderTarget);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int elbowTarget = elbowDrive.getCurrentPosition();
        elbowDrive.setTargetPosition(elbowTarget);
        elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double shoulderPosition = shoulderDrive.getCurrentPosition();
            double elbowPosition = elbowDrive.getCurrentPosition();

            shoulderTriggerUp.update(gamepad1.b);
            shoulderTriggerDown.update(gamepad1.x);
            if (shoulderTriggerUp.wasTriggered())
            {
                shoulderTarget += 50;
            }
            if (shoulderTriggerDown.wasTriggered())
            {
                shoulderTarget -= 50;
            }

            shoulderDrive.setTargetPosition(shoulderTarget);
            shoulderDrive.setPower(0.75);
            // shoulderDrive.setPower(gamepad1.right_stick_x * 0.75);

            elbowTriggerUp.update(gamepad1.y);
            elbowTriggerDown.update(gamepad1.a);
            if (elbowTriggerUp.wasTriggered())
            {
                elbowTarget += 3;
            }
            if (elbowTriggerDown.wasTriggered())
            {
                elbowTarget -= 3;
            }

            elbowDrive.setTargetPosition(elbowTarget);
            elbowDrive.setPower(1.0);
            // elbowDrive.setPower(gamepad1.left_stick_x * 0.75);

            // ****************************************************************************
            // color detection
            // ****************************************************************************

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


            // ****************************************************************************
            // update telemetry data
            // ****************************************************************************

            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Shoulder Pos", shoulderPosition);
            telemetry.addData("Shoulder Tgt", shoulderTarget);
            telemetry.addData("Elbow Pos", elbowPosition);
            telemetry.addData("Elbow Tgt", elbowTarget);
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
