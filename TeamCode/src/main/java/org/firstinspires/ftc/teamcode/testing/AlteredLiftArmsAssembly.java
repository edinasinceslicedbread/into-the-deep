package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "LiftArmsAssemblyTest", group = "Testing")
public class AlteredLiftArmsAssembly extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    // scissor lift drive, shoulder, elbow and claw
    private Servo elbowServo = null;
    private DcMotor shoulderDrive = null;
    private Servo clawServo = null;
    private ColorRangeSensor colorSensor;

    // rising edge triggers example
    private RisingEdgeTrigger elbowTriggerUp = new RisingEdgeTrigger();
    private RisingEdgeTrigger elbowTriggerDown = new RisingEdgeTrigger();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        // scissor drive, claw server, and extend / retract
        shoulderDrive = hardwareMap.get(DcMotor.class, "shoulderDrive");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "clawColorSensor");

        // assign scissor, extension, and claw directions
        shoulderDrive.setDirection(DcMotor.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        double elbowPosition = 0.02;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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

            // put logic here
            int shoulderPosition = shoulderDrive.getCurrentPosition();
            if (shoulderPosition > 0) {
                shoulderDrive.setPower(gamepad1.right_trigger * 0.50);
            }
            if (shoulderPosition > 720) {
                shoulderDrive.setPower(gamepad1.right_trigger * -0.50);
            }

            // elbow servo section
            elbowTriggerUp.update(gamepad1.right_trigger > 0.1);
            elbowTriggerDown.update(gamepad1.left_trigger > 0.1);
            if (elbowTriggerUp.wasTriggered()) {
                elbowPosition = elbowPosition + 0.025;
                if (elbowPosition > 1) {
                    elbowPosition = 1.0;
                }
            }
            if (elbowTriggerDown.wasTriggered()) {
                elbowPosition = elbowPosition - 0.025;
                if (elbowPosition < 0) {
                    elbowPosition = 0;
                }
            }
            elbowServo.setPosition(elbowPosition);

            // update telemetry data
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Shoulder", shoulderPosition);
            telemetry.addData("Elbow", elbowPosition);
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
