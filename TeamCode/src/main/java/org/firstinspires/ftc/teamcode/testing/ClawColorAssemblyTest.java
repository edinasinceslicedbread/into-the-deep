package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "ClawColorTest", group = "Testing")
public class ClawColorAssemblyTest extends LinearOpMode {

    static final int CYCLE_MS = 50;             // period of each cycle

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // main wheel drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // scissor lift drive, shoulder, elbow and claw
    private DcMotor scissorDrive = null;
    private Servo shoulderServo = null;
    private Servo elbowServo = null;
    private Servo clawServo = null;

    // digital limit switches
    private TouchSensor scissorLimitLo = null;

    // rising edge triggers example
    private RisingEdgeTrigger buttonTriggerA = new RisingEdgeTrigger();

    //color sensors
    private ColorRangeSensor colorSensor;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        // main wheel drive motor hardware names
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // scissor drive, claw server, color sensor, and extend / retract
        scissorDrive = hardwareMap.get(DcMotor.class, "scissorDrive");
        shoulderServo = hardwareMap.get(Servo.class, "shoulderServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorRangeSensor.class,"clawColorSensor");

        // assign wheel motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // assign scissor, extension, and claw directions
        scissorDrive.setDirection(DcMotor.Direction.FORWARD);
        shoulderServo.setDirection(Servo.Direction.FORWARD);
        elbowServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // put logic here
            int blue = colorSensor.blue();
            int red = colorSensor.red();
            int green = colorSensor.green();

            double color = 0;
            // blue is 1, red is 2, blue is 3
            // All number data for the if statement can be replaced with a different value for a set value depending on distance from block.
            if (blue > red && blue > green && blue > 25)
            {
                color = 1;
            }
            if (red > green && red > blue && red > 40)
            {
                color = 2;
            }
            if (red > 60 && green > 50 && blue < 40)
            {
                color = 4;
            }

            if (gamepad1.a == true)
            {
                clawServo.setPosition(0.30);
            }
            else if (gamepad1.y == true)
            {
                clawServo.setPosition(0.05);
            }

            // update telemetry data
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine(String.format("red=%d, green=%d, blue=%d",red,green,blue));
            telemetry.addData("Color", color);

            if (color == 1)
            {
                telemetry.addData("Blue", true);
            }
            if (color == 2)
            {
                telemetry.addData("Red", true);
            }
            if (color == 3)
            {
                telemetry.addData("Green", true);
            }
            if (color == 4)
            {
                telemetry.addData("Yellow", true);
            }
            telemetry.update();

            // idle time for servo
            sleep(CYCLE_MS);
            idle();

        }
    }
}
