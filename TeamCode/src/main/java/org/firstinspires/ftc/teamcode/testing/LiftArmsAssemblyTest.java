import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "Test / LiftArmsAssemblyTest", group = "Testing")
public class LiftArmsAssemblyTest extends LinearOpMode {

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

        // get hardware map
        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "clawColorSensor");

        // assign directions
        shoulderDrive.setDirection(DcMotorEx.Direction.REVERSE);
        elbowDrive.setDirection(DcMotorEx.Direction.REVERSE);
        
        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // **********************************************************
            // arm control
            // **********************************************************

            // get encoder positions
            int shoulderPosition = shoulderDrive.getCurrentPosition();
            int elbowPosition = elbowDrive.getCurrentPosition();

            if (gamepad1.left_trigger > 0.05)
            {
                shoulderDrive.setPower(-gamepad1.left_trigger * 0.50);
            }
            if (gamepad1.right_trigger > 0.05)
            {
                shoulderDrive.setPower(gamepad1.right_trigger * 0.50);
            }
            
            
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
            telemetry.addData("Shoulder Encoder", shoulderPosition);
            telemetry.addData("Elbow Encoder", elbowPosition);
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
