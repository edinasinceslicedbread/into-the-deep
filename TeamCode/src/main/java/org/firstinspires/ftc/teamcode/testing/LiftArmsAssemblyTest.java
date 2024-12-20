

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "LiftArmsAsseblyTest", group = "Testing")
public class LiftArmsAssemblyTest extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    // scissor lift drive, shoulder, elbow and claw
    private DcMotor scissorDrive = null;
    private Servo shoulderServo = null;
    private Servo elbowServo = null;

    // digital limit switches
    private TouchSensor scissorLimitLo = null;

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
        scissorDrive = hardwareMap.get(DcMotor.class, "scissorDrive");
        shoulderServo = hardwareMap.get(Servo.class, "shoulderServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");

        // assign scissor, extension, and claw directions
        scissorDrive.setDirection(DcMotor.Direction.FORWARD);
        shoulderServo.setDirection(Servo.Direction.REVERSE);
        elbowServo.setDirection(Servo.Direction.FORWARD);

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        double shoulderPosition = 0;
        double elbowPosition = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // put logic here
            shoulderTriggerUp.update(gamepad1.y);
            shoulderTriggerDown.update(gamepad1.a);
            if (shoulderTriggerUp.wasTriggered()) {
                shoulderPosition = shoulderPosition + 0.05;
            }
            shoulderServo.setPosition(shoulderPosition);


            if (shoulderTriggerDown.wasTriggered()) {
                shoulderPosition = shoulderPosition - 0.05;
            }
            shoulderServo.setPosition(shoulderPosition);

            elbowTriggerUp.update(gamepad1.x);
            elbowTriggerDown.update(gamepad1.b);
            if (elbowTriggerUp.wasTriggered()) {
                elbowPosition = elbowPosition + 0.05;
            }
            elbowServo.setPosition(elbowPosition);


            if (elbowTriggerDown.wasTriggered()) {
                elbowPosition = elbowPosition - 0.05;
            }
            elbowServo.setPosition(elbowPosition);

            // update telemetry data
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Shoulder", shoulderPosition);
            telemetry.update();

        }

    }
}
