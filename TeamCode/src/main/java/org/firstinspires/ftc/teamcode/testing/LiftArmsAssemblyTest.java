

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "LiftArmsAsseblyTest", group = "Testing")
public class LiftArmsAssemblyTest extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();


    // scissor lift drive, shoulder, elbow and claw
    private Servo elbowServo = null;
    private DcMotor shoulderDrive = null;

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
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");

        // assign scissor, extension, and claw directions
        shoulderMotor.setDirection(DcMotor.Direction.FORWARD);

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        double elbowPosition = 0.02;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // put logic here
            int shoulderPosition = shoulderDrive.getCurrentPosition();
            shoulderDrive.setPower(gamepad1.left_stick_x * 0.10);



            elbowTriggerUp.update(gamepad1.x);
            elbowTriggerDown.update(gamepad1.b);
            if (elbowTriggerUp.wasTriggered()) {
                if (elbowPosition > 0 && elbowPosition < 1) {
                    elbowPosition = elbowPosition + 0.01;
                }
            }
            elbowServo.setPosition(elbowPosition);


            if (elbowTriggerDown.wasTriggered()) {
                if (elbowPosition > 0 && elbowPosition < 1) {
                    elbowPosition = elbowPosition - 0.01;
                }
            }
            elbowServo.setPosition(elbowPosition);

            // update telemetry data
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Shoulder", shoulderPosition);
            telemetry.addData("Elbow", elbowPosition);
            telemetry.update();

        }

    }
}
