package org.firstinspires.ftc.teamcode.testing.claw;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;

@TeleOp(name = "CLAW | Servo Gripper Test", group = "$$$$ Claw")
public class ClawServoGripperTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------

    // claw
    private Servo clawServo = null;

    RisingEdgeTrigger leftBumperTrigger = new RisingEdgeTrigger();
    RisingEdgeTrigger rightBumperTrigger = new RisingEdgeTrigger();


    // halfway point of claw
    double servoPosition = 0.50;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------
        clawServo = hardwareMap.get(Servo.class, "clawServo");

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

            leftBumperTrigger.update(gamepad1.left_bumper);
            rightBumperTrigger.update(gamepad1.right_bumper);

            // left bumper opens
            if (leftBumperTrigger.wasTriggered()) {
                servoPosition += 0.05;
            }

            // right bumper closes
            if (rightBumperTrigger.wasTriggered()) {
                servoPosition -= 0.05;
            }

            // limit servo position
            if (servoPosition < 0.0)
            {
                servoPosition = 0.0;
            }
            else if (servoPosition > 1.0)
            {
                servoPosition = 1.0;
            }

            clawServo.setPosition(servoPosition);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Press left / right bumpers to test the claw gripper.");
            telemetry.addLine("It starts at 0.50 and increments by 0.05 with each press.");
            telemetry.addData("Servo Target", servoPosition);
            telemetry.addData("Servo Actual", clawServo.getPosition());
            telemetry.update();

        }
    }
}
