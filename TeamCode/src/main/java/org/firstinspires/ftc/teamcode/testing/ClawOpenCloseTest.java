package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TEST | Claw Open / Close Test", group = "$$$$")
public class ClawOpenCloseTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------
    private Servo clawServo = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.FORWARD);

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


            if (gamepad1.right_bumper || gamepad2.right_bumper)
            {
                clawServo.setPosition(0.30);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper)
            {
                clawServo.setPosition(0.05);
            }

            // update telemetry data
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Press left / right bumpers to test the claw gripper.");
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.update();

            // idle time for servo
            idle();

        }
    }
}
