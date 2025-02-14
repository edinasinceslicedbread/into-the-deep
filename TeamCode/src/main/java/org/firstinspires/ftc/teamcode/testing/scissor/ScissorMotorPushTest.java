package org.firstinspires.ftc.teamcode.testing.scissor;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SCISSOR | Motor Push Test", group = "$$$$ Scissor")
public class ScissorMotorPushTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------

    // scissor
    private DcMotorEx scissorDrive = null;
    private TouchSensor scissorLoSensor = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Definitions
        //------------------------------------------------------------------------------------------------

        // scissor
        scissorDrive = hardwareMap.get(DcMotorEx.class, "scissorDrive");
        scissorLoSensor = hardwareMap.get(TouchSensor.class, "scissorLoSensor");

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


            //------------------------------------------------------------------------------------------------
            // Scissor Control
            //------------------------------------------------------------------------------------------------

            // read position and limit switch
            int scissorTicksActual = scissorDrive.getCurrentPosition();
            boolean scissorLoPressed = scissorLoSensor.isPressed();

            // no power without triggers
            double scissorPower = 0;

            // right trigger moves scissor up to limited value
            if (gamepad1.right_trigger > 0.05) {
                scissorPower = gamepad1.right_trigger * 0.5;
            }

            // left trigger moves scissor down to limited value
            if (gamepad1.left_trigger > 0.05) {
                scissorPower = -gamepad1.left_trigger * 0.2;
            }

            // lower limit switch override
            if (scissorPower < 0.0 && scissorLoPressed) {
                scissorPower = 0;
            }

            // output the power
            scissorDrive.setPower(scissorPower);


            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Test the scissor by gently using with the triggers.");
            telemetry.addLine("Right trigger moves the scissor up.");
            telemetry.addLine("Left trigger moves the scissor down.");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Scissor: PWR[%4.2f] TICKS[%d]", scissorPower, scissorTicksActual));
            telemetry.addLine(String.format("Lower Limit Switch: [%s]", scissorLoPressed));
            telemetry.update();

        }

    }
}

