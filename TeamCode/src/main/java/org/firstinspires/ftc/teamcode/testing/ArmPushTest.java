package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;

@TeleOp(name = "TEST | Arm | Push Test", group = "$$$$")
public class ArmPushTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------
    private DcMotorEx shoulderDrive = null;
    private DcMotorEx elbowDrive = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        // hardware map
        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");

        // motor directions
        shoulderDrive.setDirection(DcMotorEx.Direction.REVERSE);
        elbowDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // motor modes and braking
        shoulderDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elbowDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulderDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbowDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // other variables
        double shoulderPower = 0;
        double elbowPower = 0;
        RisingEdgeTrigger leftBumper = new RisingEdgeTrigger();     // gamepad left bumper
        RisingEdgeTrigger rightBumper = new RisingEdgeTrigger();    // gamepad right bumper


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
            // Run triggers every cycle
            //------------------------------------------------------------------------------------------------
            leftBumper.update(gamepad1.left_bumper);
            rightBumper.update(gamepad1.right_bumper);

            //------------------------------------------------------------------------------------------------
            // Lift Arm Control
            //------------------------------------------------------------------------------------------------
            int shoulderTicksActual = shoulderDrive.getCurrentPosition();
            int elbowTicksActual = elbowDrive.getCurrentPosition();

            if (rightBumper.wasTriggered()) {
                shoulderPower += 0.01;
                if (shoulderPower > 1.0){
                    shoulderPower = 1.0;
                }
            }
            if (leftBumper.wasTriggered()) {
                shoulderPower -= 0.01;
                if (shoulderPower < -1.0){
                    shoulderPower = -1.0;
                }
            }

            // use bumpers to set holding power
            elbowPower = 0;
            if (gamepad1.right_trigger > 0.01) {
                elbowPower = gamepad1.right_trigger * 0.5;
            }
            if (gamepad1.left_trigger > 0.01) {
                elbowPower = -gamepad1.left_trigger * 0.5;
            }

            // reset power with any main button
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                elbowPower = 0;
                shoulderPower = 0;
            }

            // set shoulder power
            shoulderDrive.setPower(shoulderPower);
            elbowDrive.setPower(elbowPower);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Move the elbow and shoulder by hand.");
            telemetry.addLine("Encoder counters should be counting upwards.");
            telemetry.addLine("Test the elbow motor by gently using the triggers.");
            telemetry.addLine("Test the shoulder by incrementing power with the buttons.");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Shoulder: PWR[%4.2f] TICKS[%d]", shoulderPower, shoulderTicksActual));
            telemetry.addLine(String.format("Elbow: PWR[%4.2f] TICKS[%d]", elbowPower, elbowTicksActual));
            telemetry.update();

        }

    }

}

