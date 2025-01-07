package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "LiftArms Tuning Shoulder", group = "Tuning")
public class LiftArmsTuningShoulder extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // scissor lift drive, shoulder, elbow and claw
    private DcMotorEx shoulderDrive = null;

    // rising edge triggers to jog the motor
    private RisingEdgeTrigger shoulderTrigForward = new RisingEdgeTrigger();
    private RisingEdgeTrigger shoulderTrigBackward = new RisingEdgeTrigger();

    private RisingEdgeTrigger coefficientTrigP = new RisingEdgeTrigger();
    private RisingEdgeTrigger coefficientTrigI = new RisingEdgeTrigger();
    private RisingEdgeTrigger coefficientTrigD = new RisingEdgeTrigger();
    private RisingEdgeTrigger coefficientTrigF = new RisingEdgeTrigger();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        int POSITION_INCREMENT = 100;

        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        shoulderDrive.setDirection(DcMotorEx.Direction.FORWARD);

        // set initial PIDF coefficients
        PIDFCoefficients actualCoefficients = shoulderDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients targetCoefficients = new PIDFCoefficients(actualCoefficients);

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // set initial position to current position
        int shoulderTarget = shoulderDrive.getCurrentPosition();
        shoulderDrive.setTargetPosition(shoulderTarget);
        shoulderDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulderDrive.setPower(1.0);

        // get tuning parameters
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // read inputs
            int shoulderPosition = shoulderDrive.getCurrentPosition();
            double shoulderCurrent = shoulderDrive.getCurrent(CurrentUnit.AMPS);
            actualCoefficients = shoulderDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            // shoulder servo section
            shoulderTrigForward.update(gamepad1.dpad_up);
            shoulderTrigBackward.update(gamepad1.dpad_down);
            if (shoulderTrigForward.wasTriggered()) {
                shoulderTarget = shoulderTarget + POSITION_INCREMENT;
            }
            if (shoulderTrigBackward.wasTriggered()) {
                shoulderTarget = shoulderTarget - POSITION_INCREMENT;
            }

            // write outputs
            shoulderDrive.setTargetPosition(shoulderTarget);

            // *********************************************************************
            // adjust PIDF coefficients
            // *********************************************************************

            // position
            coefficientTrigP.update(gamepad1.x);
            if (coefficientTrigP.wasTriggered() && gamepad1.right_bumper) {
                targetCoefficients.p += 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }
            if (coefficientTrigP.wasTriggered() && gamepad1.left_bumper) {
                targetCoefficients.p -= 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }

            // integral
            coefficientTrigI.update(gamepad1.y);
            if (coefficientTrigI.wasTriggered() && gamepad1.right_bumper) {
                targetCoefficients.i += 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }
            if (coefficientTrigI.wasTriggered() && gamepad1.left_bumper) {
                targetCoefficients.i -= 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }

            // derivative
            coefficientTrigD.update(gamepad1.b);
            if (coefficientTrigD.wasTriggered() && gamepad1.right_bumper) {
                targetCoefficients.d += 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }
            if (coefficientTrigD.wasTriggered() && gamepad1.left_bumper) {
                targetCoefficients.d -= 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }

            // feed forward
            coefficientTrigF.update(gamepad1.a);
            if (coefficientTrigF.wasTriggered() && gamepad1.right_bumper) {
                targetCoefficients.f += 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }
            if (coefficientTrigF.wasTriggered() && gamepad1.left_bumper) {
                targetCoefficients.f -= 0.1;
                shoulderDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, targetCoefficients);
            }

            // *********************************************************************
            // update telemetry data
            // *********************************************************************

            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine(String.format("Target / Actual: [%d] / [%d]", shoulderTarget, shoulderPosition));
            telemetry.addLine(String.format("Current (Amps): %d", shoulderCurrent));
            telemetry.addLine(String.format("Coefficients (PID) [%4.2f] [%4.2f] [%4.2f]", actualCoefficients.p, actualCoefficients.i, actualCoefficients.d));
            telemetry.addLine(String.format("Feed Forward: [%4.2f]", actualCoefficients.f));
            telemetry.update();

        }

    }
}
