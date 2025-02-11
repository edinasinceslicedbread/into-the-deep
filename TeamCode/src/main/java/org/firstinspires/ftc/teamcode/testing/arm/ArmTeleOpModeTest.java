package org.firstinspires.ftc.teamcode.testing.arm;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ctrl.ArmController;

@TeleOp(name = "ARM | 04 TeleOpMode Test", group = "$$$$ Arm")
public class ArmTeleOpModeTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Controllers
    //------------------------------------------------------------------------------------------------
    ArmController armController = new ArmController();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Initialize
        //------------------------------------------------------------------------------------------------
        armController.initialize(hardwareMap);

        //------------------------------------------------------------------------------------------------
        // Start Button
        //------------------------------------------------------------------------------------------------

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // reset controllers
        runtime.reset();
        armController.reset();

        //------------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------------
            // Arm Control
            //------------------------------------------------------------------------------------------------
            armController.update(runtime, gamepad1, gamepad2);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the armController.shoulder.");
            telemetry.addLine(String.format("X [%s] -> Home Position", armController.shoulderState.atHomePos));
            telemetry.addLine(String.format("Y [%s] [%s]-> Basket or Chamber", armController.shoulderState.atBasketPos, armController.shoulderState.atDrivePos));
            telemetry.addLine(String.format("A [%s] -> Stop Motion", armController.shoulderState.currentVelActual > 0.0));
            telemetry.addLine(String.format("B [%s] -> Pick at Chamber Submersible", armController.shoulderState.atPickPos));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Power: FFW[%4.2f] PID[%4.2f]", armController.shoulderState.motorPowerFFW, armController.shoulderState.motorPowerPID));
            telemetry.addLine(String.format("Shoulder Target: GOAL[%d] SP[%d]", (int) Math.round(armController.shoulderController.getGoal().position), (int) Math.round(armController.shoulderController.getSetpoint().position)));
            telemetry.addLine(String.format("Shoulder Status: PWR[%4.2f] TICKS[%d]", armController.shoulderState.motorPower, armController.shoulderState.currentPosActual));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Shoulder Error: POS[%4.2f] VEL[%4.2f]", armController.shoulderController.getPositionError(), armController.shoulderController.getVelocityError()));
            telemetry.addLine(String.format("At: Goal[%s] SetPoint[%s]", armController.shoulderController.atGoal(), armController.shoulderController.atSetpoint()));
            telemetry.update();


        }

    }

}

