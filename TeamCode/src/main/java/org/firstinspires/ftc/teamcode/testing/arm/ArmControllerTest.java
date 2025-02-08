package org.firstinspires.ftc.teamcode.testing.arm;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ctrl.ArmController;

@TeleOp(name = "TEST | Arm | Controller Test", group = "$$$$ Arm")
public class ArmControllerTest extends LinearOpMode {

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
            telemetry.addData("Arm State", armController.armControlState);
            telemetry.addData("--- Motion Run Time", armController.profileRunTime);
            telemetry.addData("Profiles", armController.shoulderProfile.length);
            telemetry.addLine(String.format("Home[%s] Drop[%s] Pick[%s]", armController.shoulderState.inHomeWindow, armController.shoulderState.inDropWindow, armController.shoulderState.inPickWindow));
            telemetry.addData("------- FFW Power", armController.shoulderState.shoulderPowerFfw);
            telemetry.addLine(String.format("        P [%4.2f] V [%4.2f] A [%4.2f]", armController.shoulderState.shoulderPosTargetRad, armController.shoulderState.shoulderVelTargetRad, armController.shoulderState.shoulderAccTargetRad));
            telemetry.addData("------- POS Power", armController.shoulderState.shoulderPowerPidPos);
            telemetry.addLine(String.format("        A [%d] T [%d] E [%d]", armController.shoulderState.currentPosActual, armController.shoulderState.currentPosTarget, armController.shoulderState.currentPosError));
            telemetry.addData("------- VEL Power", armController.shoulderState.shoulderPowerPidVel);
            telemetry.addLine(String.format("        A [%d] T [%d] E [%d]", armController.shoulderState.currentVelActual, armController.shoulderState.currentVelTarget, armController.shoulderState.currentVelError));
            telemetry.addData("------- PID Power", armController.shoulderState.shoulderPower);
            telemetry.update();


        }

    }

}

