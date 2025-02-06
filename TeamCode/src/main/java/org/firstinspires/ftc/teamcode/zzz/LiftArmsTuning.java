package org.firstinspires.ftc.teamcode.zzz;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "Tuning / Robot Arm", group = "002 Tuning")
@Config
@Disabled
public class LiftArmsTuning extends LinearOpMode {

    public static class ShoulderParams {
        public double ticksPerDegree = 1440.0 / 360.0;

        // control parameters
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        // feed forward factor
        public double kF = 0.0;

    }

    public static class ElbowParams {
        public double ticksPerDegree = 288.0 / 360.0;

        // control parameters
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        // feed forward factor
        public double kF = 0.0;

    }

    public static LiftArmsTuning.ShoulderParams SHOULDER_PARAMS = new LiftArmsTuning.ShoulderParams();
    public static LiftArmsTuning.ElbowParams ELBOW_PARAMS = new LiftArmsTuning.ElbowParams();

    private ElapsedTime runtime = new ElapsedTime();

    // drive hardware
    private DcMotorEx shoulderDrive = null;
    private DcMotorEx elbowDrive = null;

    // controller
    private PIDController shoulderController;
    private PIDController elbowController;

    @Override
    public void runOpMode() {

        // get drive hardware
        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");

        // set drive directions
        shoulderDrive.setDirection(DcMotor.Direction.REVERSE);
        elbowDrive.setDirection(DcMotor.Direction.REVERSE);

        // initialize controllers
        shoulderController = new PIDController(SHOULDER_PARAMS.kP, SHOULDER_PARAMS.kI, SHOULDER_PARAMS.kD);
        elbowController = new PIDController(ELBOW_PARAMS.kP, ELBOW_PARAMS.kI, ELBOW_PARAMS.kD);

        // update some telemetry and wait for start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // read shoulder inputs
            int shoulderTicksActual = shoulderDrive.getCurrentPosition();
            double shoulderAmps = shoulderDrive.getCurrent(CurrentUnit.AMPS);

            // read elbow inputs
            int elbowTicksActual = elbowDrive.getCurrentPosition();
            double elbowAmps = elbowDrive.getCurrent(CurrentUnit.AMPS);

            // shoulder control
            shoulderController.setPID(SHOULDER_PARAMS.kP, SHOULDER_PARAMS.kI, SHOULDER_PARAMS.kD);
            int shoulderTicksTarget = 0;
            double shoulderPID = shoulderController.calculate(elbowTicksActual, shoulderTicksTarget);

            double shoulderFeedForward = Math.cos(Math.toRadians(shoulderTicksActual / SHOULDER_PARAMS.ticksPerDegree));
            double shoulderPower = shoulderPID + shoulderFeedForward;

            // elbow control
            elbowController.setPID(ELBOW_PARAMS.kP, ELBOW_PARAMS.kI, ELBOW_PARAMS.kD);
            int elbowTicksTarget = 0; // shoulderTicksTarget * 0.1;
            double elbowPID = elbowController.calculate(elbowTicksActual, elbowTicksTarget);
            double elbowFeedForward = 0.0;
            double elbowPower = elbowPID+ elbowFeedForward;

            // write outputs
            shoulderDrive.setPower(shoulderPower);
            elbowDrive.setPower(elbowPower);

            // *********************************************************************
            // update telemetry data
            // *********************************************************************

            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("SHOULDER");
            telemetry.addLine(String.format("Target / Actual: [%d] / [%d]", shoulderTicksTarget, shoulderTicksActual));
            telemetry.addLine(String.format("Power (Amps): %4.2f", shoulderPower));
            telemetry.addLine(String.format("Current (Amps): %4.2f", shoulderAmps));
            telemetry.addLine("ELBOW");
            telemetry.addLine(String.format("Target / Actual: [%d] / [%d]", elbowTicksTarget, elbowTicksActual));
            telemetry.addLine(String.format("Power (Amps): %4.2f", elbowPower));
            telemetry.addLine(String.format("Current (Amps): %4.2f", elbowAmps));
            telemetry.update();

        }

    }
}
