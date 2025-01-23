package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "Tuning / Robot Arm", group = "002")
@Config
public class LiftArmsTuning extends LinearOpMode {

    public static class ShoulderParams {
        public double degreesPerTick = 1440.0 / 360.0;

        // control parameters
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        // feed forward factor
        public double kF = 0.0;

    }

    public static class ElbowParams {
        public double degreesPerTick = 288.0 / 360.0;

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
            double shoulderTicksTarget = 0.0;
            double shoulderPID = shoulderController.calculate(elbowTicksActual, shoulderTicksTarget);

            // elbow control
            elbowController.setPID(ELBOW_PARAMS.kP, ELBOW_PARAMS.kI, ELBOW_PARAMS.kD);
            double elbowTicksTarget = shoulderTicksTarget * 0.1;
            double elbowPID = elbowController.calculate(elbowTicksActual, elbowTicksTarget);

            double shoulderPower = 0.0;
            double elbowPower = 0.0;

            // write outputs
            shoulderDrive.setPower(shoulderPower);
            elbowDrive.setPower(elbowPower);

            // *********************************************************************
            // update telemetry data
            // *********************************************************************

            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("SHOULDER");
            telemetry.addLine(String.format("Target / Actual: [%d] / [%d]", shoulderTicksTarget, shoulderTicksActual));
            telemetry.addLine(String.format("Current (Amps): %d", shoulderAmps));
            telemetry.addLine("ELBOW");
            telemetry.addLine(String.format("Target / Actual: [%d] / [%d]", elbowTicksTarget, elbowTicksActual));
            telemetry.addLine(String.format("Current (Amps): %d", elbowAmps));
            telemetry.update();

        }

    }
}
