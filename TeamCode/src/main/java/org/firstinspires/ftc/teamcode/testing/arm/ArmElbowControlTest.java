package org.firstinspires.ftc.teamcode.testing.arm;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;

@TeleOp(name = "ARM | 02 Elbow Control Test", group = "$$$$ Arm")
public class ArmElbowControlTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------
    private DcMotorEx elbowDrive = null;

    //------------------------------------------------------------------------------------------------
    // Parameters
    //------------------------------------------------------------------------------------------------

    public static class ElbowParams {

        // the elbow motor has 240 ticks per revolution
        public double ticksPerDegree = 240.0 / 360.0;

        // elbow positions (in encoder ticks)
        public int homePosTicks = 0;
        public int drivePosTicks = 150;
        public int basketPosTicks = 85;
        public int pickPosTicks = 33;

        // motion profile constraints
        public int maxVelocity = 90;        // ticks per second
        public int maxAcceleration = 45;    // ticks per second per second
        public int stopVelocity = 180;      // ticks per second
        public int stopAcceleration = 90;   // ticks per second per second

        // controller tolerance
        public int positionTolerance = 4;         // encoder ticks
        public int velocityTolerance = 100;        // encoder ticks per second

        // control parameters
        public double kP = 0.1;     // proportional PID loop gain
        public double kI = 0.0;     // integral PID loop gain (not used)
        public double kD = 0.0;     // derivative PID loop gain (not used)

        // function to convert degrees to encoder ticks
        public int degreesToTicks(double degrees) {
            return (int) Math.round(degrees * ticksPerDegree);
        }

        // function to convert encoder ticks to degrees
        public double ticksToDegree(int ticks) {
            return (double) ticks / ticksPerDegree;
        }

        public TrapezoidProfile.Constraints getMaxConstraints() {
            return new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        }

        public TrapezoidProfile.Constraints getStopConstraints() {
            return new TrapezoidProfile.Constraints(stopVelocity, stopAcceleration);
        }

    }

    public static class ElbowState {

        // input position and velocity
        public int currentPosActual = 0;
        public int currentVelActual = 0;

        // position flags
        public boolean atHomePos = false;
        public boolean atDrivePos = false;
        public boolean atBasketPos = false;
        public boolean atPickPos = false;

        // output power
        public double motorPower;

        public void update(DcMotorEx drive, ElbowParams params) {

            // input position and velocity
            currentPosActual = drive.getCurrentPosition();
            currentVelActual = params.degreesToTicks(drive.getVelocity(AngleUnit.DEGREES));

            // position flags
            atHomePos = Math.abs(currentPosActual - params.homePosTicks) < params.positionTolerance;
            atDrivePos = Math.abs(currentPosActual - params.drivePosTicks) < params.positionTolerance;
            atBasketPos = Math.abs(currentPosActual - params.basketPosTicks) < params.positionTolerance;
            atPickPos = Math.abs(currentPosActual - params.pickPosTicks) < params.positionTolerance;

        }

    }

    // parameters and state
    public ElbowParams elbowParams = new ElbowParams();
    public ElbowState elbowState = new ElbowState();

    // elbow PID controller
    public ProfiledPIDController elbowController
            = new ProfiledPIDController(elbowParams.kP, elbowParams.kI, elbowParams.kD, elbowParams.getMaxConstraints());

    // button triggers
    public RisingEdgeTrigger homePosTrigger = new RisingEdgeTrigger();    // send to home button
    public RisingEdgeTrigger upperPosTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger pickPosTrigger = new RisingEdgeTrigger();    // send to pick button
    public RisingEdgeTrigger stopTrigger = new RisingEdgeTrigger();    // send to drop button

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");
        elbowDrive.setDirection(DcMotorEx.Direction.REVERSE);

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
        // Reset
        //------------------------------------------------------------------------------------------------

        // elbow reset
        elbowDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elbowDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbowController.setTolerance(elbowParams.positionTolerance, elbowParams.velocityTolerance);
        elbowController.reset(elbowDrive.getCurrentPosition());
        elbowController.setGoal(new TrapezoidProfile.State(elbowDrive.getCurrentPosition(), 0.0));

        //------------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            // update triggers
            homePosTrigger.update(gamepad1.x);
            upperPosTrigger.update(gamepad1.y);
            pickPosTrigger.update(gamepad1.b);
            stopTrigger.update(gamepad1.a);

            // update status
            elbowState.update(elbowDrive, elbowParams);

            // watch triggers
            if (homePosTrigger.wasTriggered()) {
                elbowController.setConstraints(elbowParams.getMaxConstraints());
                elbowController.setGoal(new TrapezoidProfile.State(elbowParams.homePosTicks, 0.0));
            }
            if (upperPosTrigger.wasTriggered()) {
                if (elbowState.atDrivePos) {
                    elbowController.setConstraints(elbowParams.getMaxConstraints());
                    elbowController.setGoal(new TrapezoidProfile.State(elbowParams.basketPosTicks, 0.0));
                } else {
                    elbowController.setConstraints(elbowParams.getMaxConstraints());
                    elbowController.setGoal(new TrapezoidProfile.State(elbowParams.drivePosTicks, 0.0));
                }
            }
            if (pickPosTrigger.wasTriggered()) {
                elbowController.setConstraints(elbowParams.getMaxConstraints());
                elbowController.setGoal(new TrapezoidProfile.State(elbowParams.pickPosTicks, 0.0));
            }
            if (stopTrigger.wasTriggered()) {
                elbowController.setConstraints(elbowParams.getStopConstraints());
                elbowController.setGoal(new TrapezoidProfile.State(elbowState.currentPosActual, 0.0));
            }

            // elbow PID controller
            elbowState.motorPower = elbowController.calculate(elbowState.currentPosActual);

            // limit to max motor power range
            if (elbowState.motorPower > 0.3)
            {
                elbowState.motorPower = 0.3;
            }
            if (elbowState.motorPower < -0.3)
            {
                elbowState.motorPower = -0.3;
            }

            // elbow motor power
            elbowDrive.setPower(elbowState.motorPower);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the elbow.");
            telemetry.addLine(String.format("X [%s] -> Home Position", elbowState.atHomePos));
            telemetry.addLine(String.format("Y [%s] [%s]-> Basket or Chamber", elbowState.atBasketPos, elbowState.atDrivePos));
            telemetry.addLine(String.format("A [%s] -> Stop Motion", elbowState.currentVelActual > 0.0));
            telemetry.addLine(String.format("B [%s] -> Pick at Chamber Submersible", elbowState.atPickPos));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Elbow Target: TICKS[%d]", (int) Math.round(elbowController.getGoal().position)));
            telemetry.addLine(String.format("Elbow Status: PWR[%4.2f] TICKS[%d]", elbowState.motorPower, elbowState.currentPosActual));
            telemetry.addLine(String.format("Elbow Error: POS[%4.2f] VEL[%4.2f]", elbowController.getPositionError(), elbowController.getVelocityError()));
            telemetry.addLine(String.format("At: Goal[%s] SetPoint[%s]", elbowController.atGoal(), elbowController.atSetpoint()));
            telemetry.update();

        }

    }

}
