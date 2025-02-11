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
        public double ticksPerDegree = 288.0 / 360.0;

        // elbow positions in encoder ticks
        public int homePosTicks = 0;
        public int basketPosTicks = 74;
        public int pickPosTicks = 33;
        public int drivePosTicks = 173;

        // motion profile constraints (in encoder ticks)
        public int maxVelocity = 90;
        public int maxAcceleration = 45;
        public int stopVelocity = 180;
        public int stopAcceleration = 90;

        // controller tolerance
        public int positionTolerance = 4;         // encoder ticks
        public int velocityTolerance = 10;         // encoder ticks per second

        // control parameters
        public double kP = 0.1;
        public double kI = 0.0;
        public double kD = 0.0;

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
        public boolean atBasketPos = false;
        public boolean atPickPos = false;
        public boolean atDrivePos = false;

        // output power
        public double elbowPower;

        public void update(DcMotorEx elbowDrive, ElbowParams elbowParams) {

            currentPosActual = elbowDrive.getCurrentPosition();
            currentVelActual = elbowParams.degreesToTicks(elbowDrive.getVelocity(AngleUnit.DEGREES));

            atHomePos = Math.abs(currentPosActual - elbowParams.homePosTicks) <= elbowParams.positionTolerance;
            atBasketPos = Math.abs(currentPosActual - elbowParams.basketPosTicks) <= elbowParams.positionTolerance;
            atPickPos = Math.abs(currentPosActual - elbowParams.pickPosTicks) <= elbowParams.positionTolerance;
            atDrivePos = Math.abs(currentPosActual - elbowParams.drivePosTicks) <= elbowParams.positionTolerance;

        }

    }

    // parameters
    public ElbowParams elbowParams = new ElbowParams();
    public ElbowState elbowState = new ElbowState();

    // elbow PID controller

    public ProfiledPIDController elbowController
            = new ProfiledPIDController(elbowParams.kP, elbowParams.kI, elbowParams.kD, elbowParams.getMaxConstraints());

    // button triggers
    public RisingEdgeTrigger homePosTrigger = new RisingEdgeTrigger();    // send to home button
    public RisingEdgeTrigger basketPosTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger drivePosTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger pickPosTrigger = new RisingEdgeTrigger();    // send to pick button

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
            basketPosTrigger.update(gamepad1.y);
            pickPosTrigger.update(gamepad1.b);
            drivePosTrigger.update(gamepad1.a);

            // update elbow state
            elbowState.update(elbowDrive, elbowParams);

            if (homePosTrigger.wasTriggered()) {
                elbowController.setConstraints(elbowParams.getMaxConstraints());
                elbowController.setGoal(new TrapezoidProfile.State(elbowParams.homePosTicks, 0.0));
            }
            if (basketPosTrigger.wasTriggered()) {
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
            if (drivePosTrigger.wasTriggered()) {
                elbowController.setConstraints(elbowParams.getStopConstraints());
                elbowController.setGoal(new TrapezoidProfile.State(elbowState.currentPosActual, 0.0));
            }

            // run PID controller
            elbowState.elbowPower = elbowController.calculate(elbowState.currentPosActual);

            // torque limit
            if (elbowState.elbowPower > 0.2) {
                elbowState.elbowPower = 0.2;
            }

            // set motor power
            elbowDrive.setPower(elbowState.elbowPower);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the elbow.");
            telemetry.addLine("X -> Home Pick, Y -> Basket Drop, A -> Chamber Clip, B -> Submersible Pick");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Elbow Target: TICKS[%d]", (int) Math.round(elbowController.getGoal().position)));
            telemetry.addLine(String.format("Elbow Status: PWR[%4.2f] TICKS[%d]", elbowState.elbowPower, elbowState.currentPosActual));
            telemetry.addLine(String.format("Elbow Error: POS[%4.2f] VEL[%4.2f]", elbowController.getPositionError(), elbowController.getVelocityError()));
            telemetry.addLine(String.format("At: Goal[%s] SetPoint[%s]", elbowController.atGoal(), elbowController.atSetpoint()));
            telemetry.update();

        }

    }

}
