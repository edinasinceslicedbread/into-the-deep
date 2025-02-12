package org.firstinspires.ftc.teamcode.testing.arm;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;

@TeleOp(name = "ARM | 03 Shoulder Control Test", group = "$$$$ Arm")
public class ArmShoulderControlTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------
    private DcMotorEx shoulderDrive = null;

    //------------------------------------------------------------------------------------------------
    // Parameters
    //------------------------------------------------------------------------------------------------

    public static class ShoulderParams {

        // the shoulder motor has 1440 ticks per revolution
        public double ticksPerDegree = 1440.0 / 360.0;

        // shoulder positions (in encoder ticks)
        public int homePosTicks = 0;
        public int drivePosTicks = 240;
        public int basketPosTicks = 360;
        public int pickPosTicks = 780;

        // motion profile constraints
        public int maxVelocity = 150;       // ticks per second
        public int maxAcceleration = 125;   // ticks per second per second
        public int stopVelocity = 100;      // ticks per second
        public int stopAcceleration = 50;   // ticks per second per second

        // feed forward parameters
        public double kS = 0.1;             // static FFW gain (power just to turn motor and gears freely)
        public double kCos = 0.7;           // gravity FFW gain (power to move arm up from horizontal)
        public double kV = 0.05;             // velocity FFW gain (applied to velocity set point)
        public double kA = 0.1;             // acceleration FFW gain (not used, or profile generator doesn't return acceleration)

        // control parameters
        public double kP = 0.03;            // proportional PID loop gain
        public double kI = 0.0;             // integral PID loop gain (not used)
        public double kD = 0.0;             // derivative PID loop gain (not used)

        // controller tolerance
        public int positionTolerance = 50;  // encoder ticks
        public int velocityTolerance = 250; // encoder ticks per second, high value as this is pretty shaky

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

    public static class ShoulderState {

        // input position and velocity
        public int currentPosActual = 0;
        public int currentVelActual = 0;

        // position flags
        public boolean atHomePos = false;
        public boolean atDrivePos = false;
        public boolean atBasketPos = false;
        public boolean atPickPos = false;

        public boolean atHomeGoal = false;
        public boolean atDriveGoal = false;
        public boolean atBasketGoal = false;
        public boolean atPickGoal = false;

        // output power
        public double motorPowerFFW = 0.0;
        public double motorPowerPID = 0.0;
        public double motorPower = 0.0;

        public void update(DcMotorEx drive, ShoulderParams params) {

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
    public ShoulderParams shoulderParams = new ShoulderParams();
    public ShoulderState shoulderState = new ShoulderState();

    // shoulder FFW and PID controller
    public ArmFeedforward shoulderFeedforward = new ArmFeedforward(shoulderParams.kS, shoulderParams.kCos, shoulderParams.kV, shoulderParams.kA);
    public ProfiledPIDController shoulderController
            = new ProfiledPIDController(shoulderParams.kP, shoulderParams.kI, shoulderParams.kD, shoulderParams.getMaxConstraints());

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

        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        shoulderDrive.setDirection(DcMotorEx.Direction.REVERSE);

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

        // shoulder reset
        shoulderDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulderDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulderController.setTolerance(shoulderParams.positionTolerance, shoulderParams.velocityTolerance);
        shoulderController.reset(shoulderDrive.getCurrentPosition());
        shoulderController.setGoal(new TrapezoidProfile.State(shoulderDrive.getCurrentPosition(), 0.0));

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
            shoulderState.update(shoulderDrive, shoulderParams);

            // watch triggers
            if (homePosTrigger.wasTriggered()) {
                shoulderController.setConstraints(shoulderParams.getMaxConstraints());
                shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.homePosTicks, 0.0));
            }
            if (upperPosTrigger.wasTriggered()) {
                if (shoulderState.atDrivePos) {
                    shoulderController.setConstraints(shoulderParams.getMaxConstraints());
                    shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.basketPosTicks, 0.0));
                } else {
                    shoulderController.setConstraints(shoulderParams.getMaxConstraints());
                    shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.drivePosTicks, 0.0));
                }
            }
            if (pickPosTrigger.wasTriggered()) {
                shoulderController.setConstraints(shoulderParams.getMaxConstraints());
                shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.pickPosTicks, 0.0));
            }
            if (stopTrigger.wasTriggered()) {
                shoulderController.setConstraints(shoulderParams.getStopConstraints());
                shoulderController.setGoal(new TrapezoidProfile.State(shoulderState.currentPosActual, 0.0));
            }

            // shoulder feed forward controller
            double shoulderPosTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().position)));
            double shoulderVelTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().velocity)));
            shoulderState.motorPowerFFW = shoulderFeedforward.calculate(shoulderPosTargetRad, shoulderVelTargetRad, 0.0);

            // shoulder PID controller
            shoulderState.motorPowerPID = shoulderController.calculate(shoulderState.currentPosActual);

            // shoulder power
            shoulderState.motorPower = shoulderState.motorPowerFFW + shoulderState.motorPowerPID;

            // limit to max motor power range
            if (shoulderState.motorPower > 1.0)
            {
                shoulderState.motorPower = 1.0;
            }
            if (shoulderState.motorPower < -1.0)
            {
                shoulderState.motorPower = -1.0;
            }

            // turn off power if at home or pick positions
            if (shoulderState.atHomePos && shoulderController.atGoal()) {
                shoulderState.motorPower = 0.0;
            }
            if (shoulderState.atPickPos && shoulderController.atGoal()) {
                shoulderState.motorPower = 0.0;
            }

            shoulderDrive.setPower(shoulderState.motorPower);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the shoulder.");
            telemetry.addLine(String.format("X [%s] -> Home Position", shoulderState.atHomePos));
            telemetry.addLine(String.format("Y [%s] [%s]-> Basket or Chamber", shoulderState.atBasketPos, shoulderState.atDrivePos));
            telemetry.addLine(String.format("A [%s] -> Stop Motion", shoulderState.currentVelActual > 0.0));
            telemetry.addLine(String.format("B [%s] -> Pick at Chamber Submersible", shoulderState.atPickPos));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Power: FFW[%4.2f] PID[%4.2f]", shoulderState.motorPowerFFW, shoulderState.motorPowerPID));
            telemetry.addLine(String.format("Shoulder Target: GOAL[%d] SP[%d]", (int) Math.round(shoulderController.getGoal().position), (int) Math.round(shoulderController.getSetpoint().position)));
            telemetry.addLine(String.format("Shoulder Status: PWR[%4.2f] TICKS[%d]", shoulderState.motorPower, shoulderState.currentPosActual));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Shoulder Error: POS[%4.2f] VEL[%4.2f]", shoulderController.getPositionError(), shoulderController.getVelocityError()));
            telemetry.addLine(String.format("At: Goal[%s] SetPoint[%s]", shoulderController.atGoal(), shoulderController.atSetpoint()));
            telemetry.update();

        }

    }

}
