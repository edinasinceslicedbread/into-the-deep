package org.firstinspires.ftc.teamcode.ctrl;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmController {

    //------------------------------------------------------------------------------------------------
    // Hardware
    //------------------------------------------------------------------------------------------------

    private DcMotorEx shoulderDrive = null;
    private DcMotorEx elbowDrive = null;

    //------------------------------------------------------------------------------------------------
    // Parameters and State
    //------------------------------------------------------------------------------------------------

    public static class ShoulderParams {

        // the shoulder motor has 1440 ticks per revolution
        public double ticksPerDegree = 1440.0 / 360.0;

        // controller tolerance
        public int positionTolerance = 50;  // encoder ticks
        public int velocityTolerance = 250; // encoder ticks per second, high value as this is pretty shaky

        // shoulder positions (in encoder ticks)
        public int homePosTicks = 0;
        public int drivePosTicks = 240;
        public int basketPosTicks = 360;
        public int pickPosTicks = 780;

        // motion profile constraints
        public int maxVelocity = 125;       // ticks per second
        public int maxAcceleration = 100;   // ticks per second per second
        public int stopVelocity = 200;      // ticks per second
        public int stopAcceleration = 150;   // ticks per second per second

        // feed forward parameters
        public double kS = 0.1;             // static FFW gain (power just to turn motor and gears freely)
        public double kCos = 0.6;           // gravity FFW gain (power to move arm up from horizontal)
        public double kV = 0.1;             // velocity FFW gain (applied to velocity set point)
        public double kA = 0.0;             // acceleration FFW gain (not used, or profile generator doesn't return acceleration)

        // control parameters
        public double kP = 0.015;            // proportional PID loop gain
        public double kI = 0.0;             // integral PID loop gain (not used)
        public double kD = 0.0;             // derivative PID loop gain (not used)

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

        // command flags
        public int lastGoalTargetTicks = 0;

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

    public static class ElbowParams {

        // the elbow motor has 240 ticks per revolution
        public double ticksPerDegree = 240.0 / 360.0;

        // elbow positions (in encoder ticks)
        public int homePosTicks = 0;
        public int drivePosTicks = 150;
        public int basketPosTicks = 85;
        public int pickPosTicks = 33;

        // home positions to clear 42 inch limits
        public int homeApproachTicks = -10;

        // motion profile constraints
        public int maxVelocity = 90;        // ticks per second
        public int maxAcceleration = 45;    // ticks per second per second
        public int stopVelocity = 180;      // ticks per second
        public int stopAcceleration = 90;   // ticks per second per second

        // controller tolerance
        public int positionTolerance = 4;         // encoder ticks
        public int velocityTolerance = 100;        // encoder ticks per second

        // control parameters
        public double kP = 0.05;     // proportional PID loop gain
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

        // command flags
        public int lastGoalTargetTicks = 0;

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

    //------------------------------------------------------------------------------------------------
    // Definitions
    //------------------------------------------------------------------------------------------------

    // parameters and state
    public ShoulderParams shoulderParams = new ShoulderParams();
    public ShoulderState shoulderState = new ShoulderState();
    public ElbowParams elbowParams = new ElbowParams();
    public ElbowState elbowState = new ElbowState();

    // elbow PID controller
    public ProfiledPIDController elbowController
            = new ProfiledPIDController(elbowParams.kP, elbowParams.kI, elbowParams.kD, elbowParams.getMaxConstraints());

    // shoulder FFW and PID controller
    public ArmFeedforward shoulderFeedforward = new ArmFeedforward(shoulderParams.kS, shoulderParams.kCos, shoulderParams.kV, shoulderParams.kA);
    public ProfiledPIDController shoulderController
            = new ProfiledPIDController(shoulderParams.kP, shoulderParams.kI, shoulderParams.kD, shoulderParams.getMaxConstraints());

    // button triggers
    public RisingEdgeTrigger homePosTrigger = new RisingEdgeTrigger();    // send to home button
    public RisingEdgeTrigger upperPosTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger pickPosTrigger = new RisingEdgeTrigger();    // send to pick button
    public RisingEdgeTrigger stopTrigger = new RisingEdgeTrigger();    // send to drop button

    //------------------------------------------------------------------------------------------------
    // Initialize
    //------------------------------------------------------------------------------------------------

    public void initialize(HardwareMap hardwareMap) {

        // hardware map
        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");

        // motor directions
        shoulderDrive.setDirection(DcMotorEx.Direction.REVERSE);
        elbowDrive.setDirection(DcMotorEx.Direction.REVERSE);

    }

    //------------------------------------------------------------------------------------------------
    // Reset
    //------------------------------------------------------------------------------------------------
    public void reset() {

        // shoulder reset
        shoulderDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulderDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulderController.setTolerance(shoulderParams.positionTolerance, shoulderParams.velocityTolerance);
        shoulderController.reset(shoulderDrive.getCurrentPosition());
        shoulderController.setGoal(new TrapezoidProfile.State(shoulderDrive.getCurrentPosition(), 0.0));

        // elbow reset
        elbowDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elbowDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbowController.setTolerance(elbowParams.positionTolerance, elbowParams.velocityTolerance);
        elbowController.reset(elbowDrive.getCurrentPosition());
        elbowController.setGoal(new TrapezoidProfile.State(elbowDrive.getCurrentPosition(), 0.0));

    }

    //------------------------------------------------------------------------------------------------
    // Update
    //------------------------------------------------------------------------------------------------
    public void update(ElapsedTime runtime, Gamepad gamepad1, Gamepad gamepad2) {

        // button triggers
        homePosTrigger.update(gamepad1.x);
        upperPosTrigger.update(gamepad1.y);
        pickPosTrigger.update(gamepad1.b);
        stopTrigger.update(gamepad1.a);

        // update status
        shoulderState.update(shoulderDrive, shoulderParams);
        elbowState.update(elbowDrive, elbowParams);

        //------------------------------------------------------------------------------------------------
        // Monitor Triggers
        //------------------------------------------------------------------------------------------------

        // go to home position triggered
        if (homePosTrigger.wasTriggered()) {

            // drive the shoulder to home and elbow to the approaching home clearance position
            shoulderController.setConstraints(shoulderParams.getMaxConstraints());
            shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.homePosTicks, 0.0));
            shoulderState.lastGoalTargetTicks = elbowParams.homePosTicks;
            // drive the elbow to the home position
            elbowController.setConstraints(elbowParams.getMaxConstraints());
            elbowController.setGoal(new TrapezoidProfile.State(elbowParams.homePosTicks, 0.0));
            elbowState.lastGoalTargetTicks = elbowParams.homePosTicks;

        }

        // go to upper basket or chamber position triggered
        if (upperPosTrigger.wasTriggered()) {

            if (shoulderState.lastGoalTargetTicks == elbowParams.drivePosTicks) {
                // if we were at submersible pick or chamber position then go up to basket position
                shoulderController.setConstraints(shoulderParams.getMaxConstraints());
                shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.basketPosTicks, 0.0));
                shoulderState.lastGoalTargetTicks = elbowParams.basketPosTicks;
                elbowController.setConstraints(elbowParams.getMaxConstraints());
                elbowController.setGoal(new TrapezoidProfile.State(elbowParams.basketPosTicks, 0.0));
                elbowState.lastGoalTargetTicks = elbowParams.basketPosTicks;
            } else {
                // otherwise by default go to the drive and chamber position
                shoulderController.setConstraints(shoulderParams.getMaxConstraints());
                shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.drivePosTicks, 0.0));
                shoulderState.lastGoalTargetTicks = elbowParams.drivePosTicks;
                elbowController.setConstraints(elbowParams.getMaxConstraints());
                elbowController.setGoal(new TrapezoidProfile.State(elbowParams.drivePosTicks, 0.0));
                elbowState.lastGoalTargetTicks = elbowParams.drivePosTicks;
            }
        }

        // go to pick from submersible location trigger
        if (pickPosTrigger.wasTriggered()) {
            shoulderController.setConstraints(shoulderParams.getMaxConstraints());
            shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.pickPosTicks, 0.0));
            shoulderState.lastGoalTargetTicks = elbowParams.pickPosTicks;
            elbowController.setConstraints(elbowParams.getMaxConstraints());
            elbowController.setGoal(new TrapezoidProfile.State(elbowParams.pickPosTicks, 0.0));
            elbowState.lastGoalTargetTicks = elbowParams.pickPosTicks;
        }

        // stop motion trigger
        if (stopTrigger.wasTriggered()) {
            shoulderController.setConstraints(shoulderParams.getStopConstraints());
            shoulderController.setGoal(new TrapezoidProfile.State(shoulderState.currentPosActual, 0.0));
            elbowController.setConstraints(elbowParams.getStopConstraints());
            elbowController.setGoal(new TrapezoidProfile.State(elbowState.currentPosActual, 0.0));
            elbowState.lastGoalTargetTicks = elbowState.currentPosActual;
        }

        //------------------------------------------------------------------------------------------------
        // Shoulder Control
        //------------------------------------------------------------------------------------------------

        // shoulder feed forward controller
        double shoulderPosTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().position)));
        double shoulderVelTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().velocity)));
        shoulderState.motorPowerFFW = shoulderFeedforward.calculate(shoulderPosTargetRad, shoulderVelTargetRad, 0.0);

        // shoulder PID controller
        shoulderState.motorPowerPID = shoulderController.calculate(shoulderState.currentPosActual);

        // shoulder power
        shoulderState.motorPower = shoulderState.motorPowerFFW + shoulderState.motorPowerPID;

        // limit to max motor power range
        if (shoulderState.motorPower > 1.0) {
            shoulderState.motorPower = 1.0;
        }
        if (shoulderState.motorPower < -1.0) {
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
        // Elbow Control
        //------------------------------------------------------------------------------------------------

        // elbow PID controller
        elbowState.motorPower = elbowController.calculate(elbowState.currentPosActual);

        // limit to max motor power range
        if (elbowState.motorPower > 0.3) {
            elbowState.motorPower = 0.3;
        }
        if (elbowState.motorPower < -0.3) {
            elbowState.motorPower = -0.3;
        }

        // elbow motor power
        elbowDrive.setPower(elbowState.motorPower);

    }

}
