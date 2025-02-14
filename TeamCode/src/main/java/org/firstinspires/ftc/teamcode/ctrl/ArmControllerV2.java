package org.firstinspires.ftc.teamcode.ctrl;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmControllerV2 {

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
        public int positionTolerance = 40;  // encoder ticks
        public int velocityTolerance = 250; // encoder ticks per second, high value as this is pretty shaky

        // shoulder positions (in encoder ticks)
        public int homePosTicks = 0;
        public int readyPosTicks = degreesToTicks(87.0);
        public int chamberPosTicks = degreesToTicks(68.0);
        public int basketPosTicks = degreesToTicks(88.0);
        public int extendPosTicks = 780;
        public int pickupPosTicks = 780;

        // motion profile constraints
        public int maxVelocity = 150;       // ticks per second
        public int maxAcceleration = 150;   // ticks per second per second
        public int stopVelocity = 200;      // ticks per second
        public int stopAcceleration = 150;   // ticks per second per second

        // feed forward parameters for profile positions
        public double kS = 0.1;             // static FFW gain (power just to turn motor and gears freely)
        public double kCos = 0.6;           // gravity FFW gain (power to move arm up from horizontal)
        public double kV = 0.1;             // velocity FFW gain (applied to velocity set point)
        public double kA = 0.0;             // acceleration FFW gain (not used, or profile generator doesn't return acceleration)

        // control parameters
        public double kP = 0.025;            // proportional PID loop gain
        public double kI = 0.0;              // integral PID loop gain (not used)
        public double kD = 0.0;              // derivative PID loop gain (not used)

        // function to convert degrees to encoder ticks
        public int degreesToTicks(double degrees) {
            return (int) Math.round(degrees * ticksPerDegree);
        }

        // function to convert encoder ticks to degrees
        public double ticksToDegree(int ticks) {
            return (double) ticks / ticksPerDegree;
        }

        public TrapezoidProfile.Constraints getProfileConstraints() {
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
        public boolean atReadyPos = false;
        public boolean atChamberPos = false;
        public boolean atBasketPos = false;
        public boolean atExtendPos = false;
        public boolean atPickupPos = false;

        // command flags
        public int lastMoveTargetTicks = 0;

        // output power
        public double motorPowerFFW = 0.0;
        public double motorPowerPID = 0.0;
        public double motorPower = 0.0;

        public void update(DcMotorEx drive, ShoulderParams params, ProfiledPIDController controller) {

            // input position and velocity
            currentPosActual = drive.getCurrentPosition();
            currentVelActual = params.degreesToTicks(drive.getVelocity(AngleUnit.DEGREES));

            // position flags
            atHomePos = Math.abs(currentPosActual - params.homePosTicks) < params.positionTolerance && controller.atGoal();
            atReadyPos = Math.abs(currentPosActual - params.readyPosTicks) < params.positionTolerance && controller.atGoal();
            atChamberPos = Math.abs(currentPosActual - params.chamberPosTicks) < params.positionTolerance && controller.atGoal();
            atBasketPos = Math.abs(currentPosActual - params.basketPosTicks) < params.positionTolerance && controller.atGoal();
            atExtendPos = Math.abs(currentPosActual - params.extendPosTicks) < params.positionTolerance && controller.atGoal();
            atPickupPos = Math.abs(currentPosActual - params.pickupPosTicks) < params.positionTolerance && controller.atGoal();

        }

        public boolean atStopSupportPos(ShoulderParams params, ProfiledPIDController controller) {
            boolean atExtendStop = Math.abs(currentPosActual - params.extendPosTicks) < params.positionTolerance
                    && controller.getSetpoint().position == params.extendPosTicks;
            boolean atHomeStop = Math.abs(currentPosActual - params.homePosTicks) < params.positionTolerance
                    && controller.getSetpoint().position == params.homePosTicks;
            return atExtendStop || atHomeStop;
        }

    }

    public static class ElbowParams {

        // the elbow motor has 240 ticks per revolution
        public double ticksPerDegree = 240.0 / 360.0;

        // controller tolerance
        public int positionTolerance = 3;         // encoder ticks
        public int velocityTolerance = 50;        // encoder ticks per second

        // elbow positions (in encoder ticks)
        public int homePosTicks = 0;
        public int readyPosTicks = -10;
        public int chamberPosTicks = 150;
        public int basketPosTicks = 90;
        public int extendPosTicks = 34;
        public int pickupPosTicks = 50;

        // home positions to clear 42 inch limits
        public int homeApproachTicks = -10;

        // motion profile constraints
        public int maxVelocity = 90;        // ticks per second
        public int maxAcceleration = 45;    // ticks per second per second
        public int stopVelocity = 180;      // ticks per second
        public int stopAcceleration = 90;   // ticks per second per second

        // control parameters
        public double kP = 0.08;     // proportional PID loop gain
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

        public TrapezoidProfile.Constraints getProfileConstraints() {
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
        public boolean atReadyPos = false;
        public boolean atChamberPos = false;
        public boolean atBasketPos = false;
        public boolean atExtendPos = false;
        public boolean atPickupPos = false;

        // command flags
        public int lastMoveTargetTicks = 0;

        // output power
        public double motorPower;

        public void update(DcMotorEx drive, ElbowParams params, ProfiledPIDController controller) {

            // input position and velocity
            currentPosActual = drive.getCurrentPosition();
            currentVelActual = params.degreesToTicks(drive.getVelocity(AngleUnit.DEGREES));

            // position flags
            atHomePos = Math.abs(currentPosActual - params.homePosTicks) < params.positionTolerance && controller.atGoal();
            atReadyPos = Math.abs(currentPosActual - params.readyPosTicks) < params.positionTolerance && controller.atGoal();
            atChamberPos = Math.abs(currentPosActual - params.chamberPosTicks) < params.positionTolerance && controller.atGoal();
            atBasketPos = Math.abs(currentPosActual - params.basketPosTicks) < params.positionTolerance && controller.atGoal();
            atExtendPos = Math.abs(currentPosActual - params.extendPosTicks) < params.positionTolerance && controller.atGoal();
            atPickupPos = Math.abs(currentPosActual - params.pickupPosTicks) < params.positionTolerance && controller.atGoal();

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
    public ProfiledPIDController elbowController = new ProfiledPIDController(elbowParams.kP, elbowParams.kI, elbowParams.kD, elbowParams.getProfileConstraints());

    // shoulder FFW and PID controller
    public ArmFeedforward shoulderFeedForward = new ArmFeedforward(shoulderParams.kS, shoulderParams.kCos, shoulderParams.kV, shoulderParams.kA);
    public ProfiledPIDController shoulderController = new ProfiledPIDController(shoulderParams.kP, shoulderParams.kI, shoulderParams.kD, shoulderParams.getProfileConstraints());

    // button triggers
    public RisingEdgeTrigger homePosTrigger = new RisingEdgeTrigger();      // move arm to home position button
    public RisingEdgeTrigger upperPosTrigger = new RisingEdgeTrigger();     // move arm to basked or chamber drop position button
    public RisingEdgeTrigger extendPosTrigger = new RisingEdgeTrigger();    // move arm to extended position at submersible button
    public RisingEdgeTrigger stopTrigger = new RisingEdgeTrigger();         // stop arm movement immediately button

    // controller variables
    public int armControlState = 0;         // state machine variable

    // status variables
    public boolean atHomePos = false;
    public boolean atReadyPos = false;
    public boolean atChamberPos = false;
    public boolean atBasketPos = false;
    public boolean atExtendedPos = false;
    public boolean atPickupPos = false;

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
        update(runtime, gamepad1, gamepad2, 0);
    }

    public void update(ElapsedTime runtime, Gamepad gamepad1, Gamepad gamepad2, int armAutoCmd) {

        // button triggers
        homePosTrigger.update(gamepad1.x || gamepad2.x || armAutoCmd == 1);
        upperPosTrigger.update(gamepad1.y || gamepad2.y || armAutoCmd == 2);
        extendPosTrigger.update(gamepad1.b || gamepad2.b || armAutoCmd == 3);
        stopTrigger.update(gamepad1.a || gamepad2.a || armAutoCmd == 4);

        // update status
        shoulderState.update(shoulderDrive, shoulderParams, shoulderController);
        elbowState.update(elbowDrive, elbowParams, elbowController);

        // update status variables
        atHomePos = shoulderState.atHomePos && elbowState.atHomePos;
        atReadyPos = shoulderState.atReadyPos && elbowState.atReadyPos;
        atChamberPos = shoulderState.atChamberPos && elbowState.atChamberPos;
        atBasketPos = shoulderState.atBasketPos && elbowState.atBasketPos;
        atExtendedPos = shoulderState.atExtendPos && elbowState.atExtendPos;
        atPickupPos = shoulderState.atPickupPos && elbowState.atPickupPos;

        //------------------------------------------------------------------------------------------------
        // Monitor Triggers
        //------------------------------------------------------------------------------------------------

        // watch for stop trigger
        if (stopTrigger.wasTriggered()) {
            shoulderController.setConstraints(shoulderParams.getStopConstraints());
            elbowController.setConstraints(elbowParams.getStopConstraints());
            shoulderController.setGoal(new TrapezoidProfile.State(shoulderState.currentPosActual, 0.0));
            elbowController.setGoal(new TrapezoidProfile.State(elbowState.currentPosActual, 0.0));
            armControlState = 0;
        }

        switch (armControlState) {

            case 0: // idle state

                // home position trigger
                if (homePosTrigger.wasTriggered() && !shoulderState.atHomePos) {
                    if (armAutoCmd == 1)
                    {
                        shoulderState.lastMoveTargetTicks = shoulderParams.homePosTicks;
                        elbowState.lastMoveTargetTicks = elbowParams.homePosTicks;
                    }
                    else if (shoulderState.lastMoveTargetTicks == shoulderParams.basketPosTicks || shoulderState.lastMoveTargetTicks == shoulderParams.extendPosTicks) {
                        shoulderState.lastMoveTargetTicks = shoulderParams.chamberPosTicks;
                        elbowState.lastMoveTargetTicks = elbowParams.chamberPosTicks;
                    } else {
                        shoulderState.lastMoveTargetTicks = shoulderParams.homePosTicks;
                        elbowState.lastMoveTargetTicks = elbowParams.homePosTicks;
                    }
                    armControlState = 1;
                }

                // upper position trigger
                if (upperPosTrigger.wasTriggered()) {
                    if (shoulderState.lastMoveTargetTicks == shoulderParams.chamberPosTicks || armAutoCmd == 2) {
                        shoulderState.lastMoveTargetTicks = shoulderParams.basketPosTicks;
                        elbowState.lastMoveTargetTicks = elbowParams.basketPosTicks;
                    } else {
                        shoulderState.lastMoveTargetTicks = shoulderParams.chamberPosTicks;
                        elbowState.lastMoveTargetTicks = elbowParams.chamberPosTicks;
                    }
                    armControlState = 1;
                }

                // extended position trigger
                if (extendPosTrigger.wasTriggered()) {
                    if (shoulderState.lastMoveTargetTicks == shoulderParams.basketPosTicks) {
                        shoulderState.lastMoveTargetTicks = shoulderParams.chamberPosTicks;
                        elbowState.lastMoveTargetTicks = elbowParams.chamberPosTicks;
                    } else {
                        if (elbowState.lastMoveTargetTicks == elbowParams.extendPosTicks) {
                            shoulderState.lastMoveTargetTicks = shoulderParams.pickupPosTicks;
                            elbowState.lastMoveTargetTicks = elbowParams.pickupPosTicks;
                        } else {
                            shoulderState.lastMoveTargetTicks = shoulderParams.extendPosTicks;
                            elbowState.lastMoveTargetTicks = elbowParams.extendPosTicks;
                        }
                    }
                    armControlState = 1;
                }

                break;

            case 1: // check if move to ready position is needed
                if (shoulderState.atHomePos || shoulderState.lastMoveTargetTicks == shoulderParams.homePosTicks) {
                    shoulderController.setConstraints(shoulderParams.getProfileConstraints());
                    elbowController.setConstraints(elbowParams.getProfileConstraints());
                    shoulderController.setGoal(new TrapezoidProfile.State(shoulderParams.readyPosTicks, 0.0));
                    elbowController.setGoal(new TrapezoidProfile.State(elbowParams.readyPosTicks, 0.0));
                    armControlState = 2;
                } else {
                    armControlState = 3;
                }
                break;

            case 2: // wait for the move to ready position to complete then finish original target move
                if (shoulderState.atReadyPos && elbowState.atReadyPos) {
                    armControlState = 3;
                }
                break;

            case 3: // initiate the move

                if (shoulderState.lastMoveTargetTicks == shoulderParams.homePosTicks) {
                    shoulderController.setConstraints(shoulderParams.getProfileConstraints());
                    shoulderController.setGoal(new TrapezoidProfile.State(shoulderState.lastMoveTargetTicks, 0.0));
                    armControlState = 4;
                } else {
                    shoulderController.setConstraints(shoulderParams.getProfileConstraints());
                    shoulderController.setGoal(new TrapezoidProfile.State(shoulderState.lastMoveTargetTicks, 0.0));
                    elbowController.setConstraints(elbowParams.getProfileConstraints());
                    elbowController.setGoal(new TrapezoidProfile.State(elbowState.lastMoveTargetTicks, 0.0));
                    armControlState = 5;
                }
                break;

            case 4: // move elbow to home position
                if (shoulderController.atGoal() && elbowController.atGoal()) {
                    elbowController.setConstraints(elbowParams.getProfileConstraints());
                    elbowController.setGoal(new TrapezoidProfile.State(elbowState.lastMoveTargetTicks, 0.0));
                    armControlState = 5;
                }
                break;

            case 5: // wait for move complete
                if (shoulderController.atGoal() && elbowController.atGoal()) {
                    armControlState = 0;
                }
                if (elbowState.lastMoveTargetTicks == elbowParams.pickupPosTicks)
                {
                    armControlState = 0;
                }
                break;

        }


        //------------------------------------------------------------------------------------------------
        // Shoulder Control
        //------------------------------------------------------------------------------------------------

        // shoulder feed forward controller
        double shoulderPosTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().position)));
        double shoulderVelTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().velocity)));
        shoulderState.motorPowerFFW = shoulderFeedForward.calculate(shoulderPosTargetRad, shoulderVelTargetRad, 0.0);

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

        // turn off power if at home or extended positions
        if (shoulderState.atStopSupportPos(shoulderParams, shoulderController)) {
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
