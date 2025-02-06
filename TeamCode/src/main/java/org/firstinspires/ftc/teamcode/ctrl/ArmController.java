package org.firstinspires.ftc.teamcode.ctrl;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmController {

    //------------------------------------------------------------------------------------------------
    // Hardware and Parameters
    //------------------------------------------------------------------------------------------------

    private DcMotorEx shoulderDrive = null;
    private DcMotorEx elbowDrive = null;

    public static class ShoulderParams {

        // motor with 60:1 gearbox has 1440 ticks per revolution
        public double ticksPerDegree = 1440.0 / 360.0;

        // shoulder position targets (encoder ticks)
        public int homePosTicks = 0;
        public int dropPosTicks = degreesToTicks(95.0);
        public int pickPosTicks = 780;
        public int inPosTolerance = 10;         // encoder ticks

        // profile parameters (in encoder ticks)
        public int maxVelocity = 100;           // ticks per second
        public int maxAcceleration = 50;       // ticks per second squared
        public int maxErrorTicks = 40;          // causes error

        // feed forward parameters
        public double ffw_kS = 0.0;             // static gain (not used)
        public double ffw_kCos = 0.6;          // gravity gain, holding power from lift arm push test
        public double ffw_kV = 0.01;            // velocity gain
        public double ffw_kA = 0.01;            // acceleration gain

        // position control parameters
        public double pos_kP = 0.016;           // proportional gain
        public double pos_kI = 0.05;            // integral gain
        public double pos_kD = 0.0;             // derivative gain

        // velocity control parameters
        public double vel_kP = 0.0;             // proportional gain
        public double vel_kI = 0.0;             // integral gain
        public double vel_kD = 0.0;             // derivative gain

        // function to convert degrees to encoder ticks
        public int degreesToTicks(double degrees) {
            return (int) Math.round(degrees * ticksPerDegree);
        }

        // function to convert encoder ticks to degrees
        public double ticksToDegree(int ticks) {
            return (double) ticks / ticksPerDegree;
        }

    }

    public static class ElbowParams {

        // the elbow motor has 288 ticks per revolution
        public double ticksPerDegree = 240.0 / 360.0;

        // max velocity (in degrees per second)
        public double maxVelocity = 30.0;

        // elbow positions in encoder ticks
        public int homePosTicks = 0;
        public int dropPosTicks = degreesToTicks(90.0);
        public int pickPosTicks = 50;
        public int inPosTolerance = 10;         // encoder ticks

        // control parameters
        public double pos_kP = 5.0;
        public double vel_kP = 5.0;
        public double vel_kI = 0.0;
        public double vel_kD = 0.0;
        public double vel_kF = 0.0;

        // function to convert degrees to encoder ticks
        public int degreesToTicks(double degrees) {
            return (int) Math.round(degrees * ticksPerDegree);
        }

        // function to convert encoder ticks to degrees
        public double ticksToDegree(int ticks) {
            return (double) ticks / ticksPerDegree;
        }

    }

    //------------------------------------------------------------------------------------------------
    // Arm State
    //------------------------------------------------------------------------------------------------

    public static class ShoulderState {

        private final ShoulderParams shoulderParams;

        private ShoulderState(ShoulderParams params) {
            shoulderParams = params;
        }

        // position status
        public int currentPosActual = 0;
        public int currentPosTarget = 0;
        public int currentPosError = 0;

        // velocity
        public int currentVelActual = 0;
        public int currentVelTarget = 0;
        public int currentVelError = 0;

        // acceleration
        private int currentAccTarget = 0;

        // position windows
        public boolean inHomeWindow = false;
        public boolean inDropWindow = false;
        public boolean inPickWindow = false;

        // output power
        public double shoulderPowerFfw;
        public double shoulderPowerPidPos;
        public double shoulderPowerPidVel;
        public double shoulderPower;

        public double shoulderPosTargetRad = 0.01;
        public double shoulderVelTargetRad = 0.0;
        public double shoulderAccTargetRad = 0.0;

        private void update(int currentPosition, double currentVelDegrees) {
            currentPosActual = currentPosition;
            currentPosError = currentPosTarget - currentPosActual;
            currentVelActual = shoulderParams.degreesToTicks(currentVelDegrees);
            currentVelError = currentVelTarget - currentVelActual;

            // position windows
            inHomeWindow = Math.abs(currentPosActual - shoulderParams.homePosTicks) < shoulderParams.inPosTolerance;
            inDropWindow = Math.abs(currentPosActual - shoulderParams.dropPosTicks) < shoulderParams.inPosTolerance;
            inPickWindow = Math.abs(currentPosActual - shoulderParams.pickPosTicks) < shoulderParams.inPosTolerance;

            // radians for feed forward controller
            shoulderPosTargetRad = Math.toRadians(shoulderParams.ticksToDegree(currentPosTarget));
            shoulderVelTargetRad = Math.toRadians(shoulderParams.ticksToDegree(currentVelTarget));
            shoulderAccTargetRad = Math.toRadians(shoulderParams.ticksToDegree(currentAccTarget));

        }

    }

    public ShoulderParams shoulderParams = new ShoulderParams();
    public ShoulderState shoulderState = new ShoulderState(shoulderParams);
    public ElbowParams elbowParams = new ElbowParams();

    //------------------------------------------------------------------------------------------------
    // Arm configuration
    //------------------------------------------------------------------------------------------------

    // control fields
    public int armControlState = 0;         // state machine variable
    public int currentProfileIndex = 0;
    public int profileEndPosition = 0;
    public double profileStartTime = 0.0;
    public double profileRunTime = 0.0;

    // state and triggers
    private final RisingEdgeTrigger homeTrigger = new RisingEdgeTrigger();    // send to home button
    private final RisingEdgeTrigger dropTrigger = new RisingEdgeTrigger();    // send to drop button
    private final RisingEdgeTrigger pickTrigger = new RisingEdgeTrigger();    // send to pick button

    // shoulder control
    public TrapezoidalMotionProfile.Profile[] shoulderProfile = new TrapezoidalMotionProfile.Profile[0];
    private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(shoulderParams.ffw_kS, shoulderParams.ffw_kCos, shoulderParams.ffw_kV, shoulderParams.ffw_kA);
    private final PIDController shoulderPosCtrl = new PIDController(shoulderParams.pos_kP, shoulderParams.pos_kI, shoulderParams.pos_kD);
    private final PController shoulderVelCtrl = new PController(shoulderParams.vel_kP);

    public void initialize(HardwareMap hardwareMap) {

        //------------------------------------------------------------------------------------------------
        // Arm Hardware Setup
        //------------------------------------------------------------------------------------------------

        // hardware map
        shoulderDrive = hardwareMap.get(DcMotorEx.class, "shoulderDrive");
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");

        // motor directions
        shoulderDrive.setDirection(DcMotorEx.Direction.REVERSE);
        elbowDrive.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void reset() {

        // shoulder motor
        shoulderState.currentPosTarget = shoulderDrive.getCurrentPosition();
        shoulderDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shoulderDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // elbow motor
        elbowDrive.setPositionPIDFCoefficients(elbowParams.pos_kP);
        elbowDrive.setVelocityPIDFCoefficients(elbowParams.vel_kP, elbowParams.vel_kI, elbowParams.vel_kD, elbowParams.vel_kF);
        elbowDrive.setTargetPosition(elbowDrive.getCurrentPosition());
        elbowDrive.setTargetPositionTolerance(elbowParams.inPosTolerance);
        elbowDrive.setVelocity(elbowParams.maxVelocity, AngleUnit.DEGREES);
        elbowDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elbowDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    public void update(ElapsedTime runtime, Gamepad gamepad1, Gamepad gamepad2) {

        //------------------------------------------------------------------------------------------------
        // Arm Control
        //------------------------------------------------------------------------------------------------

        // arm button triggers
        homeTrigger.update(gamepad1.x);
        dropTrigger.update(gamepad1.y);
        pickTrigger.update(gamepad1.b);

        // shoulder status and error
        shoulderState.update(shoulderDrive.getCurrentPosition(), shoulderDrive.getVelocity(AngleUnit.DEGREES));

        switch (armControlState) {

            case 0: // idle

                if (homeTrigger.wasTriggered()) {
                    elbowDrive.setTargetPosition(elbowParams.homePosTicks);
                    profileEndPosition = shoulderParams.homePosTicks;
                    armControlState = 1;
                }
                if (dropTrigger.wasTriggered()) {
                    elbowDrive.setTargetPosition(elbowParams.dropPosTicks);
                    profileEndPosition = shoulderParams.dropPosTicks;
                    armControlState = 1;
                }
                if (pickTrigger.wasTriggered()) {
                    elbowDrive.setTargetPosition(elbowParams.pickPosTicks);
                    profileEndPosition = shoulderParams.pickPosTicks;
                    armControlState = 1;
                }

                break;

            case 1: // calculate profile
                shoulderProfile = TrapezoidalMotionProfile.generateProfile(shoulderState.currentPosActual, profileEndPosition, shoulderParams.maxVelocity, shoulderParams.maxAcceleration);
                profileStartTime = runtime.seconds();
                currentProfileIndex = 0;
                armControlState = 2;
                break;

            case 2: // move shoulder to position

                // shoulder position                
                profileRunTime = runtime.seconds() - profileStartTime;
                for (int i = 0; i < shoulderProfile.length; i++) {
                    if (shoulderProfile[i].time >= profileRunTime) {
                        shoulderState.currentPosTarget = shoulderProfile[i].getPositionTicks();
                        shoulderState.currentVelTarget = shoulderProfile[i].getVelocityTicks();
                        shoulderState.currentAccTarget = shoulderProfile[i].getAccelerationTicks();
                        currentProfileIndex = i;
                        break;
                    }
                    if (i == shoulderProfile.length - 1) {
                        shoulderState.currentPosTarget = shoulderProfile[shoulderProfile.length - 1].getPositionTicks();
                        shoulderState.currentVelTarget = shoulderProfile[shoulderProfile.length - 1].getVelocityTicks();
                        shoulderState.currentAccTarget = shoulderProfile[shoulderProfile.length - 1].getAccelerationTicks();
                        armControlState = 0;
                    }
                }

                if (gamepad1.a || homeTrigger.wasTriggered() || dropTrigger.wasTriggered() || pickTrigger.wasTriggered()) {
                    shoulderState.currentPosTarget = shoulderState.currentPosActual;
                    shoulderState.currentVelTarget = 0;
                    shoulderState.currentAccTarget = 0;
                    armControlState = 0;
                }

                break;

            case 3: // error state

                shoulderState.currentPosTarget = shoulderState.currentPosActual;
                shoulderState.currentVelTarget = 0;
                shoulderState.currentAccTarget = 0;
                if (gamepad1.a || homeTrigger.wasTriggered() || dropTrigger.wasTriggered() || pickTrigger.wasTriggered()) {
                    armControlState = 0;
                }

                break;

        }

        // shoulder feed forward and PID control
        shoulderState.shoulderPowerFfw = shoulderFeedforward.calculate(shoulderState.shoulderPosTargetRad, shoulderState.shoulderVelTargetRad, shoulderState.shoulderAccTargetRad);
        shoulderState.shoulderPowerPidPos = shoulderPosCtrl.calculate(shoulderState.currentPosActual, shoulderState.currentPosTarget);
        shoulderState.shoulderPowerPidVel = shoulderVelCtrl.calculate(shoulderState.currentVelActual, shoulderState.currentVelTarget);

        // shoulder sum the controllers output power
        shoulderState.shoulderPower = shoulderState.shoulderPowerFfw + shoulderState.shoulderPowerPidPos + shoulderState.shoulderPowerPidVel;

        // check for exceeding following error limit
        if (Math.abs(shoulderState.currentPosError) > shoulderParams.maxErrorTicks || armControlState == 3) {
            shoulderState.shoulderPower = 0.0;
            armControlState = 3;
        }
        if (shoulderState.shoulderPower > 0.9) {
            shoulderState.shoulderPower = 0.9;
        }
        if (shoulderState.shoulderPower < -0.9) {
            shoulderState.shoulderPower = -0.9;
        }
        shoulderDrive.setPower(shoulderState.shoulderPower);

        // output elbow power
        elbowDrive.setPower(1.0);

    }

}
