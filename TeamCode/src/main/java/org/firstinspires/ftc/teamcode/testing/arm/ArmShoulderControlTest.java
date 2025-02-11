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

@TeleOp(name = "ARM | 03 Control Shoulder Test", group = "$$$$ Arm")
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

        // shoulder positions in encoder ticks
        public int homePickPosTicks = 0;
        public int basketDropPosTicks = degreesToTicks(95.0);
        public int chamberClipPosTicks = degreesToTicks(75.0);
        public int submersiblePickPosTicks = 750;

        // motion profile constraints (in encoder ticks)
        public int maxVelocity = 100;
        public int maxAcceleration = 50;

        // feed forward parameters
        public double kS = 0.1;             // static gain (not used)
        public double kCos = 0.6;           // gravity gain, holding power from lift arm push test
        public double kV = 0.1;             // velocity gain
        public double kA = 0.0;             // acceleration gain

        // control parameters
        public double kP = 0.01;
        public double kI = 0.0;
        public double kD = 0.0;

        // controller tolerance
        public int positionTolerance = 50;  // encoder ticks
        public int velocityTolerance = 100; // encoder ticks per second

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

    }

    public static class ShoulderState {

        // input readings
        public int currentPosActual = 0;
        public int currentVelActual = 0;

        // output power
        public double shoulderPowerFfw = 0.0;
        public double shoulderPowerPid = 0.0;
        public double shoulderPower = 0.0;

    }

    // parameters
    public ShoulderParams shoulderParams = new ShoulderParams();
    public ShoulderState shoulderState = new ShoulderState();

    // shoulder PID controller
    public ArmFeedforward shoulderFeedforward = new ArmFeedforward(shoulderParams.kS, shoulderParams.kCos, shoulderParams.kV, shoulderParams.kA);
    public ProfiledPIDController shoulderController
            = new ProfiledPIDController(shoulderParams.kP, shoulderParams.kI, shoulderParams.kD, shoulderParams.getProfileConstraints());

    // button triggers
    public RisingEdgeTrigger homePickTrigger = new RisingEdgeTrigger();    // send to home button
    public RisingEdgeTrigger upperBasketTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger chamberClipTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger submersiblePickTrigger = new RisingEdgeTrigger();    // send to pick button

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
            homePickTrigger.update(gamepad1.x);
            upperBasketTrigger.update(gamepad1.y);
            chamberClipTrigger.update(gamepad1.a);
            submersiblePickTrigger.update(gamepad1.b);

            // get current position and velocity
            shoulderState.currentPosActual = shoulderDrive.getCurrentPosition();
            shoulderState.currentVelActual = shoulderParams.degreesToTicks(shoulderDrive.getVelocity(AngleUnit.DEGREES));

            if (homePickTrigger.wasTriggered()) {
                int goalPosition = shoulderController.atGoal() || true ? shoulderParams.homePickPosTicks : shoulderState.currentPosActual;
                shoulderController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }
            if (upperBasketTrigger.wasTriggered()) {
                int goalPosition = shoulderController.atGoal() || true ? shoulderParams.basketDropPosTicks : shoulderState.currentPosActual;
                shoulderController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }
            if (chamberClipTrigger.wasTriggered()) {
                int goalPosition = shoulderController.atGoal() || true ? shoulderParams.chamberClipPosTicks : shoulderState.currentPosActual;
                shoulderController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }
            if (submersiblePickTrigger.wasTriggered()) {
                int goalPosition = shoulderController.atGoal() || true ? shoulderParams.submersiblePickPosTicks : shoulderState.currentPosActual;
                shoulderController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }

            // run feed forward controller
            double shoulderPosTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().position)));
            double shoulderVelTargetRad = Math.toRadians(shoulderParams.ticksToDegree((int) Math.round(shoulderController.getSetpoint().velocity)));
            shoulderState.shoulderPowerFfw = shoulderFeedforward.calculate(shoulderPosTargetRad, shoulderVelTargetRad, 0.0);

            // run PID controller
            shoulderState.shoulderPowerPid = shoulderController.calculate(shoulderState.currentPosActual);

            // set motor power
            shoulderState.shoulderPower = shoulderState.shoulderPowerFfw + shoulderState.shoulderPowerPid;

            // turn off power if at home
            if (shoulderController.getSetpoint().position < 10.0 && shoulderController.getPositionError() < 20.0) {
                shoulderState.shoulderPower = 0.0;
            }

            // limit to max motor power range
            if (shoulderState.shoulderPower > 1.0)
            {
                shoulderState.shoulderPower = 1.0;
            }
            if (shoulderState.shoulderPower < -1.0)
            {
                shoulderState.shoulderPower = -1.0;
            }
            shoulderDrive.setPower(shoulderState.shoulderPower);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the shoulder.");
            telemetry.addLine("X -> Home Pick, Y -> Basket Drop, A -> Chamber Clip, B -> Submersible Pick");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Power: FFW[%4.2f] PID[%4.2f]", shoulderState.shoulderPowerFfw, shoulderState.shoulderPowerPid));
            telemetry.addLine(String.format("Shoulder Target: GOAL[%d] SP[%d]", (int) Math.round(shoulderController.getGoal().position), (int) Math.round(shoulderController.getSetpoint().position)));
            telemetry.addLine(String.format("Shoulder Status: PWR[%4.2f] TICKS[%d]", shoulderState.shoulderPower, shoulderState.currentPosActual));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Shoulder Error: POS[%4.2f] VEL[%4.2f]", shoulderController.getPositionError(), shoulderController.getVelocityError()));
            telemetry.addLine(String.format("At: Goal[%s] SetPoint[%s]", shoulderController.atGoal(), shoulderController.atSetpoint()));
            telemetry.update();

        }

    }

}
