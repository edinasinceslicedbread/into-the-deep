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

        // elbow positions in encoder ticks
        public int homePickPosTicks = 0;
        public int basketDropPosTicks = degreesToTicks(95.0);
        public int chamberClipPosTicks = degreesToTicks(135.0);
        public int submersiblePickPosTicks = 50;

        // motion profile constraints (in encoder ticks)
        public int maxVelocity = 90;
        public int maxAcceleration = 45;

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

        public TrapezoidProfile.Constraints getProfileConstraints() {
            return new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        }

    }

    public static class ElbowState {
        public int currentPosActual = 0;
        public int currentVelActual = 0;
    }

    // parameters
    public ElbowParams elbowParams = new ElbowParams();
    public ElbowState elbowState = new ElbowState();

    // elbow PID controller

    public ProfiledPIDController elbowController
            = new ProfiledPIDController(elbowParams.kP, elbowParams.kI, elbowParams.kD, elbowParams.getProfileConstraints());

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
            homePickTrigger.update(gamepad1.x);
            upperBasketTrigger.update(gamepad1.y);
            chamberClipTrigger.update(gamepad1.a);
            submersiblePickTrigger.update(gamepad1.b);

            // get current position and velocity
            elbowState.currentPosActual = elbowDrive.getCurrentPosition();
            elbowState.currentVelActual = elbowParams.degreesToTicks(elbowDrive.getVelocity(AngleUnit.DEGREES));

            if (homePickTrigger.wasTriggered()) {
                int goalPosition = elbowController.atGoal() ? elbowParams.homePickPosTicks : elbowState.currentPosActual;
                elbowController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }
            if (upperBasketTrigger.wasTriggered()) {
                int goalPosition = elbowController.atGoal() ? elbowParams.basketDropPosTicks : elbowState.currentPosActual;
                elbowController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }
            if (chamberClipTrigger.wasTriggered()) {
                int goalPosition = elbowController.atGoal() ? elbowParams.chamberClipPosTicks : elbowState.currentPosActual;
                elbowController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }
            if (submersiblePickTrigger.wasTriggered()) {
                int goalPosition = elbowController.atGoal() ? elbowParams.submersiblePickPosTicks : elbowState.currentPosActual;
                elbowController.setGoal(new TrapezoidProfile.State(goalPosition, 0.0));
            }

            // run PID controller
            double elbowPower = elbowController.calculate(elbowState.currentPosActual);

            // set motor power
            if (elbowPower > 0.2) {
                elbowPower = 0.2;
            }
            elbowDrive.setPower(elbowPower);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the elbow.");
            telemetry.addLine("X -> Home Pick, Y -> Basket Drop, A -> Chamber Clip, B -> Submersible Pick");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Elbow Target: TICKS[%d]", (int) Math.round(elbowController.getGoal().position)));
            telemetry.addLine(String.format("Elbow Status: PWR[%4.2f] TICKS[%d]", elbowPower, elbowState.currentPosActual));
            telemetry.addLine(String.format("Elbow Error: POS[%4.2f] VEL[%4.2f]", elbowController.getPositionError(), elbowController.getVelocityError()));
            telemetry.addLine(String.format("At: Goal[%s] SetPoint[%s]", elbowController.atGoal(), elbowController.atSetpoint()));
            telemetry.update();

        }

    }

}
