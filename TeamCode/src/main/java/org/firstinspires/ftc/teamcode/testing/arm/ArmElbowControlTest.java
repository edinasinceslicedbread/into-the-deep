package org.firstinspires.ftc.teamcode.testing.arm;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;

@TeleOp(name = "ARM | Elbow Control Test", group = "$$$$ Arm")
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

        // the elbow motor has 288 ticks per revolution
        public double ticksPerDegree = 240.0 / 360.0;

        // max velocity (in degrees per second)
        public double maxVelocity = 30.0;

        // elbow positions in encoder ticks
        public int homePosTicks = 0;
        public int dropPosTicks = degreesToTicks(90.0);
        public int drivePosTicks = degreesToTicks(60.0);
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

    // parameters
    public ElbowParams elbowParams = new ElbowParams();

    // control varialbes
    // button triggers
    public RisingEdgeTrigger homeTrigger = new RisingEdgeTrigger();    // send to home button
    public RisingEdgeTrigger dropTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger driveTrigger = new RisingEdgeTrigger();    // send to drop button
    public RisingEdgeTrigger pickTrigger = new RisingEdgeTrigger();    // send to pick button
    double elbowTicksTarget = 0.0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        // hardware map
        elbowDrive = hardwareMap.get(DcMotorEx.class, "elbowDrive");

        // motor directions
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

        // reset elbow motor
        elbowDrive.setPositionPIDFCoefficients(elbowParams.pos_kP);
        elbowDrive.setVelocityPIDFCoefficients(elbowParams.vel_kP, elbowParams.vel_kI, elbowParams.vel_kD, elbowParams.vel_kF);
        elbowDrive.setTargetPosition(elbowDrive.getCurrentPosition());
        elbowDrive.setTargetPositionTolerance(elbowParams.inPosTolerance);
        elbowDrive.setVelocity(elbowParams.maxVelocity, AngleUnit.DEGREES);
        elbowDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elbowDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // control variables
        int elbowTicksTarget = 0;

        //------------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            // update triggers
            homeTrigger.update(gamepad1.x);
            dropTrigger.update(gamepad1.y);
            driveTrigger.update(gamepad1.a);
            pickTrigger.update(gamepad1.b);

            int elbowTicksActual = elbowDrive.getCurrentPosition();

            // move commands
            if (homeTrigger.wasTriggered()) {
                elbowTicksTarget=elbowParams.homePosTicks;
                elbowDrive.setTargetPosition(elbowTicksTarget);
            }
            if (dropTrigger.wasTriggered()) {
                elbowTicksTarget=elbowParams.dropPosTicks;
                elbowDrive.setTargetPosition(elbowTicksTarget);
            }
            if (driveTrigger.wasTriggered()) {
                elbowTicksTarget=elbowParams.drivePosTicks;
                elbowDrive.setTargetPosition(elbowTicksTarget);
            }
            if (pickTrigger.wasTriggered()) {
                elbowTicksTarget=elbowParams.pickPosTicks;
                elbowDrive.setTargetPosition(elbowTicksTarget);
            }

            // set motor power
            double elbowPower = 1.0;
            elbowDrive.setPower(elbowPower);

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Use the A, B, X, and Y buttons to move the elbow.");
            telemetry.addLine("X -> Home, Y -> Drop, A -> Driving, B -> Pick");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Elbow Target: TICKS[%d]", elbowTicksTarget));
            telemetry.addLine(String.format("Elbow Status: PWR[%4.2f] TICKS[%d]", elbowPower, elbowTicksActual));
            telemetry.update();

        }

    }

}
