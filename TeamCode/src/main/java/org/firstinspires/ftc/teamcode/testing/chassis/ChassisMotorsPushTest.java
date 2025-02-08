package org.firstinspires.ftc.teamcode.testing.chassis;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TEST | Chassis | Motors Push Test", group = "$$$$ Chassis")
public class ChassisMotorsPushTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        // hardware map
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // motor directions (reverse right motors)
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

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
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            // read current positions
            int leftFrontTicksActual = leftFrontDrive.getCurrentPosition();
            int leftBackTicksActual = leftBackDrive.getCurrentPosition();
            int rightFrontTicksActual = rightFrontDrive.getCurrentPosition();
            int rightBackTicksActual = rightBackDrive.getCurrentPosition();

            // move each wheel by pressing buttons
            double leftFrontPower = gamepad1.x ? 0.25 : 0.0;    // X gamepad
            double leftBackPower = gamepad1.a ? 0.25 : 0.0;     // A gamepad
            double rightFrontPower = gamepad1.y ? 0.25 : 0.0;   // Y gamepad
            double rightBackPower = gamepad1.b ? 0.25 : 0.0;    // B gamepad

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // update telemetry data
            telemetry.addLine("Use the four buttons X,Y,A,B to drive each wheel.");
            telemetry.addLine("Encoder counters should be moving upwards.");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("PWR: [%4.2f]-<>-[%4.2f]", leftFrontPower, rightFrontPower));
            telemetry.addLine(String.format("PWR: [%4.2f]-<>-[%4.2f]", leftBackPower, rightBackPower));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("TICKS: [%d]-<>-[%d]", leftFrontTicksActual, rightFrontTicksActual));
            telemetry.addLine(String.format("TICKS: [%d]-<>-[%d]", leftBackTicksActual, rightBackTicksActual));
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine("    \\\\    \\\\");
            telemetry.addLine("     \\\\    \\\\");
            telemetry.addLine("     X-<>-Y: Front Wheels (right -<>- left\")");
            telemetry.addLine("       A-<>-B: Back Wheels (right -<>- left");
            telemetry.addLine("         \\\\    \\\\");
            telemetry.addLine("          \\\\    \\\\");
            telemetry.update();

            idle();

        }

    }
}
