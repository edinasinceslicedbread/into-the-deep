package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "$$$ AUTO-A (PUSH 24 INCH)", group = "$$$A")
public class AutoOpModeA2 extends LinearOpMode {

    // config variables
    double inPerTick = 0.026;
    double maxDistanceInches = 24.0;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // hardware
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        // main wheel drive motor hardware names
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // assign wheel motor directions
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // other variables
        int autoOpModeState = 0;
        double startingTicks = 0.0;
        double distance = 0.0;

        // *******************************************************************************************
        // Wait for the game to start (driver presses START)
        // *******************************************************************************************
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //------------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            double lf = leftFrontDrive.getCurrentPosition();
            double rf = rightFrontDrive.getCurrentPosition();
            double lb = leftBackDrive.getCurrentPosition();
            double rb = rightBackDrive.getCurrentPosition();
            double currentTicks = (lf + rf + lb + rb) / 4;

            //------------------------------------------------------------------------------------------------
            // state machine
            //------------------------------------------------------------------------------------------------
            switch (autoOpModeState) {

                case 0: // initial state

                    leftFrontDrive.setPower(-0.20);
                    rightFrontDrive.setPower(-0.20);
                    rightBackDrive.setPower(-0.20);
                    leftBackDrive.setPower(-0.20);
                    startingTicks = currentTicks;
                    autoOpModeState = 1;
                    break;

                case 1: // wait for distance reached

                    distance = (startingTicks - currentTicks) * inPerTick;
                    if (distance > maxDistanceInches) {
                        leftFrontDrive.setPower(0.0);
                        rightFrontDrive.setPower(0.0);
                        leftBackDrive.setPower(0.0);
                        rightBackDrive.setPower(0.0);
                    }

            }

            //------------------------------------------------------------------------------------------------
            // Update Telemetry
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Distance", distance);
            telemetry.addData("Auto OpMode State", autoOpModeState);
            telemetry.update();

        }

    }

}
