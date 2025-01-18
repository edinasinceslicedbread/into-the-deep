package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RisingEdgeTrigger;


@TeleOp(name = "Chassis Assembly Test", group = "Testing")
public class ChassisAssemblyTest extends LinearOpMode {

    // main wheel drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // scissor lift drive and limit sensor
    private DcMotor scissorDrive = null;
    private TouchSensor scissorLimitLoSensor = null;


    @Override
    public void runOpMode() {

        // initialize wheel drive with motor hardware names
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // assign wheel motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // scissor drive and direction
        scissorDrive = hardwareMap.get(DcMotor.class, "scissorDrive");
        scissorDrive.setDirection(DcMotor.Direction.FORWARD);

        // limit touch sensor switch
        scissorLimitLoSensor = hardwareMap.get(TouchSensor.class, "scissorLimitLo");

        // wait for start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // read current positions
            int leftFrontCounts = leftFrontDrive.getCurrentPosition();
            int leftBackCounts = leftBackDrive.getCurrentPosition();
            int rightFrontCounts = rightFrontDrive.getCurrentPosition();
            int rightBackCounts = rightBackDrive.getCurrentPosition();

            // scissor encoder and limit switch
            int scissorCounts = scissorDrive.getCurrentPosition();
            boolean scissorLimitLo = scissorLimitLoSensor.isPressed();

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

            // move scissor by pressing buttons
            double scissorPower = 0;
            if (gamepad1.right_trigger > 0.05) {
                scissorPower = gamepad1.right_trigger;
                if (scissorCounts >= 2000) {
                    scissorPower = 0;
                }
            }
            if (gamepad1.left_trigger > 0.05) {
                scissorPower = -gamepad1.left_trigger;
                if (scissorCounts <= 0) {
                    scissorPower = 0;
                }
            }
            if (scissorLimitLo) {
                scissorPower = -0.1;
            }

            // enforce limits
            if (scissorPower < -0.1) {
                scissorPower = -0.1;
            }
            if (scissorPower > 0.5) {
                scissorPower = 0.5;
            }

            scissorDrive.setPower(scissorPower);


            // update telemetry data
            telemetry.addLine(String.format("[%d]----[%d]", leftFrontCounts, rightFrontCounts));
            telemetry.addLine(String.format("[%d]----[%d]", leftBackCounts, rightBackCounts));
            telemetry.addLine(String.format("Scissor [%d] [%s]", scissorCounts, scissorLimitLo));
            telemetry.update();

            idle();

        }

    }
}
