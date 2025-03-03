package org.firstinspires.ftc.teamcode.competition;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ctrl.ArmController;
import org.firstinspires.ftc.teamcode.ctrl.ArmControllerV2;

@Autonomous(name = "$$$ AUTO (TEMPLATE)", group = "$$$")
@Disabled
public class AutoModeTemplate extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------

    // chassis
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // scissor
    private DcMotorEx scissorDrive = null;
    private TouchSensor scissorLoSensor = null;

    // claw
    private Servo clawServo = null;

    // arm
    private ArmControllerV2 armController = new ArmControllerV2();

    double clawOpenPosition = 0.50;
    double clawClosePosition = 0.05;

    @SuppressLint("DefaultLocale")
    @Override

    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------

        // chassis
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // scissor
        scissorDrive = hardwareMap.get(DcMotorEx.class, "scissorDrive");
        scissorLoSensor = hardwareMap.get(TouchSensor.class, "scissorLoSensor");

        // claw
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // arm
        armController.initialize(hardwareMap);

        //------------------------------------------------------------------------------------------------
        // Advanced Setup
        //------------------------------------------------------------------------------------------------

        // chassis motor directions (reverse right motors)
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // other variables
        int autoOpModeState = 0;
        int armAutoCmd = 0;

        //------------------------------------------------------------------------------------------------
        // Start Button
        //------------------------------------------------------------------------------------------------

        // update some telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // arm reset
        armController.reset();

        //------------------------------------------------------------------------------------------------
        // Run until the end of the match (driver presses STOP)
        //------------------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            //------------------------------------------------------------------------------------------------
            // state machine
            //------------------------------------------------------------------------------------------------
            switch (autoOpModeState) {
                case 0: // initial state

                    // close claw
                    clawServo.setPosition(clawClosePosition);

                    // wait a couple seconds, move to next state
                    if (runtime.seconds() > 1.0) {
                        armAutoCmd = 0;
//                        leftFrontDrive.setPower(-0.30);
//                        rightFrontDrive.setPower(-0.30);
//                        rightBackDrive.setPower(-0.30);
//                        leftBackDrive.setPower(-0.30);
//                        leftFrontDrive.setPower(0);
//                        rightFrontDrive.setPower(0);
//                        rightBackDrive.setPower(0);
//                        leftBackDrive.setPower(0);
                        // reset command
                        autoOpModeState = 1;
                    }
                    break;

                case 1: // move arm to basket position

                    armAutoCmd = 2;
                    if (armController.atBasketPos) {
                        armAutoCmd = 0; // reset command
                        autoOpModeState = 2;
                    }
                    break;

                case 2: // move scissor up

                    // power scissor drive up until position reached
                    scissorDrive.setPower(0.5);
                    if (scissorDrive.getCurrentPosition() > 2000) {
                        scissorDrive.setPower(0.0);
                        autoOpModeState = 3;
                    }
                    break;

                case 3: // open claw

                    // close claw and wait a couple seconds
                    clawServo.setPosition(clawOpenPosition);
                    if (runtime.seconds() > 1.0) {
                        autoOpModeState = 4;
                    }
                    break;

                case 4: // move both arm and scissor to home position

                    // scissor
                    scissorDrive.setPower(-0.25);
                    if (scissorLoSensor.isPressed()) {
                        scissorDrive.setPower(0.0);
                    }

                    // arm
                    armAutoCmd = 1;
                    if (armController.atHomePos && scissorLoSensor.isPressed()) {
                        armAutoCmd = 0; // reset command
                        autoOpModeState = 5;
                    }
                    break;

            }

            //------------------------------------------------------------------------------------------------
            // run arm controller every cycle
            //------------------------------------------------------------------------------------------------
            armController.update(runtime, gamepad1, gamepad2, armAutoCmd);

            //------------------------------------------------------------------------------------------------
            // Update Telemetry
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Auto OpMode State", autoOpModeState);
            telemetry.update();

        }

    }

}
