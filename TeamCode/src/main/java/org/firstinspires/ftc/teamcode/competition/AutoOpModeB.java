package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ctrl.ArmController;
import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;
import org.firstinspires.ftc.teamcode.testing.arm.ArmControllerAuto;


@Autonomous(name = "$$$ AUTO-B (BETA)", group = "$$$")
@Disabled
public class AutoOpModeB extends LinearOpMode {
    //Source code from ArmCtrlAuto
    ArmControllerAuto armControllerAuto = new ArmControllerAuto();
    // scissor lift constants
    static final double SCISSOR_MIN_POS = 1000;    // Minimum scissor lift encoder position
    static final double SCISSOR_MAX_POS = 10000;     // Maximum scissor lift encoder position

    // claw gripper constants
    static final int CYCLE_MS = 50;             // period of each cycle
    static final double CLAW_MIN_POS = 0.0;     // Minimum rotational position
    static final double CLAW_MAX_POS = 1.0;     // Maximum rotational position

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // main wheel drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // scissor lift drive, extension drive, and claw servo
    private DcMotor scissorDrive = null;
    private Servo clawServo = null;

    int autoUpdateVariable;

    // digital limit switches
    // TODO: uncomment if touch sensors are added
    // private TouchSensor scissorLimitLo = null;
    // private TouchSensor scissorLimitHi = null;
    // private TouchSensor extensionLimitBwd = null;
    // private TouchSensor extensionLimitFwd = null;

   ArmController armcontroller = new ArmController();

    @Override
    public void runOpMode() {

        // *******************************************************************************************
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // *******************************************************************************************

        // main wheel drive motor hardware names
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // scissor drive, claw server, and extend / retract
        scissorDrive = hardwareMap.get(DcMotor.class, "scissorDrive");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // digital limit switches
        // TODO: uncomment if touch sensors are added
        // scissorLimitLo = hardwareMap.get(DigitalChannel.class, "scissorLoSensor");
        // scissorLimitHi = hardwareMap.get(DigitalChannel.class, "scissorHiSensor");
        // extensionLimitBwd = hardwareMap.get(DigitalChannel.class, "extensionLimitBwd");
        // extensionLimitFwd = hardwareMap.get(DigitalChannel.class, "extensionLimitFwd");

        // assign wheel motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // assign scissor, extension, and claw directions
        scissorDrive.setDirection(DcMotor.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // outside the while loop, set initial claw servo position
        double clawServoPosition = (CLAW_MAX_POS - CLAW_MIN_POS) / 2.0; // Start at half position

        // outside the while loop, set homing mode false
        boolean homingModeActive = false;

        // *******************************************************************************************
        // Wait for the game to start (driver presses START)
        // *******************************************************************************************
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //************************************************************************************************
        //Autonomous Test Code B
        //************************************************************************************************
        //TODO; Redo program. 1Pick Up Block 2Use top chasis for angle 3scissor lift 4drive forward and drop block in to basket while staying parralel to wall.
        //TODO; Scissor lift and clamp claw at start - add back and forth for top after turn.

        // close claw
        clawServo.setPosition(0.05);

        armcontroller.update(new ElapsedTime(), gamepad1, gamepad2, 1);
        sleep(2000);

        // raise scissor lift
        scissorDrive.setPower(1.0);
        leftFrontDrive.setPower(-0.3);
        rightFrontDrive.setPower(-0.3);
        leftBackDrive.setPower(-0.3);
        rightBackDrive.setPower(-0.3);
        sleep(2500);            // TOTO: raise lift time
        scissorDrive.setPower(0.0);
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        sleep(1000);
        int autoUpdateVariable = 2;

        // TODO: retract time
        sleep(1500);

        // drive robot forward
        leftFrontDrive.setPower(-0.1);
        rightFrontDrive.setPower(-0.1);
        leftBackDrive.setPower(-0.1);
        rightBackDrive.setPower(-0.1);
        sleep(300);            // TOTO: drive forward time
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        sleep(1000);

        // open claw

        clawServo.setPosition(0.30);
        sleep(5000);


    }
}
