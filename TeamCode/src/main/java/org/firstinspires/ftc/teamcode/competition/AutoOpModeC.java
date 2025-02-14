package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;


@Autonomous(name = "$$$ AUTO-RR (Pusher)", group = "$$$")
@Disabled
public class AutoOpModeC extends LinearOpMode {

    // scissor lift constants
    static final double SCISSOR_MIN_POS = 1000;    // Minimum scissor lift encoder position
    static final double SCISSOR_MAX_POS = 10000;     // Maximum scissor lift encoder position
     // Maximum claw extension encoder position

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

    // digital limit switches
    // TODO: uncomment if touch sensors are added
    // private TouchSensor scissorLimitLo = null;
    // private TouchSensor scissorLimitHi = null;
    // private TouchSensor extensionLimitBwd = null;
    // private TouchSensor extensionLimitFwd = null;

    private RisingEdgeTrigger homingTrigger = new RisingEdgeTrigger();

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

        // *******************************************************************************************
        // Wait for the game to start (driver presses START)
        // *******************************************************************************************
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        Pose2d initialPose = new Pose2d(0, 60, 180);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)

                //Edit Path
                .lineToX(56)
                .waitSeconds(0.001)
                .lineToX(38)
                .waitSeconds(0.001)
                .turn(Math.toRadians(90))
                .waitSeconds(0.001)
                .lineToY(10)
                .waitSeconds(0.001)

                .strafeTo(new Vector2d(47, 10))
                .strafeTo(new Vector2d(47, 57))
                .strafeTo(new Vector2d(47, 10))

                .strafeTo(new Vector2d(55, 10))
                .strafeTo(new Vector2d(55, 57))
                .strafeTo(new Vector2d(55, 10))

                .strafeTo(new Vector2d(61, 10))
                .strafeTo(new Vector2d(62, 57))
                .strafeTo(new Vector2d(62, 38))

                .strafeTo(new Vector2d(-60, 38))
                .strafeTo(new Vector2d(-60, 58));

        Actions.runBlocking(tab1.build());




    }


}

