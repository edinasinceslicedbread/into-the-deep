package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ctrl.RisingEdgeTrigger;

// RR-specific imports
import com.acmerobotics.roadrunner.ftc.Actions;


@Autonomous(name = "$$$ AUTO-A (Pusher)", group = "$$$")
public class AutoOpModeA extends LinearOpMode {

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

        // *******************************************************************************************
        // Wait for the game to start (driver presses START)
        // *******************************************************************************************
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
<<<<<<< HEAD
                =======


        //Edit Path


>>>>>>> 161b98a74b5a3d03e3cb47dbefded4897f4fe01b

        leftFrontDrive.setPower(-0.25);
        rightFrontDrive.setPower(-0.25);
        rightBackDrive.setPower(-0.25);
        leftBackDrive.setPower(-0.25);

        sleep(2000);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);


    }


}

