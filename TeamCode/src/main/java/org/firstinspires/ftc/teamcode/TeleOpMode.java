package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Basic: Dog6d7d Linear OpMode", group="Linear OpModeTwo")
public class TeleOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor scissorDrive = null;
    private Servo   servo = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        scissorDrive = hardwareMap.get(DcMotor.class, "scissorDrive");
        servo = hardwareMap.get(Servo.class, "clawServo");
        double  position = 0;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            double Slowmax =0.25;
            if (gamepad1.dpad_up) {
                leftFrontPower  = Slowmax;
                rightFrontPower = Slowmax;
                leftBackPower   = Slowmax;
                rightBackPower  = Slowmax;
            } if (gamepad1.dpad_down) {
                leftFrontPower = -Slowmax;
                rightFrontPower = -Slowmax;
                leftBackPower = -Slowmax;
                rightBackPower = -Slowmax;
            }if (gamepad1.dpad_left) {
                leftFrontPower  = -Slowmax;
                rightFrontPower = -Slowmax;
                leftBackPower   = Slowmax;
                rightBackPower  = Slowmax;
            } if (gamepad1.dpad_right) {
                leftFrontPower  = Slowmax;
                rightFrontPower = Slowmax;
                leftBackPower   = -Slowmax;
                rightBackPower  = -Slowmax;
            }

            /* drive directions */
            double scissorDrivePower = 0;
            if (gamepad1.right_trigger > 0)
            {
                scissorDrivePower = 0.25;
            }
            if (gamepad1.left_trigger > 0)
            {
                scissorDrivePower = -0.25;
            }
            scissorDrive.setPower(scissorDrivePower);


            // Define class members

            if (gamepad1.y = true)
            {
                position = 1.0;
            }
            if (gamepad1.b = true)
            {
                position = 0.0;
            }
            servo.setPosition(position);
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /* drive directions
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("3764 Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front L/R", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  L/R", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.update();


        }
    }
}
