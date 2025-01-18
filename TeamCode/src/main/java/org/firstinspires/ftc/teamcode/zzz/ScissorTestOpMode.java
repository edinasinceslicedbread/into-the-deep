
package org.firstinspires.ftc.teamcode.zzz;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Scissor Lift Test Program", group="z")
@Disabled
public class ScissorTestOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor scissorDrive = null;

    @Override
    public void runOpMode() {

        scissorDrive = hardwareMap.get(DcMotor.class, "scissorDrive");
        scissorDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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

            // Send calculated power to wheels
            scissorDrive.setPower(scissorDrivePower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("3764 Status", "Run Time: " + runtime.toString());
            telemetry.addData("Scissor Power", "%4.2f", scissorDrivePower);

            telemetry.update();


        }
    }}
