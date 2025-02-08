package org.firstinspires.ftc.teamcode.testing.claw;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TEST | Claw | Color Sensor Test", group = "$$$$ Claw")
public class ClawColorSensorTest extends LinearOpMode {

    // elapsed time
    private final ElapsedTime runtime = new ElapsedTime();

    //------------------------------------------------------------------------------------------------
    // Hardware Definitions
    //------------------------------------------------------------------------------------------------
    private ColorRangeSensor colorSensor;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //------------------------------------------------------------------------------------------------
        // Hardware Setup
        //------------------------------------------------------------------------------------------------
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");


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

            // color sensor data
            int blue = colorSensor.blue();
            int red = colorSensor.red();
            int green = colorSensor.green();

            int detectedColor = 0;
            // blue is 1, red is 2, green is 3
            // All number data for the if statement can be replaced with a different value for a set value depending on distance from block.
            if (blue > red && blue > green && blue > 25) {
                detectedColor = 1;
            } else if (red > green && red > blue && red > 40) {
                detectedColor = 2;
            } else if (red > 60 && green > 50 && blue < 40) {
                detectedColor = 4;
            }

            //------------------------------------------------------------------------------------------------
            // Telemetry Data
            //------------------------------------------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addLine("Hold a game piece under the sensor to test.");
            telemetry.addLine("Red, green, and blue values are read by the sensor.");
            telemetry.addLine("A color state of red, blue, green or yellow is determined.");
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine(String.format("Red=%d, Blue=%d, Green=%d", red, green, blue));
            telemetry.addData("Detected Color State", detectedColor);
            telemetry.addLine("--------------------------------------------------");
            telemetry.addData("Red", detectedColor == 2);
            telemetry.addData("Blue", detectedColor == 1);
            telemetry.addData("Green", detectedColor == 3);
            telemetry.addData("Yellow", detectedColor == 4);
            telemetry.update();


        }
    }
}
