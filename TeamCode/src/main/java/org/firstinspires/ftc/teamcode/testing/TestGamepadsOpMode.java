/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test: Gamepad 1 & 2 Buttons", group = "Test")
public class TestGamepadsOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Gamepad 1
            telemetry.addData("Gamepad 1 - Left Stick Btn", gamepad1.left_stick_button);
            telemetry.addData("Gamepad 1 - Right Stick Btn", gamepad1.right_stick_button);
            telemetry.addData("Gamepad 1 - Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 - Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Gamepad 1 - Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Gamepad 1 - Right Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Gamepad 1 - A", gamepad1.a);
            telemetry.addData("Gamepad 1 - B", gamepad1.b);
            telemetry.addData("Gamepad 1 - X", gamepad1.x);
            telemetry.addData("Gamepad 1 - Y", gamepad1.y);
            telemetry.addData("Gamepad 1 - Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Gamepad 1 - Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Gamepad 1 - Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Gamepad 1 - Right Trigger", gamepad1.right_trigger);
            telemetry.addData("Gamepad 1 - Dpad Up", gamepad1.dpad_up);
            telemetry.addData("Gamepad 1 - Dpad Down", gamepad1.dpad_down);
            telemetry.addData("Gamepad 1 - Dpad Left", gamepad1.dpad_left);
            telemetry.addData("Gamepad 1 - Dpad Right", gamepad1.dpad_right);
            telemetry.addData("Gamepad 1 - Start", gamepad1.start);
            telemetry.addData("Gamepad 1 - Back", gamepad1.back);
            telemetry.addData("Gamepad 1 - Guide", gamepad1.guide);
            // Gamepad 2
            telemetry.addData("Gamepad 2 - Left Stick Btn", gamepad2.left_stick_button);
            telemetry.addData("Gamepad 2 - Right Stick Btn", gamepad2.right_stick_button);
            telemetry.addData("Gamepad 2 - Left Stick X", gamepad2.left_stick_x);
            telemetry.addData("Gamepad 2 - Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Gamepad 2 - Right Stick X", gamepad2.right_stick_x);
            telemetry.addData("Gamepad 2 - Right Stick Y", gamepad2.right_stick_y);
            telemetry.addData("Gamepad 2 - A", gamepad2.a);
            telemetry.addData("Gamepad 2 - B", gamepad2.b);
            telemetry.addData("Gamepad 2 - X", gamepad2.x);
            telemetry.addData("Gamepad 2 - Y", gamepad2.y);
            telemetry.addData("Gamepad 2 - Left Bumper", gamepad2.left_bumper);
            telemetry.addData("Gamepad 2 - Right Bumper", gamepad2.right_bumper);
            telemetry.addData("Gamepad 2 - Left Trigger", gamepad2.left_trigger);
            telemetry.addData("Gamepad 2 - Right Trigger", gamepad2.right_trigger);
            telemetry.addData("Gamepad 2 - Dpad Up", gamepad2.dpad_up);
            telemetry.addData("Gamepad 2 - Dpad Down", gamepad2.dpad_down);
            telemetry.addData("Gamepad 2 - Dpad Left", gamepad2.dpad_left);
            telemetry.addData("Gamepad 2 - Dpad Right", gamepad2.dpad_right);
            telemetry.addData("Gamepad 2 - Start", gamepad2.start);
            telemetry.addData("Gamepad 2 - Back", gamepad2.back);
            telemetry.addData("Gamepad 2 - Guide", gamepad2.guide);
            telemetry.update();
        }
    }
}
