/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OmniDrive", group="Drive")
//@Disabled
public class OmniDrive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareOmni   robot           = new HardwareOmni();              // Use a K9'shardware
    HardwareCosas hws = new HardwareCosas();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        hws.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Good", "Luck Koke");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Mecanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad2.right_bumper){
                hws.CA.setPower(-1);
            } else if (gamepad2.left_bumper){
                hws.CA.setPower(1);
            } else {
                hws.CA.setPower(0);
            }

            if (gamepad2.a){
                hws.CE.setPower(1);
            } else if (gamepad2.y){
                hws.CE.setPower(-1);
            } else {
                hws.CE.setPower(0);
            }

            if (gamepad1.right_bumper){
                hws.RH.setPosition(0);
            } else if (gamepad1.left_bumper){
                hws.RH.setPosition(1);
            }
            telemetry.addData("RH: %f",hws.RH.getPosition());    //

            if (gamepad1.y){
                hws.RM.setPosition(0);
            } else if (gamepad1.a){
                hws.RM.setPosition(1);
            }
            telemetry.addData("RM: %f", hws.RM.getPosition());    //

            if (gamepad1.x){
                hws.PC.setPower(1);
            } else if (gamepad1.b){
                hws.PC.setPower(-1);
            } else {
                hws.PC.setPower(0);
            }

            if (gamepad2.dpad_up){
                hws.JW.setPosition(0);
            } else if (gamepad2.dpad_down){
                hws.JW.setPosition(1);
            }
            telemetry.addData("JW: %f", hws.JW.getPosition());    //


            telemetry.update();

        }

    }
    public void Mecanum (double x, double y, double rotation){
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = -x + y - rotation;
        wheelSpeeds[1] = x + y + rotation;
        wheelSpeeds[2] = x + y - rotation;
        wheelSpeeds[3] = -x + y + rotation;

        robot.normalize(wheelSpeeds);

        robot.frontLeft.setPower(wheelSpeeds[0]);
        robot.frontRight.setPower(wheelSpeeds[1]);
        robot.backLeft.setPower(wheelSpeeds[2]);
        robot.backRight.setPower(wheelSpeeds[3]);
    }
}
