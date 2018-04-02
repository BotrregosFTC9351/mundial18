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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vuforia Auto.", group="Gaia")
@Disabled
public class DevoltColor extends LinearOpMode {


    /* Declare OpMode members. */
    DevoltHard robot = new DevoltHard() ;   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1220 ; // Andymark N everest 60 motor
    static final double DRIVE_GEAR_REDUCTION = 0.2
            ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.8 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159265358979323846);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;

    int clr=3;

    public void setClr(int value) {
        clr = value;
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap); //Inicia el HardwareMap

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Espera a que le des play
        waitForStart();

        // Baja el sensor a las pelotas ( ͡° ͜ʖ ͡°)
        robot.claw3.setPosition(1);
        cerrar();

        // Espera por precaución
        sleep(800);

        // Actualiza el sensor y determina si la pelota es azul o roja
        robot.colors = robot.colorSensor.getNormalizedColors();

        double vel = 0.01;
        int time = 500;

        //If de color (?)
        if (robot.colors.red > robot.colors.blue) {
            telemetry.addData("Ball Color", "Red");

            encoderDrive(TURN_SPEED,10, -10, .1);
            sleep(200);
            robot.claw3.setPosition(0.02);
            sleep(200);
            encoderDrive(TURN_SPEED,-10,10,.1);

        } else {
            telemetry.addData("Ball Color", "Blue");

            encoderDrive(TURN_SPEED,-10, 10, .1);
            sleep(200);
            robot.claw3.setPosition(.02);
            sleep(200);
            encoderDrive(TURN_SPEED,10, -10, .1);
        }

        telemetry.addData("Status: ","Gema derribada");
        telemetry.update();
        //Gira para ver la imagen
        encoderDrive(.2,10,-10,.4);
        encoderDrive(.2,10,10,.2);
//Center=2 Left=0 Right=1
        //Va al centro
        if (clr==2){


        }
        //Va a la izquierda
        else if (clr==0){

        }
        //Va a la derecha
        else if (clr==1){

        }
        //Se estaciona (falla vuforia)
        else{
            encoderDrive(DRIVE_SPEED,10,10,.7);
            telemetry.addData("No se allo la imagen", "error");
        }


    }

    public void moverElevador(double timeoutS, double speed) {
        robot.elevator.setPower(speed);
        runtime.reset();
        sleep((long) timeoutS * 1000);
        robot.elevator.setPower(0);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.left.setTargetPosition(newLeftTarget);
            robot.right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            runtime.reset();
            robot.left.setPower(Math.abs(speed));
            robot.right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left.isBusy() && robot.right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left.getCurrentPosition(),
                        robot.right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.left.setPower(0);
            robot.right.setPower(0);

            // Turn off RUN_TO_POSITIONz
            robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // pause after each move
        }
    }

    public void abrir () {
        robot.claw1.setPosition(0.02);
        robot.claw2.setPosition(0.8);
    }
    public void cerrar () {
        robot.claw2.setPosition(.3);
        robot.claw1.setPosition(.7);

    }
}