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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@Autonomous(name="VUFORIACUBOS-B", group ="Concept")
public class VUFORIACUBOS_B extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    HardwareOmni         robot   = new HardwareOmni();   // Use a Pushbot's hardware
    HardwareCosas cosas = new HardwareCosas();

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Andymark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .7;
    static final double     TURN_SPEED              = 0.5;

    OpenGLMatrix lastLocation = null;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AejTuGz/////AAAAmdZRjZEcgUHshHAYuhaOvWZCO2QXzSdYmf3Xjfz/Axpr6iIN7krl+sSzU/Kba24jaF51aXfZpOKxozk6xNgX/2M5V1iXAZH4C9EsbIsvYImhLq+OXc89yWUzROyEgP8zpgkMbBPxE8IUUY7UNLavSTy55KIYyyl+Q/qHvOFL2iyGVx4VhkbZ50+bk0b4LsVOQhifgbIDoJm0dSTwKC2bDfv3GYEcJtyMuZVBa8if4zwc6Nlz4kOoOaIW7pGU5e4danjpAqIuoUoGHzUF0rYuSfM3RfUKyBcIPcTCRXsjuQKe0Yv14wQav6o1yTr/7HEKMhZTTsVwXjfohOvYLwbHTsmU8/Ww6JliCUccO8LGG6Ua";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        robot.init(hardwareMap);
        cosas.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cosas.CE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cosas.CE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at FL:%7d FR:%7d BL:%7d BR:%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition());

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;


        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.update();

        waitForStart();

        relicTrackables.activate();

        cosas.JW.setPosition(0);
        sleep (1000);


        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
            if (vuMark== RelicRecoveryVuMark.CENTER){
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", sensorColor.alpha());
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });

                if(sensorColor.blue()<sensorColor.red()){
                    telemetry.addData("Normal", "");
                    cerrarCubos(-.5);
                    encoderElevador(DRIVE_SPEED, -6, 1.0);
                    sleep(100);
                    encoderDrive(DRIVE_SPEED,  -25,  -25, -25, -25, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    cosas.JW.setPosition(.7);
                    sleep(100);
                    encoderDrive(DRIVE_SPEED,  -18,  18, -18, 18, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -14.7,  -14.7, -14.7, -14.7, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  18,  -18, 18, -18, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -3,  -3, -3, -3, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderElevador(DRIVE_SPEED, 6, 1.0);
                    cerrarCubos(.5);
                    sleep(500);
                    cerrarCubos(0);
                    encoderDrive(DRIVE_SPEED,  -6,  -6, -6, -6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  6,  6, 6, 6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -36,  36, -36, 36, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    sleep(20000);

                } else {
                    telemetry.addData("Adelante", "");
                    cerrarCubos(-.7);
                    sleep(100);
                    cosas.RH.setPosition(0);
                    encoderElevador(.4, -6, 1.0);
                    encoderDrive(.2,  4,  -4, 4, -4, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    cosas.JW.setPosition(.7);
                    sleep(100);
                    encoderDrive(.2,  -4,  4, -4, 4, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -25,  -25, -25, -25, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -18,  18, -18, 18, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -14.3,  -14.3, -14.3, -14.3, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  17.5,  -17.5, 17.5, -17.5, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -3,  -3, -3, -3, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderElevador(DRIVE_SPEED, 6, 1.0);
                    cerrarCubos(.5);
                    sleep(500);
                    cerrarCubos(0);
                    encoderDrive(DRIVE_SPEED,  -6,  -6, -6, -6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  6,  6, 6, 6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -36,  36, -36, 36, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    sleep(20000);

                }


                telemetry.addData("Dejar cubo en la der ", "Funcion del otro autonomo");
            } else if (vuMark== RelicRecoveryVuMark.RIGHT){
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", sensorColor.alpha());
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });

                if(sensorColor.blue()<sensorColor.red()){
                    telemetry.addData("Normal", "");
                    cerrarCubos(-.5);
                    encoderElevador(DRIVE_SPEED, -6, 1.0);
                    sleep(100);
                    encoderDrive(DRIVE_SPEED,  -25,  -25, -25, -25, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    cosas.JW.setPosition(.7);
                    sleep(100);
                    encoderDrive(DRIVE_SPEED,  -19,  19, -19, 19, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -22,  -22, -22, -22, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  18,  -18, 18, -18, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -3,  -3, -3, -3, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderElevador(DRIVE_SPEED, 6, 1.0);
                    cerrarCubos(.5);
                    sleep(500);
                    cerrarCubos(0);
                    encoderDrive(DRIVE_SPEED,  -6,  -6, -6, -6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  6,  6, 6, 6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -36,  36, -36, 36, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    sleep(20000);

                } else {
                    telemetry.addData("Adelante", "");
                    cerrarCubos(-.7);
                    sleep(100);
                    cosas.RH.setPosition(0);
                    encoderElevador(.4, -6, 1.0);
                    encoderDrive(.2,  4,  -4, 4, -4, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    cosas.JW.setPosition(.7);
                    sleep(100);
                    encoderDrive(.2,  -4,  4, -4, 4, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -25,  -25, -25, -25, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -18,  18, -18, 18, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -23,  -23, -23, -23, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  17.5,  -17.5, 17.5, -17.5, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -3,  -3, -3, -3, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderElevador(DRIVE_SPEED, 6, 1.0);
                    cerrarCubos(.5);
                    sleep(500);
                    cerrarCubos(0);
                    encoderDrive(DRIVE_SPEED,  -6,  -6, -6, -6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  6,  6, 6, 6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -36,  36, -36, 36, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    sleep(20000);


                }


                telemetry.addData("Dejar cubo en la RIGHT ", "Funcion del otro autonomo");



            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);

                // send the info back to driver station using telemetry function.
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", sensorColor.alpha());
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                // change the background color to match the color detected by the RGB sensor.
                // pass a reference to the hue, saturation, and value array as an argument
                // to the HSVToColor method.
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    }
                });

                if(sensorColor.blue()<sensorColor.red()){
                    telemetry.addData("Normal", "");
                    cerrarCubos(-.5);
                    encoderElevador(DRIVE_SPEED, -6, 1.0);
                    encoderDrive(DRIVE_SPEED,  -25,  -25, -25, -25, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    cosas.JW.setPosition(.7);
                    sleep(100);
                    encoderDrive(DRIVE_SPEED,  -8,  8, 8, -8, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -3,  -3, -3, -3, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderElevador(DRIVE_SPEED, 6, 1.0);
                    cerrarCubos(.5);
                    sleep(500);
                    cerrarCubos(0);
                    encoderDrive(DRIVE_SPEED,  -6,  -6, -6, -6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  6,  6, 6, 6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -36,  36, -36, 36, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  10,  -10, -10, 10, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    sleep(20000);
                } else {
                    telemetry.addData("Adelante", "");
                    cerrarCubos(-.7);
                    sleep(100);
                    cosas.RH.setPosition(0);
                    encoderElevador(.4, -6, 1.0);
                    encoderDrive(.2,  5,  -5, 5, -5, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    cosas.JW.setPosition(.7);
                    sleep(100);
                    encoderDrive(.2,  -4,  4, -4, 4, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -25,  -25, -25, -25, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -10,  10, 10, -10, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -3,  -3, -3, -3, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderElevador(DRIVE_SPEED, 6, 1.0);
                    cerrarCubos(.5);
                    sleep(500);
                    cerrarCubos(0);
                    encoderDrive(DRIVE_SPEED,  -6,  -6, -6, -6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  6,  6, 6, 6, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  -36,  36, -36, 36, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(DRIVE_SPEED,  10,  -10, -10, 10, 5.0 );  // S1: Forward 47 Inches with 5 Sec timeout
                    sleep(20000);

                }


                telemetry.addData("Dejar cubo en la LEFT ", "Funcion del otro autonomo");
            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public void cerrarCubos (double velocidad){
        cosas.CA.setPower(velocidad);
    }




    public void encoderElevador (double speed, double inches, double timeoutS){
        int newTarjet;
        if (opModeIsActive()){
            newTarjet = cosas.CE.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            cosas.CE.setTargetPosition(newTarjet);

            cosas.CE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            cosas.CE.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (cosas.CE.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to:%7d", newTarjet);
                telemetry.addData("Path2",  "Running at: %7d",
                        cosas.CE.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            cosas.CE.setPower(0);

            // Turn off RUN_TO_POSITION
            cosas.CE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }



    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to FL:%7d FR:%7d BL:%7d BR:%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at FL:%7d FR:%7d BL:%7d BR: %7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

}
