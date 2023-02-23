package org.firstinspires.ftc.teamcode;
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

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that rus in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Layer Cake", group = "Iterative Opmode")

public class Iterative_Opmode_V_2 extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor spin = null;
    private DcMotor slides = null;
    private DcMotor intake = null;
    private DistanceSensor distLeft = null;
    private DistanceSensor distRight = null;
    private DistanceSensor distBack = null;
    private DigitalChannel magSwitch = null;
    private BNO055IMU imu = null;
    private Servo boxDoor = null;
    private double boxTest = 0.5;
    private double ROTATION_SENSITIVITY = 0.5;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");

        //initialize the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //set zero behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //reset encoders for all the motors
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize distance sensors
        //distLeft = hardwareMap.get(DistanceSensor.class, "distLeft");
        //distRight = hardwareMap.get(DistanceSensor.class, "distRight");
        //distBack = hardwareMap.get(DistanceSensor.class, "distBack");
        //magSwitch = hardwareMap.get(DigitalChannel.class, "mag");
        //magSwitch.setMode(DigitalChannel.Mode.INPUT);
        //boxDoor = hardwareMap.get(Servo.class, "boxDoor");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Get the positions of the left stick in terms of x and y
        //Invert y because of the input from the controller
        double stickX = Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x;
        double stickY = Math.abs(gamepad1.left_stick_y) < 0.1 ? 0 : -gamepad1.left_stick_y;
        //get the direction from the IMU
        double angle = imu.getAngularOrientation().firstAngle;
        //rotate the positions to prep for wheel powers
        double rotatedX = (stickX * Math.cos(PI / 4 - angle)) - (stickY * Math.sin(PI / 4 - angle));
        double rotatedY = (stickY * Math.cos(PI / 4 - angle)) + (stickX * Math.sin(PI / 4 - angle));

        if (gamepad1.b) {
            ROTATION_SENSITIVITY = .25;
        } else {
            ROTATION_SENSITIVITY = .5;
        }
        //determine how much the robot should turn
        double rotation = gamepad1.left_trigger * ROTATION_SENSITIVITY - gamepad1.right_trigger * ROTATION_SENSITIVITY;
        //test if the robot should move
        boolean areTriggersDown = Math.abs(rotation) > 0.1;
        boolean areSticksMoved = Math.sqrt((stickX * stickX) + (stickY * stickY)) > 0.1;


        if (areSticksMoved || areTriggersDown) {
            //add the rotation to the powers of the wheels
            double flPower = -rotatedY + rotation;
            double brPower = rotatedY + rotation;
            double frPower = -rotatedX + rotation;
            double blPower = rotatedX + rotation;
            //keep the powers proportional and within a range of -1 to 1
//            double motorMax = Math.max(Math.max(Math.abs(flPower), Math.abs(brPower)), Math.max(Math.abs(frPower), Math.abs(blPower)));
//            double proportion = Math.max(1, motorMax);
            frontLeft.setPower(flPower);
            backRight.setPower(brPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
        } else {
            stopDrive();
        }

        //rotate to zero
        if (gamepad1.triangle) {
            rotateToZero(0);
        }

        //telemetry
        telemetry.addData("FL: ", frontLeft.getCurrentPosition());
        telemetry.addData("FR: ", frontRight.getCurrentPosition());
        telemetry.addData("BL: ", backLeft.getCurrentPosition());
        telemetry.addData("BR: ", backRight.getCurrentPosition());
        telemetry.addData("Angle: ", imu.getAngularOrientation().firstAngle);
        telemetry.addData("RIGHT TRIGGER: ", gamepad1.right_trigger);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    private void rotateToZero(double angle) {
        //use a PID control loop to
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double k_p = Math.PI/8;
        double k_i = 0;
        double k_d = 2;
        double current_error = imu.getAngularOrientation().firstAngle - angle;
        double previous_error = current_error;
        double previous_time = 0;
        double current_time = 0;
        double max_i = 0.1;
        //while(timer.seconds() < 5){
        while (Math.abs(imu.getAngularOrientation().firstAngle - angle) > Math.PI/32) {
            current_time = runtime.milliseconds();
            current_error = angle - imu.getAngularOrientation().firstAngle;
            double p = k_p * current_error;
            double i = k_i * (current_error * (current_time - previous_time));
            i = Range.clip(i, -max_i, max_i);
            double d = k_d * ((current_error - previous_error) / (current_time - previous_time));
            double power = p + i + d;
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

            previous_error = current_error;
            previous_time = current_time;

            telemetry.addData("status:", "zeroing direction");
            telemetry.addData("time(ms):", timer.milliseconds());
            telemetry.update();
        }

        stopDrive();
    }
    @Override
    public void stop() {
    }

}