package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
//import org.firstinspires.ftc.teamcode.mechanisms.Camera;
//import org.firstinspires.ftc.teamcode.opencv.SignalColor;


@Autonomous(name = "Auto")
public class ParkAuto extends LinearOpMode {

    /**
     * Amount of time elapsed
     */

    private ElapsedTime time = new ElapsedTime();

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private BNO055IMU imu = null;

    Thread telemetryHandler = new Thread(){
        @Override
        public void run() {
            while (opModeIsActive()) {
                telemetry.addData("Runtime(s): ", runtime.seconds());
                telemetry.update();
            }
        }
    };


    @Override
    public void runOpMode() {


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

        waitForStart();
        telemetryHandler.start();
        runtime.reset();
        park(runtime);
        sleep(5000);
    }


    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void park(ElapsedTime timer){
        double startTime = timer.milliseconds();
        backLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backRight.setPower(0.5);
        frontLeft.setPower(-0.5);
        //noinspection StatementWithEmptyBody
        while(timer.milliseconds() < startTime + 1500 && opModeIsActive()){}
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
    }

}