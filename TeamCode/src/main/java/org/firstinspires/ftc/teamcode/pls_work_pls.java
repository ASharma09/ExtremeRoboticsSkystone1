package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name="pls work pls")
public class pls_work_pls extends LinearOpMode {

    encoderDrive encoder = new encoderDrive();

    //Robot robot = new Robot();
    DcMotor leftBackDrive = null;
    DcMotor leftFrontDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor rightFrontDrive = null;

    DcMotor topLiftMotor = null;
    DcMotor bottomLiftMotor = null;
    DcMotor armMotor = null;

    Servo chickenServo = null;
    Servo FMRight = null;
    Servo FMLeft = null;

    Servo rightClaw = null;
    Servo leftClaw = null;

    //Robot robot = new Robot();
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    public void runOpMode() throws InterruptedException
    {

        initialize();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        //resetAngle();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        resetAngle();

        waitForStart();

        //while (opModeIsActive()) {
        telemetry.addData("Mode", "running");
        telemetry.addData("angle", getAngle());
        telemetry.update();
        //}

        sleep(3000);

        double angle = getAngle();

        telemetry.addData("angle before the loops", angle);
        telemetry.update();



        sleep(2000);
        if(getAngle() > 0) {
            while (getAngle() > 0) { //if getAngle() is pos it is to the left
                //turn right
//                telemetry.addData("turning right", getAngle());
//                telemetry.update();
//                sleep(1000);
                double speed = 0.1;
                rightBackDrive.setPower(speed);
                rightFrontDrive.setPower(speed);
                leftFrontDrive.setPower(-speed);
                leftBackDrive.setPower(-speed);
                //encoder.encoderTurn(.5, 1, 3, 3);
            }
        }
        else {
            while (getAngle() < 0) {
            //turn left
//            telemetry.addData("turning left", getAngle());
//            telemetry.update();
//            sleep(1000);
            double speed = 0.1;
                rightBackDrive.setPower(-speed);
                rightFrontDrive.setPower(-speed);
                leftFrontDrive.setPower(speed);
                leftBackDrive.setPower(speed);
            //encoder.encoderTurn(.5, -1, 3, 3);

            }
        }

            stopMotors();

        telemetry.addData("I reached end of loops", getAngle());
        telemetry.update();

        sleep(3000);

    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void stopMotors() {
//        encoder.robot.leftBackDrive.setPower(0);
//        encoder.robot.leftFrontDrive.setPower(0);
//        encoder.robot.rightBackDrive.setPower(0);
//        encoder.robot.rightFrontDrive.setPower(0);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public void initialize() {
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLiftMotor = hardwareMap.get(DcMotor.class, "topLiftMotor");
        bottomLiftMotor = hardwareMap.get(DcMotor.class, "bottomLiftMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        chickenServo = hardwareMap.get(Servo.class, "chickenServo");
        FMRight = hardwareMap.get(Servo.class, "FMRight");
        FMLeft = hardwareMap.get(Servo.class, "FMLeft");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
    }
}
