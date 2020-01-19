package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Extreme Robotics 2019
 */

public class Robot
{
    // Hardware map & op mode
    HardwareMap hwMap;
//    LinearOpMode opMode;
//    double drivePower = 0.9;
//    // Variables for Encoder driving
//    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andy Mark Motor Encoder
//    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
//    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);


    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBackDrive = null;
    DcMotor leftFrontDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor rightFrontDrive = null;
//    DcMotor liftMotor = null;
//    Servo leftServo = null;
//    Servo rightServo = null;
//    Servo clawServo = null;
//    Servo liftServo = null;
    Servo chickenServo = null;
    Servo FMRight = null;
    Servo FMLeft = null;

    //BNO055IMU gyro    = null;



    double[] paramA = {0, 1, -1};

    public void init(HardwareMap ProtohwMap)
    {
        hwMap = ProtohwMap;
        //opMode = linearOpMode;

        leftBackDrive = hwMap.get(DcMotor.class, "leftBackMotor");
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontMotor");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackMotor");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontMotor");
//        liftMotor = hwMap.get(DcMotor.class, "liftMotor");

        //when plugging in servos, black goes left if you're looking at it rightside up
//        rightServo = hwMap.get(Servo.class, "rightServo");
//        leftServo = hwMap.get(Servo.class, "leftServo");
//        clawServo = hwMap.get(Servo.class, "clawServo");
//        liftServo = hwMap.get(Servo.class, "liftServo");
        chickenServo = hwMap.get(Servo.class, "chickenServo");
        FMRight = hwMap.get(Servo.class, "FMRight");
        FMLeft = hwMap.get(Servo.class, "FMLeft");

        //gyro = (BNO055IMU) hwMap.gyroSensor.get("gyro");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void WaitMillis(long millis)
    {
        try
        {
            Thread.sleep(millis);
        } catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }

    //ADD LATCH FOUNDATION METHOD HERE

}
