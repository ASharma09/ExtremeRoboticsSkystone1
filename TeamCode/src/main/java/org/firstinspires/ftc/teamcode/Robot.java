package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode;

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
    LinearOpMode opMode;
//    double drivePower = 0.9;
//    // Variables for Encoder driving
//    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andy Mark Motor Encoder
//    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
//    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);


    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBackDrive;
    DcMotor leftFrontDrive;
    DcMotor rightBackDrive;
    DcMotor rightFrontDrive;
    DcMotor cascadingMotor;
    DcMotor angleMotor;
    //Servo leftServo;
    //Servo rightServo;



    double[] paramA = {0, 1, -1};

    public void init(HardwareMap ProtohwMap, LinearOpMode linearOpMode)
    {
        hwMap = ProtohwMap;
        opMode = linearOpMode;

        // Wheels
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackMotor");
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontMotor");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackMotor");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontMotor");
        cascadingMotor = hwMap.get(DcMotor.class, "cascadingMotor");
        angleMotor = hwMap.get(DcMotor.class, "angleMotor");
        //leftServo = hwMap.get(Servo.class, "leftServo");
        //rightServo = hwMap.get(Servo.class, "rightServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        cascadingMotor.setDirection(DcMotor.Direction.FORWARD);
        angleMotor.setDirection(DcMotor.Direction.FORWARD);

        // Arm
      /*  actuatorMotor = hwMap.dcMotor.get("actuatorMotor");
        omniMotor = hwMap.dcMotor.get("omniMotor");
        cascadingMotor = hwMap.dcMotor.get("cascadingMotor");
        bootMotor = hwMap.dcMotor.get("bootMotor");
        sideServo = hwMap.servo.get("sideServo");
        sweepServo = hwMap.servo.get("sweepServo");
        lockBackServo = hwMap.servo.get("lockBackServo");
        lockFrontServo = hwMap.servo.get("lockFrontServo");
        */
        // Lift
        //liftMotor = hwMap.dcMotor.get("liftMotor");
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

}
