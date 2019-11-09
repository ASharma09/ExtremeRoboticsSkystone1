/*package org.firstinspires.ftc.teamcode;

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import static android.R.attr.left;
import static android.R.attr.right;
import static android.R.attr.wallpaperCloseEnterAnimation;


@TeleOp
public class ExtremeBotDrive2 extends LinearOpMode {
    HardwareMap hwMap;
    LinearOpMode opMode;
    // Hardware map & op mode
    public void init(HardwareMap ProtohwMap, LinearOpMode linearOpMode)
    {
        hwMap = ProtohwMap;
        opMode = linearOpMode;

        // Wheels
        //Robot wheels
        DcMotor rightFrontMotor;
        DcMotor leftFrontMotor;
        DcMotor rightBackMotor;
        DcMotor leftBackMotor;

        leftFrontMotor = hwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hwMap.dcMotor.get("leftBackMotor");
        //rightFrontMotor = hwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = hwMap.dcMotor.get("rightBackMotor");
        //  testRightServo = hwMap.servo.get("testRightServo");
        // testLeftServo = hwMap.servo.get("testLeftServo");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LinearOpMode opMode;
    double drivePower = 0.9;
    // Variables for Encoder driving
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andy Mark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();


    double leftMotorTgtPower = 0;
    double rightMotorTgtPower = 0;
    // plz work PLZZZ
    // Arm
    double liftBackMotorTgtPower = .1;
    double frontArmServo = 0.0;
    double backArmServo = 0.0;
    double stopServo = 0.0;
    double positionSS = 0.0;
    double testRightServo = 0.0;
    double testLeftServo = 0.0;
    //GoldAlignDetector goldDetector;
    //SamplingOrderDetector detector;

    double[] p = {1, 1, 1, 1};
    //leftfront = 0
    //rightfront = 1
    //leftback = 2
    //rightback = 3

        public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        //robot.init(hardwareMap, this);
        telemetry.update();

// drive
        public void drive() {
            driveWithTwoJoysticks();
            //testWheelServo();
            telemetry.update();
        }

        // Wait for game to start (driver presses PLAY
        waitForStart();

        // run until driver presses STOP
        while (opModeIsActive())
        //while (opModeIsActive() && (runtime.seconds() < 31.0))
        {
            drive();
        }
    }


    // drive with joysticks
    public void driveWithTwoJoysticks() {
        double max;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back and crabs left and right,
        // the Right stick tank turns left and right
        double speedLF = -(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        double speedLB = -(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        double speedRF = -(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        double speedRB = -(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);


        // Clip values so that they are within -1 & +1
        speedLF = Range.clip(speedLF, -1, 1);
        speedLB = Range.clip(speedLB, -1, 1);
        speedRF = Range.clip(speedRF, -1, 1);
        speedRB = Range.clip(speedRB, -1, 1);

        // Set speed to motors
        //leftfront = 0
        //rightfront = 1
        //leftback = 2
        //rightback = 3
        robot.leftFrontMotor.setPower(p[0] * speedLF);
        robot.leftBackMotor.setPower(p[2] * speedLB);
        robot.rightFrontMotor.setPower(p[1] * speedRF);
        robot.rightBackMotor.setPower(p[3] * speedRB);
    }

//    public void testWheelServo () {
//        if (gamepad1.a)
//        {
//            robot.testLeftServo.setPosition(1);
//            robot.testRightServo.setPosition(-1);
//        }
//        if (gamepad1.b)
//        {
//            robot.testLeftServo.setPosition(-1);
//            robot.testRightServo.setPosition(1);
//        }
//    }
}*/