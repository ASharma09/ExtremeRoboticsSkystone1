package org.firstinspires.ftc.teamcode;

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;


/**
 * Created by femukund on 10/29/2017.
 */

@TeleOp
public class ExtremeBotDrive extends LinearOpMode {
    Robot robot = new Robot();

    //GoldAlignDetector goldDetector;
    //SamplingOrderDetector detector;

    double[] p = {1, 1, 1, 1};
    //leftfront = 0
    //rightfront = 1
    //leftback = 2
    //rightback = 3

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for game to start (driver presses PLAY
        waitForStart();

        // run until driver presses STOP
        while (opModeIsActive())
        //while (opModeIsActive() && (runtime.seconds() < 31.0))
        {
            //testWheelServo();
            driveWithTwoJoysticks();

        }
    }


    // drive with joysticks
    public void driveWithTwoJoysticks() {
        double max;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back and crabs left and right,
        // the Right stick tank turns left and right
        //double speedLF = -(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        double speedLB = -(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        //double speedRF = -(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        double speedRB = -(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);


        // Clip values so that they are within -1 & +1
        //speedLF = Range.clip(speedLF, -1, 1);
        speedLB = Range.clip(speedLB, -1, 1);
        //speedRF = Range.clip(speedRF, -1, 1);
        speedRB = Range.clip(speedRB, -1, 1);

        // Set speed to motors
        //leftfront = 0
        //rightfront = 1
        //leftback = 2
        //rightback = 3
//        robot.leftFrontMotor.setPower(p[0] * speedLF);
        robot.leftBackDrive.setPower(speedLB);
//        robot.rightFrontMotor.setPower(p[1] * speedRF);
        robot.rightBackDrive.setPower(speedRB);
    }

//    public void testWheelServo () {
//        if (gamepad1.a)
//        {
//            robot.leftServo.setPosition(1);
//            robot.rightServo.setPosition(-1);
//        }
//        if (gamepad1.b)
//        {
//            robot.leftServo.setPosition(-1);
//            robot.rightServo.setPosition(1);
//        }
    }

