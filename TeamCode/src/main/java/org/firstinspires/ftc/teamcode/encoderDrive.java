package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "drive by encoder", group = "ExtremeBot")

public class encoderDrive extends LinearOpMode {
    Robot robot = new Robot(); //use from robot class
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: ANDYMARK Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("stop and reset encoder:", "blehhh");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("run using encoder:", "hiiii");
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        //put in autonomous commands from encoderDrive
        //encoderDrive(DRIVE_SPEED, #, #, #);
        driveEncoder(DRIVE_SPEED, 2000, 2000, 5.0);
        //driveEncoder(DRIVE_SPEED, -12, -12, 5.0);
        //driveEncoder(DRIVE_SPEED, 12, -12, 5.0);
        //driveEncoder(DRIVE_SPEED, -12, 12, 5.0);
        //driveSide(DRIVE_SPEED, -1, 12, 5.0);
        //driveSide(DRIVE_SPEED, 1, 12, 5.0);

        //get servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void driveEncoder(double speed, double leftInch, double rightInch, double timeout) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //new target for the front wheels
            //newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(leftInch * COUNTS_PER_INCH);
            //newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightInch * COUNTS_PER_INCH);
            newLeftFrontTarget = (int) leftInch;
            newRightFrontTarget = (int) rightInch;

            //new target for the back wheels
            //newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(leftInch * COUNTS_PER_INCH);
            //newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightInch * COUNTS_PER_INCH);
            newLeftBackTarget = (int) leftInch;
            newRightBackTarget = (int) rightInch;

            telemetry.addData("new target:", newLeftFrontTarget);
            telemetry.update();

            //set target position for front wheels
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            //set target position for back wheels
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            telemetry.addData("setTargetPosition:", newLeftFrontTarget);
            telemetry.update();

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("run to position:", newLeftFrontTarget);
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            telemetry.addData("set power:", newLeftFrontTarget);
            telemetry.update();

  //          while (opModeIsActive() &&
//                    (runtime.seconds() < timeout) &&
    //                (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()
//                            && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy()
      //              )
        //    && robot.leftFrontMotor.getCurrentPosition() < leftInch) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.leftFrontMotor.getCurrentPosition(),
//                        robot.rightFrontMotor.getCurrentPosition());
//
//                telemetry.addData("Path3",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
//                telemetry.addData("Path4",  "Running at %7d :%7d",
//                        robot.leftBackMotor.getCurrentPosition(),
//                        robot.rightBackMotor.getCurrentPosition());
//                telemetry.update();
            //}

            telemetry.addData("while loop:", newLeftFrontTarget);
            telemetry.update();

            sleep(3000);

            telemetry.addData("sleep method:", robot.leftFrontMotor.getCurrentPosition());
            telemetry.update();

            sleep(3000);

            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
            telemetry.update();

            sleep(3000);

            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();

            sleep(3000);

            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.leftFrontMotor.getCurrentPosition(),
                    robot.rightFrontMotor.getCurrentPosition());
            telemetry.update();


            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            sleep(3000);

            telemetry.addData("stop motors", newLeftFrontTarget);
            telemetry.update();

            // Turn off RUN_TO_POSITION
//            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("run using encoder:", robot.leftFrontMotor.getCurrentPosition());
            telemetry.update();

            sleep(3000);

            //  sleep(250);   // optional pause after each move

        }

    }

    public void driveSide (double speed, int direction, int inch, double timeout) {
        int leftF;
        int leftB;
        int rightF;
        int rightB;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //new target for the front wheels
            leftF = robot.leftFrontMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            rightF = robot.rightFrontMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);

            //new target for the back wheels
            leftB = robot.leftBackMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            rightB = robot.rightBackMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);


            //if intent is to drive left
            if (direction == -1) {
                //set target position for front wheels
                robot.leftFrontMotor.setTargetPosition(-leftF);
                robot.rightFrontMotor.setTargetPosition(rightF);

                //set target position for back wheels
                robot.leftBackMotor.setTargetPosition(leftB);
                robot.rightBackMotor.setTargetPosition(-rightB);
            }

            //if intent is to drive right
            else if (direction == 1) {
                //set target position for front wheels
                robot.leftFrontMotor.setTargetPosition(leftF);
                robot.rightFrontMotor.setTargetPosition(-rightF);

                //set target position for back wheels
                robot.leftBackMotor.setTargetPosition(-leftB);
                robot.rightBackMotor.setTargetPosition(rightB);
            }

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    //(runtime.seconds() < time) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()
                            && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", leftF,  rightF);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());

                telemetry.addData("Path3",  "Running to %7d :%7d", leftB,  rightB);
                telemetry.addData("Path4",  "Running at %7d :%7d",
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }

    }
}

