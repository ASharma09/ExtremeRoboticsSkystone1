package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "encoderDrive", group = "ExtremeBot")

public class encoderDrive extends LinearOpMode {
    Robot robot = new Robot(); //use from robot class
    //private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1425.2;    // eg: ANDYMARK Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 75/90;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;

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

        //telemetry.addData("stop and reset encoder:", "blehhh");
        //telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //telemetry.addData("run using encoder:", "hiiii");
        //telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0: LFP, RFP ->", "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        //put in autonomous commands from encoderDrive
        //encoderDrive(DRIVE_SPEED, #, #, #);

        //driveEncoder(DRIVE_SPEED, -DRIVE_SPEED, 10000, 10000);
        //driveEncoder(-DRIVE_SPEED, -DRIVE_SPEED, -10000, -10000);
        //driveEncoder(-DRIVE_SPEED, DRIVE_SPEED, -10000, 10000);
        //driveEncoder(DRIVE_SPEED, -DRIVE_SPEED, 10000, -10000);
        driveSide(DRIVE_SPEED, 1, 10000);
        driveSide(DRIVE_SPEED, -1, 10000);
        driveEncoder(DRIVE_SPEED, -DRIVE_SPEED, 10000, 10000);
        //driveEncoder(-(DRIVE_SPEED), -(-DRIVE_SPEED), 10000, 10000);
        encoderBack(DRIVE_SPEED, 10000);
        //driveSide(DRIVE_SPEED, 1, 10000);


        //get servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderBack(double speed, int ticks) {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.leftFrontMotor.setTargetPosition(ticks);
        robot.rightFrontMotor.setTargetPosition(ticks);
        robot.leftBackMotor.setTargetPosition(ticks);
        robot.rightBackMotor.setTargetPosition(ticks);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontMotor.setPower(Math.abs(speed));
        robot.leftBackMotor.setPower(Math.abs(speed));
        robot.rightBackMotor.setPower(Math.abs(speed));
        robot.rightFrontMotor.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                //           (runtime.seconds() < timeout) &&
                (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy()
                )
                && robot.leftFrontMotor.getCurrentPosition() < ticks) {

            // Display it for the driver.
            telemetry.addData("LFT, RFT",  "Running to %7d :%7d", ticks,  ticks);
            telemetry.addData("LFP, RFP",  "Running at %7d :%7d",
                    robot.leftFrontMotor.getCurrentPosition(),
                    robot.rightFrontMotor.getCurrentPosition());

            telemetry.addData("LBT. RBT",  "Running to %7d :%7d", ticks,  ticks);
            telemetry.addData("LBP, RBP",  "Running at %7d :%7d",
                    robot.leftBackMotor.getCurrentPosition(),
                    robot.rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void driveEncoder(double leftSpeed, double rightSpeed, double leftTicks, double rightTicks) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //new target for the front wheels
            //newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(leftTicks * COUNTS_PER_INCH);
            //newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightTicks * COUNTS_PER_INCH);
            newLeftFrontTarget = (int) leftTicks;
            newRightFrontTarget = (int) rightTicks;

            //new target for the back wheels
            //newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(leftInch * COUNTS_PER_INCH);
            //newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightInch * COUNTS_PER_INCH);
            newLeftBackTarget = (int) leftTicks;
            newRightBackTarget = (int) rightTicks;

            //telemetry.addData("new target:", newLeftFrontTarget);
            //telemetry.update();

            //set target position for front wheels
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            //set target position for back wheels
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            //telemetry.addData("setTargetPosition:", newLeftFrontTarget);
            //telemetry.update();

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //telemetry.addData("run to position:", newLeftFrontTarget);
            //telemetry.update();

            // reset the timeout time and start motion.
            //runtime.reset();
            robot.leftFrontMotor.setPower(leftSpeed);
            robot.leftBackMotor.setPower(leftSpeed);
            robot.rightBackMotor.setPower(rightSpeed);
            robot.rightFrontMotor.setPower(rightSpeed);
//            robot.leftFrontMotor.setPower(Math.abs(leftSpeed));
//            robot.rightFrontMotor.setPower(Math.abs(rightSpeed));
//            robot.leftBackMotor.setPower(Math.abs(leftSpeed));
//            robot.rightBackMotor.setPower(Math.abs(rightSpeed));

            //telemetry.addData("set power:", newLeftFrontTarget);
            //telemetry.update();

            while (opModeIsActive() &&
         //           (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy()
                          )
                && robot.leftFrontMotor.getCurrentPosition() < leftTicks) {

                // Display it for the driver.
                telemetry.addData("LFT, RFT",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("LFP, RFP",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());

                telemetry.addData("LBT. RBT",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("LBP, RBP",  "Running at %7d :%7d",
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            //sleep(3000);

            telemetry.addData("stop motors", newLeftFrontTarget);
            //telemetry.update();

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("run using encoder:", robot.leftFrontMotor.getCurrentPosition());
            telemetry.update();

            //sleep(3000);

            //  sleep(250);   // optional pause after each move

        }

    }


    public void driveSide (double speed, int direction, int ticks) {
        int leftF;
        int leftB;
        int rightF;
        int rightB;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //new target for the front wheels
            //leftF = robot.leftFrontMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            //rightF = robot.rightFrontMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            leftF = robot.leftFrontMotor.getCurrentPosition() + ticks;
            rightF = robot.rightFrontMotor.getCurrentPosition() + ticks;

            //new target for the back wheels
            //leftB = robot.leftBackMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            //rightB = robot.rightBackMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            leftB = robot.leftBackMotor.getCurrentPosition() + ticks;
            rightB = robot.rightBackMotor.getCurrentPosition() + ticks;


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
            //runtime.reset();

            if (direction == -1) {
                robot.leftFrontMotor.setPower(-speed);
                robot.leftBackMotor.setPower(speed);
                robot.rightFrontMotor.setPower(speed);
                robot.rightBackMotor.setPower(-speed);
            }

            else if (direction == 1) {
                robot.leftFrontMotor.setPower(speed);
                robot.leftBackMotor.setPower(-speed);
                robot.rightFrontMotor.setPower(-speed);
                robot.rightBackMotor.setPower(speed);
            }
//            robot.leftFrontMotor.setPower(speed);
//            robot.rightFrontMotor.setPower(speed);
//            robot.leftBackMotor.setPower(speed);
//            robot.rightBackMotor.setPower(speed);

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



