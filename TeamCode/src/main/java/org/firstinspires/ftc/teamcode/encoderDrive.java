package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "encoderDrive", group = "ExtremeBot")

public class encoderDrive extends LinearOpMode {
    Robot robot; //use from robot class
    //private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1425.2;    // eg: ANDYMARK Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 75 / 90;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;

    double[] factor = {1, 1, 1, 1};

    public encoderDrive() {
        robot = new Robot();
    }

    //turn 90 degrees is 2360
    //100 ticks is about 1 inch when going forward
    //111 ticks is about 1 inch strafing

    //OMNI METHODS
    //Robot robot = new Robot(); //use from robot class
    //private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        stopAndResetEncoder();
        runUsingEncoder();

         //Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0: LFP, RFP ->", "Starting at %7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();
        //get servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void moveChicken(int position) {
        robot.chickenServo.setPosition(position);
        sleep(500);
    }

    public void moveFoundation(int direction) {
        if (direction == 1) {
            robot.FMRight.setPosition(1);
            robot.FMLeft.setPosition(-1);
        }
        if (direction == -1) {
            robot.FMRight.setPosition(-1);
            robot.FMLeft.setPosition(1);
        }
    }

    public void encoderBack(double speed, int ticks) {

        stopAndResetEncoder();

        runUsingEncoder();

        robot.leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.leftFrontDrive.setTargetPosition(ticks);
        robot.rightFrontDrive.setTargetPosition(ticks);
        robot.leftBackDrive.setTargetPosition(ticks);
        robot.rightBackDrive.setTargetPosition(ticks);

        runToPosition();

        robot.leftFrontDrive.setPower(Math.abs(speed));
        robot.leftBackDrive.setPower(Math.abs(speed));
        robot.rightBackDrive.setPower(Math.abs(speed));
        robot.rightFrontDrive.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                //           (runtime.seconds() < timeout) &&
                (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy()
                )
                && robot.leftFrontDrive.getCurrentPosition() < ticks) {

            // Display it for the driver.
            telemetry.addData("LFT, RFT", "Running to %7d :%7d", ticks, ticks);
            telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                    robot.leftFrontDrive.getCurrentPosition(),
                    robot.rightFrontDrive.getCurrentPosition());

            telemetry.addData("LBT. RBT", "Running to %7d :%7d", ticks, ticks);
            telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                    robot.leftBackDrive.getCurrentPosition(),
                    robot.rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        stopDriveBase();
        runUsingEncoder();

        robot.leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void encoderForward(double speed, int ticks) {
        stopAndResetEncoder();
        runUsingEncoder();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //new target for the front wheels
            //newLeftFrontTarget = (int)(leftTicks * COUNTS_PER_INCH);
            //newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightTicks * COUNTS_PER_INCH);
            newLeftFrontTarget = ticks;
            newRightFrontTarget = ticks;

            //new target for the back wheels
            //newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftInch * COUNTS_PER_INCH);
            //newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightInch * COUNTS_PER_INCH);
            newLeftBackTarget = ticks;
            newRightBackTarget = ticks;

            //set target position for front wheels
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);

            //set target position for back wheels
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            runToPosition();

            robot.leftFrontDrive.setPower(speed);
            robot.leftBackDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);

            while (opModeIsActive() &&
                    //           (runtime.seconds() < timeout) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy()
                    )
                    && robot.leftFrontDrive.getCurrentPosition() < ticks) {

                // Display it for the driver.
                telemetry.addData("LFT, RFT", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition());

                telemetry.addData("LBT. RBT", "Running to %7d :%7d", newLeftBackTarget, newRightBackTarget);
                telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            stopDriveBase();
            runUsingEncoder();
        }

    }

    public void encoderTurn(double speed, int direction, int leftTicks, int rightTicks) {
        stopAndResetEncoder();
        runUsingEncoder();

        if (opModeIsActive()) {
            int leftF = robot.leftFrontDrive.getCurrentPosition() + leftTicks;
            int leftB = robot.leftBackDrive.getCurrentPosition() + leftTicks;
            int rightF = robot.rightFrontDrive.getCurrentPosition() + rightTicks;
            int rightB = robot.rightBackDrive.getCurrentPosition() + rightTicks;

            //if intent is left turn
            if (direction == -1) {
                robot.leftFrontDrive.setTargetPosition(-leftF);
                robot.leftBackDrive.setTargetPosition(-leftB);
                robot.rightFrontDrive.setTargetPosition(rightF);
                robot.rightBackDrive.setTargetPosition(rightB);
            }
            //if intent is right turn
            else if (direction == 1) {
                robot.leftFrontDrive.setTargetPosition(leftF);
                robot.leftBackDrive.setTargetPosition(leftB);
                robot.rightFrontDrive.setTargetPosition(-rightF);
                robot.rightBackDrive.setTargetPosition(-rightB);
            }

            runToPosition();

            if (direction == -1) {
                robot.leftFrontDrive.setPower(-speed);
                robot.leftBackDrive.setPower(-speed);
                robot.rightFrontDrive.setPower(speed);
                robot.rightBackDrive.setPower(speed);
            } else if (direction == 1) {
                robot.leftFrontDrive.setPower(speed);
                robot.leftBackDrive.setPower(speed);
                robot.rightFrontDrive.setPower(-speed);
                robot.rightBackDrive.setPower(-speed);
            }

            while (opModeIsActive() &&
                    //           (runtime.seconds() < timeout) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy()
                    )
                    && robot.leftFrontDrive.getCurrentPosition() < leftTicks) {

                // Display it for the driver.
                telemetry.addData("LFT, RFT", "Running to %7d :%7d", leftF, rightF);
                telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition());

                telemetry.addData("LBT. RBT", "Running to %7d :%7d", leftB, rightB);
                telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            stopDriveBase();
        }
    }

    public void encoderStrafe(double speed, int direction, int ticks) {
        stopAndResetEncoder();
        runUsingEncoder();

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
            leftF = robot.leftFrontDrive.getCurrentPosition() + ticks;
            rightF = robot.rightFrontDrive.getCurrentPosition() + ticks;

            //new target for the back wheels
            //leftB = robot.leftBackDrive.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            //rightB = robot.rightBackMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
            leftB = robot.leftBackDrive.getCurrentPosition() + ticks;
            rightB = robot.rightBackDrive.getCurrentPosition() + ticks;

            //if intent is to drive right
            if (direction == -1) {
                //set target position for front wheels
                robot.leftFrontDrive.setTargetPosition(-leftF);
                robot.rightFrontDrive.setTargetPosition(rightF);

                //set target position for back wheels
                robot.leftBackDrive.setTargetPosition(leftB);
                robot.rightBackDrive.setTargetPosition(-rightB);
            }

            //if intent is to drive left
            else if (direction == 1) {
                //set target position for front wheels
                robot.leftFrontDrive.setTargetPosition(leftF);
                robot.rightFrontDrive.setTargetPosition(-rightF);

                //set target position for back wheels
                robot.leftBackDrive.setTargetPosition(-leftB);
                robot.rightBackDrive.setTargetPosition(rightB);
            }

            runToPosition();

            if (direction == -1) {
                robot.leftFrontDrive.setPower(-speed);
                robot.leftBackDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.rightBackDrive.setPower(-speed);
            } else if (direction == 1) {
                robot.leftFrontDrive.setPower(speed);
                robot.leftBackDrive.setPower(-speed);
                robot.rightFrontDrive.setPower(-speed);
                robot.rightBackDrive.setPower(speed);
            }

            while (opModeIsActive() &&
                    //(runtime.seconds() < time) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy()
                            && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", leftF, rightF);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition());

                telemetry.addData("Path3", "Running to %7d :%7d", leftB, rightB);
                telemetry.addData("Path4", "Running at %7d :%7d",
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            stopDriveBase();
        }

    }

    public void stopAndResetEncoder() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopDriveBase(){
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }
    public void runToPosition () {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}



