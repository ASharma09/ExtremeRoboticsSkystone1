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
    static final double DRIVE_GEAR_REDUCTION = 75 / 90;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;

    double[] factor = {1, 1, 1, 1};

    public void runOpMode() {

        robot.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        stopAndResetEncoder();
        runUsingEncoder();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0: LFP, RFP ->", "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        final double speed = 0.8;

        robot.driveSide(speed, -1, 1785);
        robot.driveEncoder(speed, speed, 5738, 5738);
        //pickup furthest block
        robot.encoderBack(speed, 800);
        robot.turnEncoder(speed, -1, 4000, 4000); //90 degrees, change later
        robot.driveEncoder(speed, speed, 5355, 5355);
        robot.driveSide(speed, 1, 6758);


        //get servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderBack(double speed, int ticks) {

        stopAndResetEncoder();

        runUsingEncoder();

        robot.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.leftFrontMotor.setTargetPosition(ticks);
        robot.rightFrontMotor.setTargetPosition(ticks);
        robot.leftBackMotor.setTargetPosition(ticks);
        robot.rightBackMotor.setTargetPosition(ticks);

        runToPosition();

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
            telemetry.addData("LFT, RFT", "Running to %7d :%7d", ticks, ticks);
            telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                    robot.leftFrontMotor.getCurrentPosition(),
                    robot.rightFrontMotor.getCurrentPosition());

            telemetry.addData("LBT. RBT", "Running to %7d :%7d", ticks, ticks);
            telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                    robot.leftBackMotor.getCurrentPosition(),
                    robot.rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        stopDriveBase();
        runUsingEncoder();

        robot.leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void driveEncoder(double leftSpeed, double rightSpeed, int leftTicks, int rightTicks) {
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
            newLeftFrontTarget = leftTicks;
            newRightFrontTarget = rightTicks;

            //new target for the back wheels
            //newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(leftInch * COUNTS_PER_INCH);
            //newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightInch * COUNTS_PER_INCH);
            newLeftBackTarget = leftTicks;
            newRightBackTarget = rightTicks;


            //set target position for front wheels
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            //set target position for back wheels
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            runToPosition();

            robot.leftFrontMotor.setPower(leftSpeed);
            robot.leftBackMotor.setPower(leftSpeed);
            robot.rightBackMotor.setPower(rightSpeed);
            robot.rightFrontMotor.setPower(rightSpeed);

            while (opModeIsActive() &&
                    //           (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy()
                    )
                    && robot.leftFrontMotor.getCurrentPosition() < leftTicks) {

                // Display it for the driver.
                telemetry.addData("LFT, RFT", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());

                telemetry.addData("LBT. RBT", "Running to %7d :%7d", newLeftBackTarget, newRightBackTarget);
                telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            stopDriveBase();

            runUsingEncoder();

        }

    }

    public void turnEncoder(double speed, int direction, int ticks) {
        stopAndResetEncoder();
        runUsingEncoder();

        if (opModeIsActive()) {
            int leftF = robot.leftFrontMotor.getCurrentPosition() + ticks;
            int leftB = robot.leftBackMotor.getCurrentPosition() + ticks;
            int rightF = robot.rightFrontMotor.getCurrentPosition() + ticks;
            int rightB = robot.rightBackMotor.getCurrentPosition() + ticks;

            //if intent is left turn
            if (direction == -1) {
                robot.leftFrontMotor.setTargetPosition(-leftF);
                robot.leftBackMotor.setTargetPosition(-leftB);
                robot.rightFrontMotor.setTargetPosition(rightF);
                robot.rightBackMotor.setTargetPosition(rightB);
            }
            //if intent is right turn
            else if (direction == 1) {
                robot.leftFrontMotor.setTargetPosition(leftF);
                robot.leftBackMotor.setTargetPosition(leftB);
                robot.rightFrontMotor.setTargetPosition(-rightF);
                robot.rightBackMotor.setTargetPosition(-rightB);
            }

            runToPosition();

            if (direction == -1) {
                robot.leftFrontMotor.setPower(-speed);
                robot.leftBackMotor.setPower(-speed);
                robot.rightFrontMotor.setPower(speed);
                robot.rightBackMotor.setPower(speed);
            } else if (direction == 1) {
                robot.leftFrontMotor.setPower(speed);
                robot.leftBackMotor.setPower(speed);
                robot.rightFrontMotor.setPower(-speed);
                robot.rightBackMotor.setPower(-speed);
            }

            while (opModeIsActive() &&
                    //           (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy()
                    )
                    && robot.leftFrontMotor.getCurrentPosition() < ticks) {

                // Display it for the driver.
                telemetry.addData("LFT, RFT", "Running to %7d :%7d", leftF, rightF);
                telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());

                telemetry.addData("LBT. RBT", "Running to %7d :%7d", leftB, rightB);
                telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            stopDriveBase();
        }
    }

    public void driveSide(double speed, int direction, int ticks) {
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

           runToPosition();

            if (direction == -1) {
                robot.leftFrontMotor.setPower(-speed);
                robot.leftBackMotor.setPower(speed);
                robot.rightFrontMotor.setPower(speed);
                robot.rightBackMotor.setPower(-speed);
            } else if (direction == 1) {
                robot.leftFrontMotor.setPower(speed);
                robot.leftBackMotor.setPower(-speed);
                robot.rightFrontMotor.setPower(-speed);
                robot.rightBackMotor.setPower(speed);
            }

            while (opModeIsActive() &&
                    //(runtime.seconds() < time) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()
                            && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", leftF, rightF);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());

                telemetry.addData("Path3", "Running to %7d :%7d", leftB, rightB);
                telemetry.addData("Path4", "Running at %7d :%7d",
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            stopDriveBase();
           runUsingEncoder();

        }

    }

    public void stopAndResetEncoder() {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopDriveBase(){
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }
    public void runToPosition () {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}



