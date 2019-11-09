package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "drive by encoder", group = "ExtremeBot")

public class encoderTest extends LinearOpMode {
    Robot robot = new Robot(); //use from robot class
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: ANDYMARK Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 0.6;
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
        //robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition()
                /*robot.rightFrontMotor.getCurrentPosition()*/);
        telemetry.update();

        waitForStart();

        //put in autonomous commands from encoderDrive
        //encoderDrive(DRIVE_SPEED, #, #, #);

        //get servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void driveEncoder(double speed, double leftInch, double rightInch, double timeout) {
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //new target for the front wheels
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (leftInch * COUNTS_PER_INCH);
            //newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (rightInch * COUNTS_PER_INCH);


            telemetry.addData("new target:", newLeftFrontTarget);
            telemetry.update();

            //set target position for front wheels
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            //robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            //robot.rightFrontMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()
                            && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy()
                    )
                    && robot.leftFrontMotor.getCurrentPosition() < leftInch) {

                // Display it for the driver.
               // telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition()/*,
                        robot.rightFrontMotor.getCurrentPosition()*/);

                telemetry.update();
            }


            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            //robot.rightFrontMotor.setPower(0);


            //Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move

        }

    }
}