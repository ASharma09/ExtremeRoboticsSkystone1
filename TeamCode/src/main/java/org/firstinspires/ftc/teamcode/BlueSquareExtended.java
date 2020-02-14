package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Random;

@Autonomous(name="BlueSquareExtended aka 2.0")
public class BlueSquareExtended extends LinearOpMode {

    //skystonePosition is for the movement part of the autonomous
    private int skystonePosition = 0;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public String[] position = new String[3];

    private int i = 0;

    private double speed = 0.5;

    //skystonePlace is for the runVuforia() method to return
    private int skystonePlace;

    private static final String VUFORIA_KEY =
            "AWgpcWb/////AAABmaGz2/7eLkNnmdf7SE2FZHVj9lqZbwrKkg/8If3+fcqfKrCTI8mB7LVi91U36rl9kbu" +
                    "9YQN4E885gcuB1EuzBZM+aDWlSel0plj5FnPPsz4+S4Z9jfMgPyCtF7yOJ93ijYIT/LWznGg34Q" +
                    "ljTOfBIsABnXFd/2iZKuTD2XMewqk+Fw+vJLEZK4SR18I4Cs/L4HXUK9BO/3KwQPTQc5nRisUJVV" +
                    "sxv4wcNFGaECpBrUdg5icwVdMfuPbPjDwtIpX7wuDLdLFXZkveohubFUlcnPGFGrj7QFald5V92+" +
                    "1C8fmKfcmv/LfuHVWJQ/4bgSkE1NcN+G/bmlnV1Wv8clJ3JzV/w3FK2+1GX6vXsD44rtHt";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private int toAngleTicks = 25;

    @Override
    public void runOpMode() {
        initialize();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        //BASIC INITIALIZATION RESETS
        resetAngle();
        stopAndResetEncoder();
        runUsingEncoder();
        moveChicken(-1);
        moveFoundation(1);

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        encoderStrafe(0.5, 1, 830);
        skystonePosition = runVuforia();
        encoderStrafe(0.5, 1, 560);
        toAngle(toAngleTicks);

        if (skystonePosition == 1) {
            //encoderBack(0.2, 350);
            telemetry.addData("skystone path ", 1);
            telemetry.update();
            encoderForward(0.4, 770);
            encoderStrafe(.3, 1, 45);

            //encoderStrafe(0.2, 1, 560);

            moveChicken(1);
            encoderStrafe(0.3, -1, 475);
            toAngle(toAngleTicks);
            encoderBack(.5, 2600);
            moveChicken(-1);
            encoderStrafe(speed, -1, 50);
            toAngle(toAngleTicks);

            encoderForward(0.4, 850);
/*            encoderForward(0.3, 400);
            encoderStrafe(speed, 1, 500);
            toAngle(toAngleTicks);
            moveChicken(1); //grabs second block
            encoderStrafe(speed, -1, 760);
            toAngle(toAngleTicks);
            encoderBack(0.8, 3600);
            moveChicken(-1); //releases second block
            encoderForward(1, 800);*/

            //encoderForward(speed, 500);

//            encoderBack(0.5, 2200);
//            encoderForward(speed, 600);
//            encoderTurn(0.5, -1, 730, 730);
//            encoderForward(0.5, 700);
//            moveFoundation(-1);
//            sleep(1000);
//            encoderBack(0.5, 2000);
        }
        if (skystonePosition == 2) {
            //move a bit less to the right than position 1
            telemetry.addData("skystone path ", 2);
            telemetry.update();
            encoderForward(0.5, 510);

            encoderStrafe(0.5, 1, 180);

            moveChicken(1);
            encoderStrafe(speed, -1, 465);
            toAngle(toAngleTicks);
            encoderBack(.75, 2400);
            moveChicken(-1);
            encoderStrafe(speed, 1, 50);
            toAngle(toAngleTicks);

            encoderForward(0.75, 3200);
            encoderForward(0.35, 400);
            //encoderForward(0.3, 344);
            encoderStrafe(speed, 1, 800);
            encoderForward(0.4, 100);
            moveChicken(1);
            encoderStrafe(speed, -1, 955);
            toAngle(toAngleTicks);
            encoderBack(.75, 3250);
            moveChicken(-1);
            encoderForward(0.8, 680);

//            encoderBack(speed, 2150);
//            encoderForward(0.4, 500);
//            encoderTurn(0.3, -1, 730, 730);
//            encoderForward(0.3, 300);
//            moveFoundation(-1);
//            sleep(1000);
//            encoderBack(0.3, 2000);
//            moveFoundation(1);
//            encoderStrafe(speed, -1, 2150);
        }
        if (skystonePosition == 3) {
            //move the same amount to the left as position 2
            telemetry.addData("skystone path ", 3);
            telemetry.update();
            encoderForward(0.75, 90);

            encoderStrafe(0.2, 1, 200);

            moveChicken(1);
            //bring chicken down
            encoderStrafe(speed, -1, 610 );
            toAngle(toAngleTicks);
            encoderBack(0.75, 1800);
            //release chicken wing
            moveChicken(-1);
            encoderStrafe(speed, -1, 50);
            toAngle(toAngleTicks);
            encoderForward(0.75, 3400);
            encoderBack(0.6, 310);
            encoderStrafe(speed, 1, 650);
            toAngle(toAngleTicks);
            moveChicken(1);
            //bring down chicken wing
            encoderStrafe(speed, -1, 680);
            toAngle(toAngleTicks);
            encoderBack(0.75, 2900);
            moveChicken(-1);

            encoderForward(1, 650);

//            encoderBack(0.5, 2200);
//            encoderForward(0.5, 600);
//            encoderTurn(0.5, -1, 730, 730);
//            encoderForward(0.5, 500);
//            moveFoundation(-1);
//            sleep(1000);
//            encoderBack(0.5, 2000);

            }
        }

        //ALL HARDWARE
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

        BNO055IMU imu;
        Orientation lastAngles = new Orientation();
        double globalAngle;

        //VUFORIA METHODS
        private int runVuforia () {
            initVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }

            if (tfod != null) {
                tfod.activate();
            }

//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
            //waitForStart();

            if (opModeIsActive()) {
                boolean isDoneWithVuforia = false;
                int isdone = 0;
                while (opModeIsActive()
                        && !isDoneWithVuforia
                ) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null /*|| getRuntime() > 7*/) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                //if (getRuntime() > 7)
                                {
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                                    //MAYBE
                                    //PUT IN i++ HERE TO CHANGE POSITION OF ELEMENT?? - A
                                    i++;

                                    if (recognition.getLeft() < 150 && recognition.getRight() < 600) {
                                        position[1] = recognition.getLabel();
                                    }
                                    if (recognition.getLeft() > 240 && recognition.getRight() > 630) {
                                        position[2] = recognition.getLabel();
                                    }
                                    //isdone++;
                                    //if (isdone==2) {
                                    isDoneWithVuforia = true;
                                    //}
                                }
                            }
//
//                        if (position[1] == "Stone" && position[2] == "Stone") {
//                            position[0] = "Skystone";
//                        }
//                        if (position[1] != "Stone" || position[2] != "Stone"){
//                            position[0] = "Stone";
//                        }

                            //telemetry.addData("block 1 is --", position[0]);
                            telemetry.addData("block 2 is --", position[1]);
                            telemetry.addData("block 3 is --", position[2]);

                            telemetry.update();

//                        if (position[0] == "Skystone") ;
//                        {
//                            skystonePlace = 1;
//                        }
                            if (position[1] == "Skystone") {
                                skystonePlace = 2;
                            }
                            if (position[2] == "Skystone") {
                                skystonePlace = 3;
                            }
                            if (position[1] != "Skystone" && position[2] != "Skystone") {
                                skystonePlace = 1;
                            }
                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
            telemetry.addData("I made it!", "skystonePlace");
            telemetry.update();
            return skystonePlace;
        }
        private int getSkystone () {
            return skystonePlace;
        }
        private void initVuforia () {

            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            //this is apparently what I was having a problem with. By making a object of the ECD class
            //I wasn't getting the vuforia engine instantiated
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }
        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }

        // GYRO METHODS
        private double getAngle () {
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
        private void resetAngle () {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
        }
        private void toAngle (int ticks) {
            if (getAngle() > 0) {
                //if getAngle() is pos it is to the left
                //turn right
//            double speed = 1;
//            rightBackDrive.setPower(-speed);
//            rightFrontDrive.setPower(-speed);
//            leftFrontDrive.setPower(speed);
//            leftBackDrive.setPower(speed);
                while (getAngle() > 0 && opModeIsActive()) {
//                telemetry.addData("inside while loop", i);
                    //telemetry.addData("angle value inside while loop", getAngle());
//                telemetry.update();
//                //sleep(100);
//                i++;
                    encoderTurn(.5, -1, ticks, ticks);
                }
//                double speed = 0.5;
                //
            } else {
//            double speed = 1;
//            rightBackDrive.setPower(speed);
//            rightFrontDrive.setPower(speed);
//            leftFrontDrive.setPower(-speed);
//            leftBackDrive.setPower(-speed);

                while (getAngle() < 0 && opModeIsActive()) {
                    //turn left
                    //encoderTurn(.5, 1, 17, 17);
                    //telemetry.addData("angle value inside while loop", getAngle());
//                telemetry.update();
//                //sleep(100);
//                i++;
                    encoderTurn(.5, 1, ticks, ticks);
                }
            }

            stopDriveBase();

            telemetry.addData("I reached end of loops", getAngle());
            telemetry.update();

            //sleep(3000);
        }

        //SERVO METHODS
        private void moveChicken ( int position){
            chickenServo.setPosition(position);
            sleep(500);
        }
        private void moveFoundation ( int direction){
            if (direction == 1) {
                FMRight.setPosition(1);
                FMLeft.setPosition(-1);
            }
            if (direction == -1) {
                FMRight.setPosition(-1);
                FMLeft.setPosition(1);
            }
        }

        //ENCODER MOVEMENT METHODS
        private void encoderBack ( double speed, int ticks){

            stopAndResetEncoder();

            runUsingEncoder();

            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

            leftFrontDrive.setTargetPosition(ticks);
            rightFrontDrive.setTargetPosition(ticks);
            leftBackDrive.setTargetPosition(ticks);
            rightBackDrive.setTargetPosition(ticks);

            runToPosition();

            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    //           (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy()
                    )
                    && leftFrontDrive.getCurrentPosition() < ticks) {

                // Display it for the driver.
                telemetry.addData("LFT, RFT", "Running to %7d :%7d", ticks, ticks);
                telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());

                telemetry.addData("LBT. RBT", "Running to %7d :%7d", ticks, ticks);
                telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            stopDriveBase();
            runUsingEncoder();

            leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        }
        private void encoderForward ( double speed, int ticks){
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
                leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                rightFrontDrive.setTargetPosition(newRightFrontTarget);

                //set target position for back wheels
                leftBackDrive.setTargetPosition(newLeftBackTarget);
                rightBackDrive.setTargetPosition(newRightBackTarget);

                runToPosition();

                leftFrontDrive.setPower(speed);
                leftBackDrive.setPower(speed);
                rightBackDrive.setPower(speed);
                rightFrontDrive.setPower(speed);

                while (opModeIsActive() &&
                        //           (runtime.seconds() < timeout) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                                leftBackDrive.isBusy() && rightBackDrive.isBusy()
                        )
                        && leftFrontDrive.getCurrentPosition() < ticks) {

                    // Display it for the driver.
                    telemetry.addData("LFT, RFT", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                    telemetry.addData("LFP, RFP", "Running at %7d :%7d",
                            leftFrontDrive.getCurrentPosition(),
                            rightFrontDrive.getCurrentPosition());

                    telemetry.addData("LBT. RBT", "Running to %7d :%7d", newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("LBP, RBP", "Running at %7d :%7d",
                            leftBackDrive.getCurrentPosition(),
                            rightBackDrive.getCurrentPosition());
                    telemetry.update();
                }

                stopDriveBase();
                runUsingEncoder();
            }

        }
        private void encoderTurn ( double speed, int direction, int leftTicks, int rightTicks){
            stopAndResetEncoder();
            runUsingEncoder();

            if (opModeIsActive()) {
                int leftF = leftFrontDrive.getCurrentPosition() + leftTicks;
                int leftB = leftBackDrive.getCurrentPosition() + leftTicks;
                int rightF = rightFrontDrive.getCurrentPosition() + rightTicks;
                int rightB = rightBackDrive.getCurrentPosition() + rightTicks;

                //if intent is left turn
                if (direction == -1) {
                    leftFrontDrive.setTargetPosition(-leftF);
                    leftBackDrive.setTargetPosition(-leftB);
                    rightFrontDrive.setTargetPosition(rightF);
                    rightBackDrive.setTargetPosition(rightB);
                }
                //if intent is right turn
                else if (direction == 1) {
                    leftFrontDrive.setTargetPosition(leftF);
                    leftBackDrive.setTargetPosition(leftB);
                    rightFrontDrive.setTargetPosition(-rightF);
                    rightBackDrive.setTargetPosition(-rightB);
                }

                runToPosition();

                if (direction == -1) {
                    leftFrontDrive.setPower(-speed);
                    leftBackDrive.setPower(-speed);
                    rightFrontDrive.setPower(speed);
                    rightBackDrive.setPower(speed);
                } else if (direction == 1) {
                    leftFrontDrive.setPower(speed);
                    leftBackDrive.setPower(speed);
                    rightFrontDrive.setPower(-speed);
                    rightBackDrive.setPower(-speed);
                }

                while (opModeIsActive() &&
                        //           (runtime.seconds() < timeout) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                                leftBackDrive.isBusy() && rightBackDrive.isBusy()
                        )
                        && leftFrontDrive.getCurrentPosition() < leftTicks) {

                    // Display it for the driver.
//                telemetry.addData("LFT, RFT", "Running to %7d :%7d", leftF, rightF);
//                telemetry.addData("LFP, RFP", "Running at %7d :%7d",
//                        robot.leftFrontDrive.getCurrentPosition(),
//                        robot.rightFrontDrive.getCurrentPosition());
//
//                telemetry.addData("LBT. RBT", "Running to %7d :%7d", leftB, rightB);
//                telemetry.addData("LBP, RBP", "Running at %7d :%7d",
//                        robot.leftBackDrive.getCurrentPosition(),
//                        robot.rightBackDrive.getCurrentPosition());
                    // telemetry.update();
                }

                stopDriveBase();
            }
        }
        private void encoderStrafe ( double speed, int direction, int ticks){
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
                leftF = leftFrontDrive.getCurrentPosition() + ticks;
                rightF = rightFrontDrive.getCurrentPosition() + ticks;

                //new target for the back wheels
                //leftB = robot.leftBackDrive.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
                //rightB = robot.rightBackMotor.getCurrentPosition() + (int)(inch * COUNTS_PER_INCH);
                leftB = leftBackDrive.getCurrentPosition() + ticks;
                rightB = rightBackDrive.getCurrentPosition() + ticks;

                //if intent is to drive right
                if (direction == -1) {
                    //set target position for front wheels
                    leftFrontDrive.setTargetPosition(-leftF);
                    rightFrontDrive.setTargetPosition(rightF);

                    //set target position for back wheels
                    leftBackDrive.setTargetPosition(leftB);
                    rightBackDrive.setTargetPosition(-rightB);
                }

                //if intent is to drive left
                else if (direction == 1) {
                    //set target position for front wheels
                    leftFrontDrive.setTargetPosition(leftF);
                    rightFrontDrive.setTargetPosition(-rightF);

                    //set target position for back wheels
                    leftBackDrive.setTargetPosition(-leftB);
                    rightBackDrive.setTargetPosition(rightB);
                }

                runToPosition();

                if (direction == -1) {
                    leftFrontDrive.setPower(-speed);
                    leftBackDrive.setPower(speed);
                    rightFrontDrive.setPower(speed);
                    rightBackDrive.setPower(-speed);
                } else if (direction == 1) {
                    leftFrontDrive.setPower(speed);
                    leftBackDrive.setPower(-speed);
                    rightFrontDrive.setPower(-speed);
                    rightBackDrive.setPower(speed);
                }

                while (opModeIsActive() &&
                        //(runtime.seconds() < time) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                                && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", leftF, rightF);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            leftFrontDrive.getCurrentPosition(),
                            rightFrontDrive.getCurrentPosition());

                    telemetry.addData("Path3", "Running to %7d :%7d", leftB, rightB);
                    telemetry.addData("Path4", "Running at %7d :%7d",
                            leftBackDrive.getCurrentPosition(),
                            rightBackDrive.getCurrentPosition());
                    telemetry.update();
                }

                stopDriveBase();
            }

        }

        //BASIC ENCODER METHODS
        private void stopAndResetEncoder () {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        private void runUsingEncoder () {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        private void stopDriveBase () {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        private void runToPosition () {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        //INITIALIZE ALL THE HARDWARE METHOD
        private void initialize () {
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




