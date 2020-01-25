package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous(name="BlueSquare")
public class BlueSquare extends encoderDrive {

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

    int skystonePosition = 0;

    Robot robot = new Robot();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public String[] position = new String[3];
    //skystonePlace is for the runVuforia() method to return
    int skystonePlace;

    private static final String VUFORIA_KEY =
            "AWgpcWb/////AAABmaGz2/7eLkNnmdf7SE2FZHVj9lqZbwrKkg/8If3+fcqfKrCTI8mB7LVi91U36rl9kbu" +
                    "9YQN4E885gcuB1EuzBZM+aDWlSel0plj5FnPPsz4+S4Z9jfMgPyCtF7yOJ93ijYIT/LWznGg34Q" +
                    "ljTOfBIsABnXFd/2iZKuTD2XMewqk+Fw+vJLEZK4SR18I4Cs/L4HXUK9BO/3KwQPTQc5nRisUJVV" +
                    "sxv4wcNFGaECpBrUdg5icwVdMfuPbPjDwtIpX7wuDLdLFXZkveohubFUlcnPGFGrj7QFald5V92+" +
                    "1C8fmKfcmv/LfuHVWJQ/4bgSkE1NcN+G/bmlnV1Wv8clJ3JzV/w3FK2+1GX6vXsD44rtHt";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle;

    @Override
    public void runOpMode() {
        super.runOpMode();

        initialize();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();



        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        //resetAngle();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        resetAngle();

        telemetry.addData("Status", "Initialized");
        //GO!!!

        final double speed = 0.5;

        moveChicken(-1);
        moveFoundation(1);

        waitForStart();

        encoderStrafe(0.2, 1, 830);
        skystonePosition = runVuforia();
        //encoderStrafe(0.2, 1, 540);

//        if (skystonePosition == 1) {
//            //encoderBack(0.2, 350);
//            telemetry.addData("skystone path ", 1);
//            telemetry.update();
//            encoderForward(0.5, 770);
//
//            encoderStrafe(0.2, 1, 560);
//
//            moveChicken(1);
//            encoderStrafe(0.5, -1, 420);
//            encoderBack(0.5, 2600);
//            moveChicken(-1);
//            encoderStrafe(speed, -1, 50);
//
//            encoderForward(speed,  500);

//            encoderBack(0.5, 2200);
//            encoderForward(speed, 600);
//            encoderTurn(0.5, -1, 730, 730);
//            encoderForward(0.5, 700);
//            moveFoundation(-1);
//            sleep(1000);
//            encoderBack(0.5, 2000);

        //}
//        if (skystonePosition == 2) {
//            //move a bit less to the right than position 1
//            telemetry.addData("skystone path ", 2);
//            telemetry.update();
//            encoderForward(0.2, 510);
//
//            encoderStrafe(0.2, 1, 560);
//
//            moveChicken(1);
//            encoderStrafe(speed, -1, 470);
//            encoderBack(speed, 2400);
//            moveChicken(-1);
//            encoderStrafe(speed, -1, 50);
//
//            encoderForward(0.4, 3400);
//            //encoderForward(0.3, 344);
//            encoderStrafe(speed, 1, 600);
//            moveChicken(1);
//            encoderStrafe(speed, -1, 625);
//            encoderBack(0.5, 3250);
//            moveChicken(-1);
//            encoderStrafe(speed, -1, 50);
//
//            encoderForward(speed, 500);

//            encoderBack(speed, 2150);
//            encoderForward(0.4, 500);
//            encoderTurn(0.3, -1, 730, 730);
//            encoderForward(0.3, 300);
//            moveFoundation(-1);
//            sleep(1000);
//            encoderBack(0.3, 2000);
//            moveFoundation(1);
//            encoderStrafe(speed, -1, 2150);
        //}

//        if (skystonePosition == 3) {
            //move the same amount to the left as position 2
            telemetry.addData("skystone path ", 3);
            telemetry.update();
            encoderForward(0.2, 90);

            encoderStrafe(0.2, 1, 590);

            moveChicken(1);
            //bring chicken down
            encoderStrafe(speed, -1, 460);
            encoderBack(0.6, 1800);
            //release chicken wing
            moveChicken(-1);
            encoderStrafe(speed, -1, 50);

            encoderForward(0.6, 3200);
            encoderBack(0.5, 310);
            encoderStrafe(speed, 1, 460);
            moveChicken(1);
            //bring down chicken wing
            encoderStrafe(speed, -1, 510);
            encoderBack(0.5, 2900);
            moveChicken(-1);
            encoderStrafe(speed, -1, 50);

            encoderForward(speed, 500);

//            encoderBack(0.5, 2200);
//            encoderForward(0.5, 600);
//            encoderTurn(0.5, -1, 730, 730);
//            encoderForward(0.5, 500);
//            moveFoundation(-1);
//            sleep(1000);
//            encoderBack(0.5, 2000);

        //}
    }

    public int runVuforia() {
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
            while (opModeIsActive()
                    && !isDoneWithVuforia
            ) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                //MAYBE
                                //PUT IN i++ HERE TO CHANGE POSITION OF ELEMENT?? - A
                                i++;

                                if (recognition.getLeft() < 150 && recognition.getRight() < 650) {
                                    position[1] = recognition.getLabel();
                                    //block 2
                                }
                                if (recognition.getLeft() > 240 && recognition.getRight() > 600) {
                                    position[2] = recognition.getLabel();
                                    //block 3
                                }
                                isDoneWithVuforia=true;
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

    public int getSkystone() {
        return skystonePlace;
    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        //this is apparently what I was having a problem with. By making a object of the ECD class
        //I wasn't getting the vuforia engine instantiated
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private double getAngle()
    {
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

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void stopMotors() {
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    public void toAngle(double angle) {
        if(getAngle() > angle) {
            while (getAngle() > angle) { //if getAngle() is pos it is to the left
                //turn right

                double speed = 0.1;
                rightBackDrive.setPower(speed);
                rightFrontDrive.setPower(speed);
                leftFrontDrive.setPower(-speed);
                leftBackDrive.setPower(-speed);

            }
        }
        else {
            while (getAngle() < angle) {
                //turn left
                double speed = 0.1;
                rightBackDrive.setPower(-speed);
                rightFrontDrive.setPower(-speed);
                leftFrontDrive.setPower(speed);
                leftBackDrive.setPower(speed);

            }
        }

        stopMotors();
    }

    public void initialize() {
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


