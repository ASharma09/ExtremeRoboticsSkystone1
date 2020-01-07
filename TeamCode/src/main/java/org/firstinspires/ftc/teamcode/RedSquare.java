package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Random;




@Autonomous(name="RedSquare")
public class RedSquare extends encoderDrive {
    //Robot robot = new Robot();

    ExternalCameraDetection ECD = new ExternalCameraDetection();

    int skystonePosition;
    //BasicOpMode_Linear basic = new BasicOpMode_Linear();
/*
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public String[] position = new String[3];
    int skystonePlace;

    private static final String VUFORIA_KEY =
            "AWgpcWb/////AAABmaGz2/7eLkNnmdf7SE2FZHVj9lqZbwrKkg/8If3+fcqfKrCTI8mB7LVi91U36rl9kbu" +
                    "9YQN4E885gcuB1EuzBZM+aDWlSel0plj5FnPPsz4+S4Z9jfMgPyCtF7yOJ93ijYIT/LWznGg34Q" +
                    "ljTOfBIsABnXFd/2iZKuTD2XMewqk+Fw+vJLEZK4SR18I4Cs/L4HXUK9BO/3KwQPTQc5nRisUJVV" +
                    "sxv4wcNFGaECpBrUdg5icwVdMfuPbPjDwtIpX7wuDLdLFXZkveohubFUlcnPGFGrj7QFald5V92+" +
                    "1C8fmKfcmv/LfuHVWJQ/4bgSkE1NcN+G/bmlnV1Wv8clJ3JzV/w3FK2+1GX6vXsD44rtHt";
/*
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        super.runOpMode();
        //ECD.runVuforia();
//        skystonePosition = ETC.getSkystonePlace();
//        telemetry.addData("skystone position", skystonePosition);
        //basic.init(hardwareMap, this);
//        private ElapsedTime runtime = new ElapsedTime();
//        private DcMotor leftBackDrive = null;
//        private DcMotor leftFrontDrive = null;
//        private DcMotor rightBackDrive = null;
//        private DcMotor rightFrontDrive = null;

        //runVuforia();
        //telemetry.addData("Skystone position", getSkystone());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Say", "All systems go!");
        //GO!!!

        final double speed = 1;

        waitForStart();
        //turn 90 degrees is 2519

        //MOVE FORWARD TO CAMERA POSITION FIRST
        //encoderStrafe (0.5, 1, 850);
        //encoderStrafe (0.5, 1, 1500);

        //skystonePosition = runVuforia();
        //ECD.runVuforia();
        skystonePosition = ECD.runVuforia();
//        telemetry.addData("position of skystone", skystonePosition);

        skystonePosition = 3;

        encoderStrafe(0.5, 1, 512);

        //MOVE TO ALIGN WITH SKYSTONE POSITION
        if (skystonePosition == 1) {
        }
        if (skystonePosition == 2) {
            //move a bit less to the right than position 1
        }
        if (skystonePosition == 3) {
            //move the same amount to the left as position 2

            encoderForward(0.1, 470);
            sleep(5000);
            encoderStrafe(0.1, -1, 360);
            encoderForward(0.2, 1250);
            //release chicken wing
            sleep(5000);
            encoderBack(0.2, 2320);
            encoderStrafe(0.1, 1, 360);
            sleep(5000);
            //bring down chicken wing
            encoderStrafe(0.1, -1, 550);
            encoderForward(0.2, 2700);
        }


        //release chicken wing



        //pickup furthest block or skystone
        moveChicken(1);


    }
/*

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

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            //MAYBE
                            //PUT IN i++ HERE TO CHANGE POSITION OF ELEMENT?? - A
                            i++;

                            if (recognition.getLeft() < 30 && recognition.getRight() < 450) {
                                position[1] = recognition.getLabel();
                            }
                            if (recognition.getLeft() > 240 && recognition.getRight() > 700) {
                                position[2] = recognition.getLabel();
                            }

                        }
                        if (position[1] == "Stone" && position[2] == "Stone") {
                            position[0] = "Skystone";
                        } else {
                            position[0] = "Stone";
                        }

                        telemetry.addData("block 1 is --", position[0]);
                        telemetry.addData("block 2 is --", position[1]);
                        telemetry.addData("block 3 is --", position[2]);

                        telemetry.update();


                        if (position[0] == "Skystone") ;
                        {
                            skystonePlace = 1;
                        }
                        if (position[1] == "Skystone") {
                            skystonePlace = 2;
                        }
                        if (position[2] == "Skystone") {
                            skystonePlace = 3;
                        }

                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
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

 */
}



