package org.firstinspires.ftc.teamcode;

import java.lang.System.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@TeleOp(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")
//@Disabled
public class ExternalCameraDetection extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    public String[] position = new String[3];
    public int skystonePlace;

    private static final String VUFORIA_KEY =
            "AWgpcWb/////AAABmaGz2/7eLkNnmdf7SE2FZHVj9lqZbwrKkg/8If3+fcqfKrCTI8mB7LVi91U36rl9kbu" +
                    "9YQN4E885gcuB1EuzBZM+aDWlSel0plj5FnPPsz4+S4Z9jfMgPyCtF7yOJ93ijYIT/LWznGg34Q" +
                    "ljTOfBIsABnXFd/2iZKuTD2XMewqk+Fw+vJLEZK4SR18I4Cs/L4HXUK9BO/3KwQPTQc5nRisUJVV" +
                    "sxv4wcNFGaECpBrUdg5icwVdMfuPbPjDwtIpX7wuDLdLFXZkveohubFUlcnPGFGrj7QFald5V92+" +
                    "1C8fmKfcmv/LfuHVWJQ/4bgSkE1NcN+G/bmlnV1Wv8clJ3JzV/w3FK2+1GX6vXsD44rtHt";

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
        runVuforia();
    }

    public int runVuforia() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null || getRuntime() > 7) {
                        if (getRuntime() > 7) {
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
                            }
                            if (position[1] != "Stone" || position[2] != "Stone") {
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
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        return skystonePlace;
    }

    public int getSkystonePlace() {
        return skystonePlace;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //System.out.println(WebcamName.class.toString());
        telemetry.addData("webcam name", WebcamName.class.toString());
        telemetry.update();

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}