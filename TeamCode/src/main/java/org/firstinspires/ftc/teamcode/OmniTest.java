package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="OmniTest")
public class OmniTest extends LinearOpMode {
    //package org.firstinspires.ftc.teamcode;

    //public class ExtremeBotAutoCrater extends LinearOpMode {
        private Robot robot = new Robot();

        @Override
        public void runOpMode() {
            double drivePower = 0.35;

            robot.init(hardwareMap, this);
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            telemetry.addData("Say", "All systems go!");
            //GO!!!

            waitForStart();

            robot.MdriveForward(drivePower, 2000);
            robot.MdriveBackward(drivePower, 2000);
            robot.MdriveRight(drivePower, 2000);
            robot.MdriveLeft(drivePower, 2000);
            robot.MturnLeft(drivePower, 2000);
            robot.MturnRight(drivePower, 2000);

        }
    }


