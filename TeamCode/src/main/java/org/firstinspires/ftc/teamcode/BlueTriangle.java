package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="BlueTriangle")
public class BlueTriangle extends encoderDrive {
    //Robot robot = new Robot();
    //BasicOpMode_Linear basic = new BasicOpMode_Linear();

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Say", "All systems go!");
        //GO!!!

        final double speed = 0.8;

        waitForStart();

        //turn 90 degrees is 2519


        driveEncoder(speed, speed, 2819, 2819); //move forward

    }
}

