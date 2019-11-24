package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="TestOpMode")
public class TestOpMode extends encoderDrive {
    //Robot robot = new Robot();
    //BasicOpMode_Linear basic = new BasicOpMode_Linear();

    @Override
    public void runOpMode() {
        super.runOpMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Say", "All systems go!");
        //GO!!!

        final double speed = 0.5;

        waitForStart();

        //driveSide(speed, -1, 1785);
        driveEncoder(speed, speed, 5000, 5000);
        encoderBack(speed, 5000);
        turnEncoder(speed, 1, 5000, 5000);
        turnEncoder(speed, -1, 5000, 5000);
        driveSide(speed, 1, 5000);
        driveSide(speed, -1, 5000);


    }
}

