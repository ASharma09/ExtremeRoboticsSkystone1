package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="RedSquare")
public class RedSquare extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Say", "All systems go!");
        //GO!!!

        final double speed = 0.8;

        waitForStart();

        robot.driveSide(speed, -1, 1785);
        robot.driveEncoder(speed, speed, 5738, 5738);
        //pickup furthest block
        robot.encoderBack(speed, 800);
        robot.turnEncoder(speed, -1, 4000, 4000); //90 degrees, change later
        robot.driveEncoder(speed, speed, 5355, 5355);
        robot.driveSide(speed, 1, 6758);
//        robot.turnEncoder(speed, -1, 4000, 4000); //45 degrees
//        robot.driveEncoder(speed, speed, 4000, 4000);
//        robot.driveSide(speed, -1, 4000);
//        robot.driveEncoder(speed, speed, 4000, 4000);
//        //deploy block
//        robot.encoderBack(speed, 4000);
//        robot.turnEncoder(speed, 1, 4000, 4000);
//        robot.driveEncoder(speed, speed, 4000, 4000);
//        //pickup furthest block
//        robot.encoderBack(speed, 4000);
//        robot.turnEncoder(speed, -1, 4000, 4000);
//        robot.encoderBack(speed, 4000); //til parked on middle line



    }
}