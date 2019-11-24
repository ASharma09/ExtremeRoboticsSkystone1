package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="RedSquare")
public class RedSquare extends encoderDrive {
    //Robot robot = new Robot();
    //BasicOpMode_Linear basic = new BasicOpMode_Linear();

    @Override
    public void runOpMode() {
        super.runOpMode();

        //basic.init(hardwareMap, this);
//        private ElapsedTime runtime = new ElapsedTime();
//        private DcMotor leftBackDrive = null;
//        private DcMotor leftFrontDrive = null;
//        private DcMotor rightBackDrive = null;
//        private DcMotor rightFrontDrive = null;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Say", "All systems go!");
        //GO!!!

        final double speed = 0.8;

        waitForStart();

        //turn 90 degrees is 2519

        //driveSide(speed, -1, 1785);

        driveEncoder(speed, speed, 2819, 2819); //move forward
        //pickup furthest block
        encoderBack(speed, 684);
        //turnEncoder(speed, -1, 4000, 4000); //90 degrees, change later
        driveSide(speed, -1, 6623);
        driveEncoder(speed, speed, 919, 919);
        driveSide(speed, 1, 6758);
        turnEncoder(speed, -1, 1725, 1725); //45 degrees


//        driveEncoder(speed, speed, 4000, 4000); //push foundation
//        driveSide(speed, -1, 4000);
//        driveEncoder(speed, speed, 4000, 4000);
//        //deploy block
//        encoderBack(speed, 4000);
//        turnEncoder(speed, 1, 4000, 4000);
//        driveEncoder(speed, speed, 4000, 4000);
//        //pickup furthest block
//        encoderBack(speed, 4000);
//        turnEncoder(speed, -1, 4000, 4000);
//        encoderBack(speed, 4000); //til parked on middle line


    }
}

