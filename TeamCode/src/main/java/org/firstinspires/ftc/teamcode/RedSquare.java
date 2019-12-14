package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Random;




@Autonomous(name="RedSquare")
public class RedSquare extends encoderDrive {
    //Robot robot = new Robot();
    Random rand = new Random();
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
        int randomNumber = rand.nextInt(2) + 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Say", "All systems go!");
        //GO!!!

        final double speed = 0.8;

        waitForStart();
        telemetry.addData("randomNumber: ", randomNumber);
        telemetry.update();
        //turn 90 degrees is 2519

        //driveSide(speed, -1, 1785);

        encoderForward(speed, 2945); //move forward
        if (randomNumber == 2) {
            encoderStrafe(speed, -1, 600);
        }
        if (randomNumber == 3) {
            encoderStrafe(speed, -1, 1200);
        }
        //pickup furthest block or skystone
        encoderBack(speed, 779);
        //turnEncoder(speed, -1, 4000, 4000); //90 degrees, change later
        encoderStrafe(speed, -1, 9733);
        encoderForward(speed, 919);
        //latch onto foundation
        encoderBack(speed, 3211);
        encoderStrafe(speed, 1, 5404);


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

