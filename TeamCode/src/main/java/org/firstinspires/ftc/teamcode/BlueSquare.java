package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="BlueSquare")
public class BlueSquare extends encoderDrive {
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

        //driveSide(speed, -1, 1785);

        encoderForward(speed, 2945); //move forward
        //pickup furthest block or skystone
        encoderBack(speed, 779);
        //turnEncoder(speed, -1, 4000, 4000); //90 degrees, change later
        encoderStrafe(speed, 1, 9733);
        encoderForward(speed, 919);
        //latch onto foundation
        encoderBack(speed, 3211);
        encoderStrafe(speed, -1, 5404);


    }
}

