package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



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

        encoderForward(speed, 2494);
        encoderStrafe(speed, 1, 7905); //skystone position 2
        encoderForward(speed, 440);
        encoderBack(speed, 600);
        encoderStrafe(speed, -1, 9000);
        encoderForward(speed, 830);
        //latch onto foundation
        encoderBack(speed, 3238);
        encoderStrafe(speed, 1, 5500); //onto blue line


    }
}

