package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="RedTriangle")
public class RedTriangle extends encoderDrive {
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

        moveChicken(-1);
        moveFoundation(1);

        waitForStart();

        encoderStrafe(speed, 1, 400);
        encoderForward(speed, 1450);
        moveFoundation(-1);
        sleep(1000);
        encoderBack(0.5, 1700);
        moveFoundation(1);
        sleep(1000);
        encoderStrafe(0.5, -1, 2250);




    }
}

