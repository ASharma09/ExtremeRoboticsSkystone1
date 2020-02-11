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

        final double speed = 0.5;

        moveChicken(-1);
        moveFoundation(1);

        waitForStart();

/*        encoderStrafe(speed, -1, 400);
        encoderForward(speed, 1450);
        moveFoundation(-1);
        sleep(1000);
        encoderBack(0.5, 1700);
        moveFoundation(1);
        sleep(1000);
        encoderStrafe(0.5, 1, 2250);*/
/*
        encoderForward(0.3, 250);
        encoderStrafe(0.3, -1, 1250);
        encoderStrafe(0.3, 1, 400);
        encoderForward(0.3, 1300);
        moveFoundation(-1);
        sleep(1000);
        encoderBack(0.3, 1700);
        moveFoundation(1);
        sleep(500);
        encoderForward(0.3, 80);
        encoderStrafe(0.3, 1, 1500);
        encoderTurn(0.3, 1, 110, 110);
        encoderStrafe(0.3, 1, 1200);
*/

        encoderForward(0.5, 1400);
        encoderStrafe(0.3, -1, 155);
        moveFoundation(-1);
        sleep(1000);
        encoderStrafe(0.3, 1, 400);
        encoderTurn(.3, -1, 800, 800);
        encoderForward(.3, 1200);
        moveFoundation(1);
        sleep(1000);
        encoderBack(0.3, 300);
        encoderStrafe(0.3, -1, 1600);
        encoderStrafe(0.3, 1, 100);
        encoderBack(.5, 1600);




    }
}

