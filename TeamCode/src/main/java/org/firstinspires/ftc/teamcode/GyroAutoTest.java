

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//WEBSITE: https://stemrobotics.cs.pdx.edu/node/7265

@Autonomous(name="GyroAutoTest")
//@Disabled
public class GyroAutoTest extends LinearOpMode
{

    Orientation anglesRight = new Orientation();
    Orientation anglesLeft = new Orientation();

    DcMotor leftBackDrive = null;
    DcMotor leftFrontDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor rightFrontDrive = null;

    //Robot robot = new Robot();
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();
        resetAngle();
    //    while (opModeIsActive()) {
        telemetry.addData("Mode", "running");
        telemetry.addData("angle", getAngle());
        telemetry.update();
     //   }

        sleep(3000);

        while(Math.signum(getAngle()) > 0) { //if getAngle() is pos it is to the left
            //turn right
            double speed = 0.2;
            rightBackDrive.setPower(-speed);
            rightFrontDrive.setPower(-speed);
            leftFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);

            if (getAngle() >= -1 && getAngle() <= 1) {
                stopMotors();
            }
            return;
        }

        while(Math.signum(getAngle()) < 0) {
            //turn left
            double speed = 0.2;
            rightBackDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftFrontDrive.setPower(-speed);
            leftBackDrive.setPower(-speed);

            if (getAngle() >= -1 && getAngle() <= 1) {
                stopMotors();
            }
            return;
        }


//        rotate(0, 0.3);
//        telemetry.addData("reached rotate", getAngle());

        //rotate(45, 0.3);
        //turnRight(-90);

        //turnRight(45);

        telemetry.addData("reached 2nd rotate", getAngle());


        // turn the motors off.
        stopMotors();


        telemetry.update();
        sleep(3000);

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    /*https://ftcforum.firstinspires.org/forum/first-tech-challenge-community-forum-this-is-an-open-forum/teams-helping-teams-programming/71149-gyro-in-rev-expansion-hub#post71318*/
    //Link above is where I got turnLeft code

    public void turnLeft(double turnAngle) {
        sleep(500);

        anglesLeft = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        double oldDegreesLeft=turnAngle;
        double scaledSpeed= speed;
        double targetHeading=anglesLeft.firstAngle+turnAngle;
        double oldAngle=anglesLeft.firstAngle;

        if(targetHeading<-180) {
            targetHeading+=360;
        }
        if(targetHeading>180){
            targetHeading-=360;
        }

        double degreesLeft = ((int)(Math.signum(anglesLeft.firstAngle-targetHeading)+1)/2)*
                (360-Math.abs(anglesLeft.firstAngle-targetHeading))+
                (int)(Math.signum(targetHeading-anglesLeft.firstAngle)+1)/2*
                        Math.abs(anglesLeft.firstAngle-targetHeading);

        //runtime.reset();
        while(opModeIsActive() &&
                //runtime.seconds() < timeoutS &&
                degreesLeft>1&&
                oldDegreesLeft-degreesLeft>=0) { //check to see if we overshot target

            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
            if(scaledSpeed>1){
                scaledSpeed=.1;
            }
            leftBackDrive.setPower(scaledSpeed); //extra power to back wheels
            rightBackDrive.setPower(-1*scaledSpeed); //due to extra weight
            leftFrontDrive.setPower(scaledSpeed);
            rightFrontDrive.setPower(-1*scaledSpeed);
            anglesLeft = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft=degreesLeft;
            degreesLeft = ((int)(Math.signum(anglesLeft.firstAngle-targetHeading)+1)/2)*
                    (360-Math.abs(anglesLeft.firstAngle-targetHeading))+
                    (int)(Math.signum(targetHeading-anglesLeft.firstAngle)+1)/2*
                            Math.abs(anglesLeft.firstAngle-targetHeading);
            if(Math.abs(anglesLeft.firstAngle-oldAngle)<1){
                speed*=1.1;
            } //bump up speed to wheels in case robot stalls before reaching target

            oldAngle=anglesLeft.firstAngle;
        }
        //stopWheels(); //our helper method to set all wheel motors to zero
        sleep(250); //small pause at end of turn
    }

    //This is what I need help with. apparently I need different code for turnleft and turnright?
    //But I don't know how to make turnLeft into a turnRight...

    public void turnRight(double turnAngle) {
        sleep(500);

        anglesRight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        double oldDegreesRight = turnAngle;
        double scaledSpeed = speed;
        double targetHeading = anglesRight.firstAngle+turnAngle;
        double oldAngle=  anglesRight.firstAngle;

        if(targetHeading<-180) {
            targetHeading+=360;
        }
        if(targetHeading>180){
            targetHeading-=360;
        }

        double degreesRight = ((int)(Math.signum(anglesRight.firstAngle-targetHeading)+1)/2)*
                (360-Math.abs(anglesRight.firstAngle-targetHeading))+
                (int)(Math.signum(targetHeading-anglesRight.firstAngle)+1)/2*
                        Math.abs(anglesRight.firstAngle-targetHeading);
        telemetry.addData("degreesRight 1", degreesRight);

        //runtime.reset();
        while(opModeIsActive() &&
                //runtime.seconds() < timeoutS &&
                degreesRight>1&&
                oldDegreesRight-degreesRight<=0) { //check to see if we overshot target

            scaledSpeed = degreesRight/(100+degreesRight)*speed;
            if(scaledSpeed>1){
                scaledSpeed=.1;
            }
            leftBackDrive.setPower(-1*scaledSpeed); //extra power to back wheels
            rightBackDrive.setPower(scaledSpeed); //due to extra weight
            leftFrontDrive.setPower(-1*scaledSpeed);
            rightFrontDrive.setPower(scaledSpeed);
            anglesRight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesRight = degreesRight;
            degreesRight = ((int)(Math.signum(anglesRight.firstAngle-targetHeading)+1)/2)*
                    (360-Math.abs(anglesRight.firstAngle-targetHeading))+
                    (int)(Math.signum(targetHeading-anglesRight.firstAngle)+1)/2*
                            Math.abs(anglesRight.firstAngle-targetHeading);

            telemetry.addData("degreesRight 2", degreesRight);

            if(Math.abs(anglesRight.firstAngle-oldAngle)<1){
                speed*=1.1;
            } //bump up speed to wheels in case robot stalls before reaching target

            oldAngle=anglesRight.firstAngle;
        }
        //stopWheels(); //our helper method to set all wheel motors to zero
        telemetry.addData("reached sleep 1", anglesRight.firstAngle);
        sleep(250); //small pause at end of turn
    }

    public void stopMotors() {
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    //public void rotateToAngle(int newAngle) {
    /*
        double diff = newAngle - getAngle();
        double diff1 = Math.abs(diff-360);
        if (newAngle == getAngle()) {
            stopMotors();
            return;
        }
        while (Math.abs(getAngle() - newAngle) > 3&&opModeIsActive()) {
            telemetry.addData("newangle", newAngle);
            telemetry.addData("getangle()", getAngle());
            telemetry.update();
            diff = newAngle - getAngle();
            diff1=Math.abs(getAngle() - newAngle);
            if ((diff > 0 && diff < 180) || (diff < 0 && Math.abs(diff) > 180)) {

                if(Math.abs(diff1)<13)
                    rotate(-0.06);
                else if(Math.abs(diff1)<50)
                    rotate(-0.1);
                else if(Math.abs(diff1)<160)
                    rotate(-0.5);
                else
                    rotate(-(0.00928571*Math.abs(diff1))+0.128571);
            } else {

                if(Math.abs(diff1)<13)
                    rotate(0.06);
                else if(Math.abs(diff1)<50)
                    rotate(0.1);
                else if(Math.abs(diff1)<160)
                    rotate(0.5);
                else
                    rotate((0.00928571*Math.abs(diff1))+0.128571);
            }

        }
        stopMotors();
    }
*/

    public void rotate(double power){
        //negative turns to left
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }


}