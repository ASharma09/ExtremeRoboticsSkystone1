// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

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

@Autonomous(name="gyrytesty")
//@Disabled
public class alsogyrotestugh extends LinearOpMode
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
        //while (opModeIsActive()) {
        telemetry.addData("Mode", "running");
        telemetry.addData("angle", getAngle());
        telemetry.update();
        //}

        sleep(3000);

        telemetry.addData("angle before ze loops", getAngle());
        telemetry.update();

        sleep(2000);

        while(getAngle() > 0) { //if getAngle() is pos it is to the left
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

        while(getAngle() < 0) {
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

        telemetry.addData("i reached end of loops", getAngle());
        telemetry.update();

        sleep(3000);

//        while(getAngle() != 90) {
//            robot.rightBackDrive.setPower(-0.5);
//            robot.rightFrontDrive.setPower(-0.5);
//            robot.leftFrontDrive.setPower(0.5);
//            robot.leftBackDrive.setPower(0.5);
//        }
//        rotate(0, 0.3);
//        telemetry.addData("reached rotate", getAngle());

        //rotate(45, 0.3);



        // drive until end of period.

//        while (opModeIsActive())
//        {
//            // Use gyro to drive in a straight line.
//            correction = checkDirection();
//
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("3 correction", correction);
//            telemetry.update();
//
//            leftBackDrive.setPower(power - correction);
//            leftFrontDrive.setPower(power - correction);
//            rightBackDrive.setPower(power + correction);
//            rightFrontDrive.setPower(power + correction);
//
//            // We record the sensor values because we will test them in more than
//            // one place with time passing between those places. See the lesson on
//            // Timing Considerations to know why.
//
//        }

        // turn the motors off.
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);

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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.

        while (degrees != getAngle()) {
            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);
            //telemetry.addData("angle", getAngle());
        }
        if (degrees == getAngle()) {
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            //telemetry.addData("angle", getAngle());
            //telemetry.update();
        }

        // rotate until turn is completed.
//        if (degrees < 0)
//        {
//            // On right turn we have to get off zero first.
//            while (opModeIsActive() && getAngle() == 0) {
//
//            }
//
//            while (opModeIsActive() && getAngle() > degrees) {
//
//            }
//        }
//        if (degrees > 0) {    // left turn.
//            while (opModeIsActive() && getAngle() < degrees) {
//
//            }
//        }

//        // turn the motors off.
//        rightBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        leftFrontDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    //https://ftcforum.firstinspires.org/forum/
    // first-tech-challenge-community-forum-this-is-an-
    // open-forum/teams-helping-teams-programming/71149-
    // gyro-in-rev-expansion-hub#post71318

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

        //runtime.reset();
        while(opModeIsActive() &&
                //runtime.seconds() < timeoutS &&
                degreesRight>1&&
                oldDegreesRight-degreesRight>=0) { //check to see if we overshot target

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
            if(Math.abs(anglesRight.firstAngle-oldAngle)<1){
                speed*=1.1;
            } //bump up speed to wheels in case robot stalls before reaching target

            oldAngle=anglesRight.firstAngle;
        }
        //stopWheels(); //our helper method to set all wheel motors to zero
        sleep(250); //small pause at end of turn
    }

    public void stopMotors() {
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
}