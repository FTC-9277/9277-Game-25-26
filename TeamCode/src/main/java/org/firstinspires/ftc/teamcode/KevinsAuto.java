package org.firstinspires.ftc.teamcode;
//  if (code not work); work
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "TestAuto")
public class KevinsAuto extends LinearOpMode {

    DcMotorEx fleft, fright, bright, bleft, sorter;
    Servo servoDoor;

    IMU imu;
    public double imuDegree = 0;

    final double SORTER_ENCODER_RESOLUTION = 20;
    final double SORTER_SCALE_PER_TICK = 360/SORTER_ENCODER_RESOLUTION;
    final double SORTER_TICKS_PER_120_DEG = 120*(SORTER_ENCODER_RESOLUTION/360);

    final int WHEEL_DIAMETER = 96;
    final double WHEEL_ENCODER_RESOLUTION = 384.5;


    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientation));


        imu.resetYaw();



        fleft = hardwareMap.get(DcMotorEx.class, "fleft");
        fright = hardwareMap.get(DcMotorEx.class, "fright");
        bright = hardwareMap.get(DcMotorEx.class, "bright");
        bleft = hardwareMap.get(DcMotorEx.class, "bleft");
        sorter = hardwareMap.get(DcMotorEx.class, "sorter");
        servoDoor = hardwareMap.get(Servo.class, "servo");
        bleft.setDirection(DcMotorEx.Direction.REVERSE);
        fleft.setDirection(DcMotorEx.Direction.REVERSE);















        fright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sorter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        // steps!

        // Step 1: Drive forward at a certain speed
        // Step 2: Wait UNTIL encoders reach a specified value
        // Step 3: Stop
//        drivingForward(500, .2);


//        while(opModeIsActive()) {
//            while (opModeIsActive()) {
//                telemetry.addData("Heading", getHeading());
//                telemetry.update();
//            }
//        }

        //This is the actual auto
        drivingForwardMM(267, .5);
        waitTime(670);
        streftAndStrightMM(670,.5);
        waitTime(67);
        turnBy(28.67,(670/2.0));

//        turnBy(90.0, 670);
//        waitTime(1000);
//        turnBy(-90.0, 670);





    }




    public double getHeading() {
//        double changeDegree = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        YawPitchRollAngles ang = imu.getRobotYawPitchRollAngles();
        double changeDegree = ang.getYaw(AngleUnit.DEGREES);

        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Change Degree",changeDegree);

        imuDegree += changeDegree;
//        imu.resetYaw();
        //double old = ang.getYaw(AngleUnit.DEGREES);
        return imuDegree;
    }

    public void turnBy(double deg, double maxPower) {
        double start = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double target = AngleUnit.normalizeDegrees(start-deg);
        int sign = (int)(deg/Math.abs(deg));

        while (Math.abs(target-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 3 && opModeIsActive()){
            telemetry.addData("start angle", start);
            telemetry.addData("target angle", target);
            telemetry.addData("current angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Dist from target", Math.abs(target-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            fleft.setVelocity(maxPower*sign);
            fright.setVelocity(maxPower*-sign);
            bright.setVelocity(maxPower*-sign);
            bleft.setVelocity(maxPower*sign);
            telemetry.update();
        }
        fleft.setVelocity(0);
        fright.setVelocity(0);
        bright.setVelocity(0);
        bleft.setVelocity(0);
    }

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }
    public void drivingForward (int target, double velocity) {

        //convert velocity to ticks



        target += averageTicks();
        while (Math.abs(target-averageTicks()) > 5 && opModeIsActive()) {
            int currentDistance = target-averageTicks();
            int sign = (currentDistance)/Math.abs(currentDistance);
            fleft.setVelocity(velocity*sign);
            fright.setVelocity(velocity*sign);
            bright.setVelocity(velocity*sign);
            bleft.setVelocity(velocity*sign);

            telemetry.addData("Encoder",averageTicks());
            telemetry.addData("Velocity",velocity*sign);
            telemetry.update();

        }

        fleft.setVelocity(0);
        fright.setVelocity(0);
        bright.setVelocity(0);
        bleft.setVelocity(0);

    }
    public void streftAndStright (int target, double velocity) {

        //convert velocity to ticks



        target += averageTicksFleftAndBright();
        while (Math.abs(target-averageTicksFleftAndBright()) > 5 && opModeIsActive()) {
            int currentDistance = target-averageTicksFleftAndBright();
            int sign = (currentDistance)/Math.abs(currentDistance);
            fleft.setVelocity(velocity*sign);
            fright.setVelocity(velocity*-sign);
            bright.setVelocity(velocity*sign);
            bleft.setVelocity(velocity*-sign);

            telemetry.addData("Encoder",averageTicksFleftAndBright());
            telemetry.addData("Velocity",velocity*sign);
            telemetry.update();

        }

        fleft.setVelocity(0);
        fright.setVelocity(0);
        bright.setVelocity(0);
        bleft.setVelocity(0);

    }

    public int averageTicks (){
        return (fleft.getCurrentPosition()+ fright.getCurrentPosition()+ bright.getCurrentPosition()+ bleft.getCurrentPosition())/4;
    }

    public int averageTicksFleftAndBright (){
        return (fleft.getCurrentPosition()+ bright.getCurrentPosition())/2;
    }

    public void drivingForwardINCH(double targetInches, double velocity) {
        drivingForwardMM((int) Math.round(targetInches/0.0394),velocity);
    }

    public void drivingForwardMM(int targetMillimeters, double velocityMetersPerSec) {
        double circumference = WHEEL_DIAMETER*Math.PI;
        double ticksPerMilli = WHEEL_ENCODER_RESOLUTION/circumference;
        double ticksPerMeter = ticksPerMilli * 1000;
        int targetTicks = (int)(targetMillimeters*(ticksPerMilli));

        double ticksPerSec = ticksPerMeter * velocityMetersPerSec;
        drivingForward(targetTicks, ticksPerSec);
    }
    public void streftAndStrightMM(int targetMillimeters, double velocityMetersPerSec) {
        double circumference = WHEEL_DIAMETER*Math.PI;
        double ticksPerMilli = WHEEL_ENCODER_RESOLUTION/circumference;
        double ticksPerMeter = ticksPerMilli * 1000;
        int targetTicks = (int)(targetMillimeters*(ticksPerMilli) * (Math.sqrt(2)));

        double ticksPerSec = ticksPerMeter * velocityMetersPerSec;
        streftAndStright(targetTicks, ticksPerSec);
    }
    public void driveUntilMilli (int milli, double velocity) {
        fleft.setVelocity(velocity) ;
        fright.setVelocity(velocity) ;
        bright.setVelocity(velocity) ;
        bleft.setVelocity(velocity) ;

        waitTime(milli);

        fleft.setVelocity(0) ;
        fright.setVelocity(0) ;
        bleft.setVelocity(0) ;
        bright.setVelocity(0) ;
    }


    public int getSorterPosition(){
        int ballPosition = 1;
        int position = sorter.getCurrentPosition();
        double degree = (position * SORTER_SCALE_PER_TICK)%360;

        //if less than 120 will be 1
        //it is already 1
        if (degree > 120 && degree <= 240) {
            ballPosition = 2;
        } else if (degree > 240 && degree <= 360) {
            ballPosition = 3;
        }
        return ballPosition;
    }

    public void turnToPosition(int goalPosition){
                                //finds ticks per deg
                        //mult degrees to rotate
        //this is the num of ticks in 120 deg

//Kevin kevin our glorious leader and king
//For him, nothing but praises we shall sing
        // SupremeGod turnCount = KEVIN_KEVIN;
        int ticksToTarget = 0;
        if (getSorterPosition() < goalPosition){
            ticksToTarget = (goalPosition-getSorterPosition());
        } else if (getSorterPosition() > goalPosition){
           ticksToTarget = (goalPosition+3-getSorterPosition());
        }
        ticksToTarget *= (int) SORTER_TICKS_PER_120_DEG;

        sorter.setTargetPosition(sorter.getTargetPosition() + ticksToTarget);

        //waits for motor to move to the position
        while (getSorterPosition()!=goalPosition){
        }

    }

    public void outputBall(){
        servoDoor.setPosition(0.5);
        sorter.setTargetPosition((int) (sorter.getCurrentPosition() + SORTER_TICKS_PER_120_DEG));
        servoDoor.setPosition(0.0);
    }


}
