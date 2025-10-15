package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    DcMotorEx fleft, fright, bright, bleft;
    Servo servo;

    IMU imu;
    public double imuDegree = 0;



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
        servo = hardwareMap.get(Servo.class, "servo");
        bleft.setDirection(DcMotorEx.Direction.REVERSE);
        fleft.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset ALLLL of the encoders
        fright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        // steps!

        // Step 1: Drive forward at a certain speed
        // Step 2: Wait UNTIL encoders reach a specified value
        // Step 3: Stop
//        drivingForward(500, .2);


        drivingForwardMM(500,0.1);
//        while(opModeIsActive()) {
//            while (opModeIsActive()) {
//                telemetry.addData("Heading", getHeading());
//                telemetry.update();
//            }
//        }




    }

    public double getHeading() {
//        double changeDegree = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        YawPitchRollAngles ang = imu.getRobotYawPitchRollAngles();
        double changeDegree = ang.getYaw(AngleUnit.DEGREES);

        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Change Degree",changeDegree);

        imuDegree += changeDegree;
        //imu.resetYaw();
        return imuDegree;
    }

    public void turmBy(double Deg, double maxPower, double kP) {
        double start = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double target = AngleUnit.normalizeDegrees(start+Deg);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("value",String.valueOf(start),String.valueOf(target), String.valueOf(heading));
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

    public int averageTicks (){
        return (fleft.getCurrentPosition()+ fright.getCurrentPosition()+ bright.getCurrentPosition()+ bleft.getCurrentPosition())/4;
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





}
