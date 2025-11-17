package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class KevinRobot {

    DcMotorEx fleft, fright, bright, bleft, shooter, shooter2 , sorter;
    Servo servoDoor;
    HardwareMap hardwareMap;
    LinearOpMode opMode;

    IMU imu;
    public double imuDegree = 0;

    final double SORTER_ENCODER_RESOLUTION = 384.5;
    final double SORTER_SCALE_PER_TICK = 360/SORTER_ENCODER_RESOLUTION;
    final double SORTER_TICKS_PER_120_DEG = 120*(SORTER_ENCODER_RESOLUTION/360);

    final int WHEEL_DIAMETER = 96;
    final double WHEEL_ENCODER_RESOLUTION = 384.5;

    double shooterSpeed = 0;

    boolean emergencyButtonPressed;

    final int SEC_TO_SHOOTER_SPEED = 4;
    //Not constant because auto and teleop use different speeds
    int maxLaunchSpeed = 1175;

    public ElapsedTime time = new ElapsedTime();

    public KevinRobot (HardwareMap hardwareMap, LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        initialize();
    }

    public void initialize() {
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
        servoDoor.setPosition(.17);
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
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setVelocity(0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setVelocity(0);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        time.reset();
    }


    public void turnBy(double deg, double maxPower) {
        double start = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double target = AngleUnit.normalizeDegrees(start-deg);
        int sign = (int)(deg/Math.abs(deg));

        while (Math.abs(target-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 3 && opMode.opModeIsActive()){
            opMode.telemetry.addData("start angle", start);
            opMode.telemetry.addData("target angle", target);
            opMode.telemetry.addData("current angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            opMode.telemetry.addData("Dist from target", Math.abs(target-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            fleft.setVelocity(maxPower*sign);
            fright.setVelocity(maxPower*-sign);
            bright.setVelocity(maxPower*-sign);
            bleft.setVelocity(maxPower*sign);
            opMode.telemetry.update();
        }
        fleft.setVelocity(0);
        fright.setVelocity(0);
        bright.setVelocity(0);
        bleft.setVelocity(0);
    }

    public void drivingForward (int target, double velocity) {

        //convert velocity to ticks



        target += averageTicks();
        while (Math.abs(target-averageTicks()) > 5 && opMode.opModeIsActive()) {
            int currentDistance = target-averageTicks();
            int sign = (currentDistance)/Math.abs(currentDistance);
            fleft.setVelocity(velocity*sign);
            fright.setVelocity(velocity*sign);
            bright.setVelocity(velocity*sign);
            bleft.setVelocity(velocity*sign);

            opMode.telemetry.addData("Encoder",averageTicks());
            opMode.telemetry.addData("Velocity",velocity*sign);
            opMode.telemetry.update();

        }

        fleft.setVelocity(0);
        fright.setVelocity(0);
        bright.setVelocity(0);
        bleft.setVelocity(0);

    }
    public void streftAndStright (int target, double velocity) {

        //convert velocity to ticks



        target += averageTicksFleftAndBright();
        while (Math.abs(target-averageTicksFleftAndBright()) > 5 && opMode.opModeIsActive()) {
            int currentDistance = target-averageTicksFleftAndBright();
            int sign = (currentDistance)/Math.abs(currentDistance);
            fleft.setVelocity(velocity*sign);
            fright.setVelocity(velocity*-sign);
            bright.setVelocity(velocity*sign);
            bleft.setVelocity(velocity*-sign);

            opMode.telemetry.addData("Encoder",averageTicksFleftAndBright());
            opMode.telemetry.addData("Velocity",velocity*sign);
            opMode.telemetry.update();

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

    public void adjustSorterDown(){
//        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20, .05, 0,0));
        sorter.setTargetPosition(sorter.getCurrentPosition()-15);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.1);
    }

    public void adjustSorterUp(){
//        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20, .05, 0,0));
        sorter.setTargetPosition(sorter.getCurrentPosition()+15);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.1);
    }

    public void turnToPosition(int goalPosition){
        //finds ticks per deg
        //mult degrees to rotate
        //this is the num of ticks in 120 deg

//Kevin kevin our glorious leader and king
//For him, nothing but praises we shall sing
//His code is always the best there is
//Unlike the drive team; every shot's a miss
//Kevin kevin's genius is skill, not luck
//He carries, even though the drivers suck
        // SupremeGod turnCount = KEVIN_KEVIN;
        int ticksToTarget = 0;
        if (getSorterPosition() < goalPosition){
            ticksToTarget = (goalPosition-getSorterPosition());
        } else if (getSorterPosition() > goalPosition){
            ticksToTarget = (goalPosition+3-getSorterPosition());
        }
        ticksToTarget *= (int) SORTER_TICKS_PER_120_DEG;

        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20, .05, 0,0));
        sorter.setTargetPosition(sorter.getTargetPosition() + ticksToTarget);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
       // sorter.setPositionPIDFCoefficients(15);
//        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients());

        opMode.telemetry.addData("get sorter position", getSorterPosition());
        opMode.telemetry.addData("get goal position", goalPosition);
        opMode.telemetry.addData("PID", sorter.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        opMode.telemetry.update();
        //waits for motor to move to the position
        int count = 0;
//        while (getSorterPosition()!=goalPosition && opMode.opModeIsActive()){
        while (Math.abs(sorter.getTargetPosition()-sorter.getCurrentPosition()) > 5 && opMode.opModeIsActive()){
            opMode.telemetry.addData("get sorter position", getSorterPosition());
            opMode.telemetry.addData("get goal position", goalPosition);
            opMode.telemetry.addData("target position", sorter.getTargetPosition());
            opMode.telemetry.addData("current position", sorter.getCurrentPosition());
            opMode.telemetry.addData("count", count);
            opMode.telemetry.addData("power", sorter.getPower());
            opMode.telemetry.addData("angle", (sorter.getCurrentPosition()* SORTER_SCALE_PER_TICK)%360);
            opMode.telemetry.addData("PID", sorter.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            opMode.telemetry.addData("button", opMode.gamepad2.b);

            opMode.telemetry.update();
            count++;

            if (opMode.gamepad2.b){

                break;
            }
        }
        opMode.telemetry.addData("completion", ticksToTarget);
        opMode.telemetry.update();
    }

//    public void openDoor(){
//        servoDoor.setPosition(0);
//        waitTime()
//        servoDoor.setPosition(0.0);
//    }
//grayson grayson, demigod son of Kevin
//After mastering code, he too shall ascend to heaven

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opMode.opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }

    public void shootBall () {

        if (time.seconds() <= SEC_TO_SHOOTER_SPEED) {
            opMode.telemetry.addData("slope", (double) (maxLaunchSpeed / SEC_TO_SHOOTER_SPEED));
            shooterSpeed = ((double) maxLaunchSpeed / SEC_TO_SHOOTER_SPEED) * time.seconds();
            shooter.setVelocity(shooterSpeed);
            shooter2.setVelocity(shooterSpeed);
            opMode.telemetry.addData("Motor speed", shooter.getVelocity());
            opMode.telemetry.addData("Motor set speed", shooterSpeed);
            opMode.telemetry.addData("Time", time);

        }else {
            shooter.setVelocity(maxLaunchSpeed);
            shooter2.setVelocity(maxLaunchSpeed);
            opMode.telemetry.addData("max speed", maxLaunchSpeed);
            opMode.telemetry.update();
        }

    }

    public void reverseBall () {

        if (time.seconds() <= SEC_TO_SHOOTER_SPEED) {
            shooterSpeed = -((double) maxLaunchSpeed / SEC_TO_SHOOTER_SPEED) * time.seconds();
            shooter.setVelocity(shooterSpeed);
            shooter2.setVelocity(shooterSpeed);
            opMode.telemetry.addData("Motor speed", shooter.getVelocity());
            opMode.telemetry.addData("Time", time);

            opMode.telemetry.update();
        } else {
            shooter.setVelocity(maxLaunchSpeed);
            shooter2.setVelocity(maxLaunchSpeed);
        }
    }





}
