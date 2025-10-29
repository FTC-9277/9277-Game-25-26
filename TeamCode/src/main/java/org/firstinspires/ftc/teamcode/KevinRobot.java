package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class KevinRobot {

    DcMotorEx fleft, fright, bright, bleft, sorter;
    Servo servoDoor;
    HardwareMap hardwareMap = null;


    IMU imu;
    public double imuDegree = 0;

    final double SORTER_ENCODER_RESOLUTION = 20;
    final double SORTER_SCALE_PER_TICK = 360/SORTER_ENCODER_RESOLUTION;
    final double SORTER_TICKS_PER_120_DEG = 120*(SORTER_ENCODER_RESOLUTION/360);

    final int WHEEL_DIAMETER = 96;
    final double WHEEL_ENCODER_RESOLUTION = 384.5;

    public KevinRobot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
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

    }

}
