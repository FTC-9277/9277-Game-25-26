package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    DcMotor fleft, fright, bright, bleft;
    Servo servo;

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        servo = hardwareMap.get(Servo.class, "serv");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        // steps!


        fleft.setPower(power);
        fright.setPower(power);
        bright.setPower(power);
        bleft.setPower(power);

        waitTime(1000);

        fleft.setPower(0);
        fright.setPower(0);
        bleft.setPower(0);
        bright.setPower(0);
    }

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }





}
