package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "First Teleop")
public class TestTeleOp extends LinearOpMode {

    DcMotor fleft, fright, bright, bleft;
    DcMotorEx testMotor;
    Servo servo;

    double speed = 0;

    public ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotor.class, "fleft");
        //fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        servo = hardwareMap.get(Servo.class, "serv");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        //fright.setDirection(DcMotorSimple.Direction.REVERSE);

        //this is for testing of drive wheel
        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        testMotor.setVelocity(0);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while(opModeIsActive()) {
            double speed=1;
            if (gamepad2.right_bumper) {
                speed = .25;
            }


            if(Math.abs(gamepad2.left_stick_y) >= 0.2 || Math.abs(gamepad2.left_stick_x) >= 0.2 || Math.abs(gamepad2.right_stick_x) >= 0.2) {
                fleft.setPower(speed * (gamepad2.left_stick_y-gamepad2.right_stick_x+gamepad2.left_stick_x));
                //fright.setPower(speed * (gamepad2.left_stick_y+gamepad2.right_stick_x-gamepad2.left_stick_x));
                bright.setPower (speed * (gamepad2.left_stick_y+gamepad2.right_stick_x+gamepad2.left_stick_x));
                bleft.setPower(speed * (gamepad2.left_stick_y-gamepad2.right_stick_x-gamepad2.left_stick_x));
            } else {
                fleft.setPower(0);
                //fright.setPower(0);
                bright.setPower(0);
                bleft.setPower(0);
            }


            if(gamepad2.y) {
                servo.setPosition(1);
            } else {
                servo.setPosition(-1);
            }

            while(gamepad2.b) {

                if (time.seconds() <= 10){
                    speed = 6*time.seconds();
                    testMotor.setVelocity(speed);
                    telemetry.addData("Motor speed",speed);
                    telemetry.addData("Time", time);

                    telemetry.update();
                } else {
                    testMotor.setVelocity(60);
                }

            }
            time.reset();
            testMotor.setVelocity(0);


        }


    }
}