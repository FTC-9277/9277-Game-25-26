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
    DcMotorEx shooter;

    double shooterSpeed = 0;

    public ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        //this is for testing of drive wheel
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setVelocity(0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            double speed=1;
            if (gamepad2.right_bumper) {
                speed = .25;
            }


            if(Math.abs(gamepad2.left_stick_y) >= 0.2 || Math.abs(gamepad2.left_stick_x) >= 0.2 || Math.abs(gamepad2.right_stick_x) >= 0.2) {
                fleft.setPower(speed * (gamepad2.left_stick_y-gamepad2.right_stick_x-gamepad2.left_stick_x));
                fright.setPower(speed * (gamepad2.left_stick_y+gamepad2.right_stick_x+gamepad2.left_stick_x));
                bright.setPower (speed * (gamepad2.left_stick_y+gamepad2.right_stick_x-gamepad2.left_stick_x));
                bleft.setPower(speed * (gamepad2.left_stick_y-gamepad2.right_stick_x+gamepad2.left_stick_x));
            } else {
                fleft.setPower(0);
                fright.setPower(0);
                bright.setPower(0);
                bleft.setPower(0);
            }



            if(gamepad2.b) {

                if (time.seconds() <= 5){
                    shooterSpeed = 320*time.seconds();
                    shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed",shooter.getVelocity());
                    telemetry.addData("Time", time);

                    telemetry.update();
                } else {
                    shooter.setVelocity(1600);
                }


            } else if (gamepad2.a) {

                if (time.seconds() <= 5){
                    shooterSpeed = -320*time.seconds();
                    shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed",shooter.getVelocity());
                    telemetry.addData("Time", time);

                    telemetry.update();
                } else {
                    shooter.setVelocity(-1600);
                }

            } else {
                time.reset();
                shooter.setVelocity(0);
            }



        }


    }
}