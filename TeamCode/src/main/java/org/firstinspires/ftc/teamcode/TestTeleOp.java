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

    final int SEC_TO_SHOOTER_SPEED = 3;
    final int MAX_LAUNCH_SPEED = 1600;

    public ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        //adding servo


        //this is for testing of drive wheel
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setVelocity(0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            double speed=1;
            if (gamepad1.right_bumper) {
                speed = .25;
            }


            if(Math.abs(gamepad1.left_stick_y) >= 0.2 || Math.abs(gamepad1.left_stick_x) >= 0.2 || Math.abs(gamepad1.right_stick_x) >= 0.2) {
                fleft.setPower(speed * (gamepad1.left_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x));
                fright.setPower(speed * (gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x));
                bright.setPower (speed * (gamepad1.left_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x));
                bleft.setPower(speed * (gamepad1.left_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x));
            } else {
                fleft.setPower(0);
                fright.setPower(0);
                bright.setPower(0);
                bleft.setPower(0);
            }



            if(gamepad2.b) {

                if (time.seconds() <= SEC_TO_SHOOTER_SPEED){
                    telemetry.addData("slope",(double)(MAX_LAUNCH_SPEED  / SEC_TO_SHOOTER_SPEED));
                    shooterSpeed = ((double) MAX_LAUNCH_SPEED  / SEC_TO_SHOOTER_SPEED)*time.seconds();
                    shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed",shooter.getVelocity());
                    telemetry.addData("Motor set speed", shooterSpeed);
                    telemetry.addData("Time", time);


                    telemetry.update();
                } else {
                    shooter.setVelocity(MAX_LAUNCH_SPEED);
                }


            } else if (gamepad2.a) {

                if (time.seconds() <= SEC_TO_SHOOTER_SPEED){
                    shooterSpeed = -((double) MAX_LAUNCH_SPEED  / SEC_TO_SHOOTER_SPEED)*time.seconds();
                    shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed",shooter.getVelocity());
                    telemetry.addData("Time", time);

                    telemetry.update();
                } else {
                    shooter.setVelocity(-MAX_LAUNCH_SPEED);
                }

            } else {
                time.reset();
                shooter.setVelocity(0);
            }



        }


    }
}