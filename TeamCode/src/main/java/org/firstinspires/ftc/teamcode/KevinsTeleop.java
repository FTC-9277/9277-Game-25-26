package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "First Teleop")
public class KevinsTeleop extends LinearOpMode {

    DcMotor fleft, fright, bright, bleft;
    DcMotorEx shooter, sorter;
    Servo servoDoor;
    double shooterSpeed = 0;

    final double SORTER_ENCODER_RESOLUTION = 20;
    final double SORTER_SCALE_PER_TICK = 360 / SORTER_ENCODER_RESOLUTION;
    final double SORTER_TICKS_PER_120_DEG = 120 * (SORTER_ENCODER_RESOLUTION / 360);

    final int SEC_TO_SHOOTER_SPEED = 3;
    final int MAX_LAUNCH_SPEED = 1350;

    public ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        sorter = hardwareMap.get(DcMotorEx.class, "sorter");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        //adding servo
        servoDoor = hardwareMap.get(Servo.class, "servo");

        //this is for testing of drive wheel
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setVelocity(0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while (opModeIsActive()) {
            double speed = 1;
            if (gamepad1.right_bumper) {
                speed = .25;
            }


            if (Math.abs(gamepad1.left_stick_y) >= 0.2 || Math.abs(gamepad1.left_stick_x) >= 0.2 || Math.abs(gamepad1.right_stick_x) >= 0.2) {
                fleft.setPower(speed * (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
                fright.setPower(speed * (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
                bright.setPower(speed * (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
                bleft.setPower(speed * (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            } else {
                fleft.setPower(0);
                fright.setPower(0);
                bright.setPower(0);
                bleft.setPower(0);
            }

            double intakeSpeed = 0;
            if (gamepad2.right_bumper) {
                intakeSpeed += 1;
            }
            if (gamepad2.left_bumper) {
                intakeSpeed -= 1;
            }
//            servo.setPower(intakeSpeed);
            if (gamepad2.left_bumper) {

                if (time.seconds() <= SEC_TO_SHOOTER_SPEED) {
                    telemetry.addData("slope", (double) (MAX_LAUNCH_SPEED / SEC_TO_SHOOTER_SPEED));
                    shooterSpeed = ((double) MAX_LAUNCH_SPEED / SEC_TO_SHOOTER_SPEED) * time.seconds();
                    shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed", shooter.getVelocity());
                    telemetry.addData("Motor set speed", shooterSpeed);
                    telemetry.addData("Time", time);


                    telemetry.update();
                } else {
                    shooter.setVelocity(MAX_LAUNCH_SPEED);
                    telemetry.addData("max speed", MAX_LAUNCH_SPEED);
                    telemetry.update();
                }


            } else if (gamepad2.a) {

                if (time.seconds() <= SEC_TO_SHOOTER_SPEED) {
                    shooterSpeed = -((double) MAX_LAUNCH_SPEED / SEC_TO_SHOOTER_SPEED) * time.seconds();
                    shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed", shooter.getVelocity());
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