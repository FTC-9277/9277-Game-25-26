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
    KevinRobot robot;

    double shooterSpeed = 0;

    final int SEC_TO_SHOOTER_SPEED = 3;
    final int MAX_LAUNCH_SPEED = 1350;

    public ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KevinRobot(hardwareMap, this);


        waitForStart();
        while (opModeIsActive()) {
            double speed = 1;
            if (gamepad1.right_bumper) {
                speed = .25;
            }


            if (Math.abs(gamepad1.left_stick_y) >= 0.2 || Math.abs(gamepad1.left_stick_x) >= 0.2 || Math.abs(gamepad1.right_stick_x) >= 0.2) {
                robot.fleft.setPower(speed * (gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
                robot.fright.setPower(speed * (gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
                robot.bright.setPower(speed * (gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
                robot.bleft.setPower(speed * (gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
            } else {
                robot.fleft.setPower(0);
                robot.fright.setPower(0);
                robot.bright.setPower(0);
                robot.bleft.setPower(0);
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
                    robot.shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed", robot.shooter.getVelocity());
                    telemetry.addData("Motor set speed", shooterSpeed);
                    telemetry.addData("Time", time);


                    telemetry.update();
                } else {
                    robot.shooter.setVelocity(MAX_LAUNCH_SPEED);
                    telemetry.addData("max speed", MAX_LAUNCH_SPEED);
                    telemetry.update();
                }


            } else if (gamepad2.a) {

                if (time.seconds() <= SEC_TO_SHOOTER_SPEED) {
                    shooterSpeed = -((double) MAX_LAUNCH_SPEED / SEC_TO_SHOOTER_SPEED) * time.seconds();
                    robot.shooter.setVelocity(shooterSpeed);
                    telemetry.addData("Motor speed", robot.shooter.getVelocity());
                    telemetry.addData("Time", time);

                    telemetry.update();
                } else {
                    robot.shooter.setVelocity(-MAX_LAUNCH_SPEED);
                }

            } else {
                time.reset();
                robot.shooter.setVelocity(0);
            }


        }


    }

}