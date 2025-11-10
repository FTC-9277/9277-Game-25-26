//Alex Alex, highest god of all
//To oppose him, no one has the gall
//Better than Ben, grayson, or Kevin
//         *pause*         six seven
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



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KevinRobot(hardwareMap, this);

        robot.shooter.setVelocity(0);

        waitForStart();
        while (opModeIsActive()) {
            double speed = 1;
            if (gamepad1.right_bumper) {
                speed = .25;
            }


            if (Math.abs(gamepad1.left_stick_y) >= 0.2 || Math.abs(gamepad1.left_stick_x) >= 0.2 || Math.abs(gamepad1.right_stick_x) >= 0.2) {
                robot.fleft.setPower(speed * (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
                robot.fright.setPower(speed * (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
                robot.bright.setPower(speed * (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
                robot.bleft.setPower(speed * (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
            } else {
                robot.fleft.setPower(0);
                robot.fright.setPower(0);
                robot.bright.setPower(0);
                robot.bleft.setPower(0);
            }


            // intake mech
            double intakeSpeed = 0;
            if (gamepad2.right_bumper) {
                intakeSpeed += 1;
            }
            if (gamepad2.left_bumper) {
                intakeSpeed -= 1;
            }
//            servo.setPower(intakeSpeed);


            // shooter mech
            if (gamepad2.left_bumper) {

                robot.shootBall();


            } else if (gamepad2.a) {

                robot.reverseBall();

            } else {
                robot.time.reset();
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
            }


        }


    }

}