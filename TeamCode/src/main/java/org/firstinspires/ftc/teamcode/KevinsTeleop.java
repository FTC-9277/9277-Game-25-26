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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "First Teleop")
public class KevinsTeleop extends LinearOpMode {
    KevinRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KevinRobot(hardwareMap, this);

       // robot.shooter.setVelocity(0);

        waitForStart();
        while (opModeIsActive()) {
            double speed = 1;
            if (gamepad1.right_bumper) {
                speed = .25;
            }


            if (Math.abs(gamepad1.left_stick_y) >= 0.2 || Math.abs(gamepad1.left_stick_x) >= 0.2 || Math.abs(gamepad1.right_stick_x) >= 0.2) {
                robot.setSafePower(robot.fleft,speed * (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
                robot.setSafePower(robot.fright,speed * (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
                robot.setSafePower(robot.bright,speed * (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
                robot.setSafePower(robot.bleft,speed * (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            } else {
                robot.fleft.setPower(0);
                robot.fright.setPower(0);
                robot.bright.setPower(0);
                robot.bleft.setPower(0);
            }

            if (Math.abs(gamepad2.right_stick_y) >= 0.2){
                robot.sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.sorter.setPower(-gamepad2.right_stick_y/4);
            }else {
                robot.sorter.setPower(0);
            }

//            // intake mech
//            double intakeSpeed = 0;
//            if (gamepad2.right_bumper) {
//                intakeSpeed += 1;
//            }
//            if (gamepad2.left_bumper) {
//                intakeSpeed -= 1;
//            }
//           // servo.setPower(intakeSpeed);



            if (gamepad2.right_bumper) {
                robot.servoDoor.setPosition(0.2);
              }  else {
                robot.servoDoor.setPosition(0.05);
            }

            if (gamepad2.left_bumper){
                robot.shootBall();
            } else {
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
                robot.time.reset();
            }

//            if (gamepad2.xWasPressed()) {
//                robot.turnToPosition((robot.getSorterPosition()+1) % 3);
////                robot.sorter.setTargetPosition(20);
//
//            }

            if (gamepad2.dpad_down){
                robot.adjustSorterDown();
            }

            if (gamepad2.dpad_up){
                robot.adjustSorterUp();
            }

            if (gamepad1.left_bumper){
                robot.maxLaunchSpeed=1075;
            }
            if (Math.abs(gamepad1.left_trigger) >= .2){
                robot.maxLaunchSpeed=1275;
            }

//
//            if (gamepad2.bWasPressed()){
//                robot.emergencyButtonPressed=true;
//           }
            telemetry.addData("Sorter Num Position", robot.getSorterPosition());
            telemetry.addData("target position", robot.sorter.getTargetPosition());
            telemetry.addData("current position", robot.sorter.getCurrentPosition());
            telemetry.addData("Shooter PID", robot.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Shooter2 PID", robot.shooter2.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Sorter PID", robot.sorter.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Max Launch Speed", robot.maxLaunchSpeed);
            telemetry.update();

        }


    }

}