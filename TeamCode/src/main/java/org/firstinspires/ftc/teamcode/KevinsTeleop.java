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
                robot.fleft.setPower(speed * (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
                robot.fright.setPower(speed * (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
                robot.bright.setPower(speed * (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
                robot.bleft.setPower(speed * (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
            } else {
                robot.fleft.setPower(0);
                robot.fright.setPower(0);
                robot.bright.setPower(0);
                robot.bleft.setPower(0);
            }


//            // intake mech
//            double intakeSpeed = 0;
//            if (gamepad2.right_bumper) {
//                intakeSpeed += 1;
//            }
//            if (gamepad2.left_bumper) {
//                intakeSpeed -= 1;
//            }
//            servo.setPower(intakeSpeed);



//            if (gamepad2.right_bumper) {
//                robot.servoDoor.setPosition(0);
//              }  else {
//                robot.servoDoor.setPosition(.17);
//            }
//
//            if (gamepad2.left_bumper){
//                robot.shootBall();
//            } else {
//                robot.shooter.setVelocity(0);
//                robot.shooter2.setVelocity(0);
//                robot.time.reset();
//            }
//
//            if (gamepad2.xWasPressed()) {
//                robot.turnToPosition((robot.getSorterPosition()+1) % 3);
////                robot.sorter.setTargetPosition(20);
//
//            }
//
//            if (gamepad2.dpadDownWasPressed()){
//                robot.adjustSorterDown();
//            }
//
//            if (gamepad2.dpadUpWasPressed()){
//                robot.adjustSorterUp();
//            }
//
//            //if (gamepad2.bWasPressed()){
//               // robot.emergencyButtonPressed=true;
//           //
//
//            telemetry.addData("target position", robot.sorter.getTargetPosition());
//            telemetry.addData("current position", robot.sorter.getCurrentPosition());

            telemetry.update();

        }


    }

}