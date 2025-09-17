package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "First Teleop")
public class TestTeleOp extends LinearOpMode {

    DcMotor fleft, fright, bright, bleft;
    Servo servo ;

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
        while(opModeIsActive()) {



            if(Math.abs(gamepad2.left_stick_y) >= 0.2 || Math.abs(gamepad2.left_stick_x) >= 0.2 || Math.abs(gamepad2.right_stick_x) >= 0.2) {
                fleft.setPower(gamepad2.left_stick_y-gamepad2.right_stick_x+gamepad2.left_stick_x);
                fright.setPower(gamepad2.left_stick_y+gamepad2.right_stick_x-gamepad2.left_stick_x);
                bright.setPower(gamepad2.left_stick_y+gamepad2.right_stick_x+gamepad2.left_stick_x);
                bleft.setPower(gamepad2.left_stick_y-gamepad2.right_stick_x-gamepad2.left_stick_x);
            } else {
                fleft.setPower(0);
                fright.setPower(0);
                bright.setPower(0);
                bleft.setPower(0);
            }
            if(gamepad2.y) {
                servo.setPosition(1);
            } else {
                servo.setPosition(-1);
            }


        }


    }
}