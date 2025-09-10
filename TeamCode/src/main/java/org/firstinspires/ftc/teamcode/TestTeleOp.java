package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "First Teleop")
public class TestTeleOp extends LinearOpMode {

    DcMotor fleft, fright, bright, bleft;

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");


        while(opModeIsActive()) {

//            if(gamepad1.a) {
//                // move robot
//                fleft.setPower(1);
//            } else {
//                // stop robot
//                fleft.setPower(0);
//            }

            if(Math.abs(gamepad2.left_stick_y) == 0.2) {
                fleft.setPower(gamepad2.left_stick_y);
            } else {
                fleft.setPower(0);
            }

        }


    }
}