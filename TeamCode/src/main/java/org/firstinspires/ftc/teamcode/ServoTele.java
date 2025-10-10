package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tele")

public class ServoTele extends LinearOpMode {

    public CRServo testServo;


    @Override
    public void runOpMode() throws InterruptedException {


        testServo = hardwareMap.get(CRServo.class,"servo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && gamepad1.left_bumper){
                testServo.setPower(1.0);

            } else if (gamepad1.b) {
                testServo.setPower(-1.0);
            }else {
                testServo.setPower(0.0);
            }
            telemetry.addData("servopower", testServo.getPower());
            telemetry.update();
        }

    }
}
