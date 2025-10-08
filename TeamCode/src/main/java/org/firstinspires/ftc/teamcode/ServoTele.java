package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tele")

class ServoTele extends LinearOpMode {

    Servo testServo;


    @Override
    public void runOpMode() throws InterruptedException {


        testServo = hardwareMap.get(Servo.class,"servo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x){
                testServo.setPosition(testServo.getPosition()+.005);

            } else if (gamepad1.b) {
                testServo.setPosition(testServo.getPosition()-.005);
            }
            telemetry.addData("servoLocation", testServo.getPosition());
            telemetry.update();
        }

    }
}
