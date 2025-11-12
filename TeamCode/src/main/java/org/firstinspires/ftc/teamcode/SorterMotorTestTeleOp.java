package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "SorterMotorTest")
public class SorterMotorTestTeleOp extends LinearOpMode {

    DcMotorEx sorter;

    @Override
    public void runOpMode() throws InterruptedException {
        sorter = hardwareMap.get(DcMotorEx.class, "sorter");

        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad2.x) {
                sorter.setPower(0.5);
            } else {
                sorter.setPower(0);
            }

            if(gamepad2.a) {
                sorter.setTargetPosition(sorter.getTargetPosition()+3);
            }

            telemetry.addData("target pos", sorter.getTargetPosition());
            telemetry.update();
        }

    }
}
