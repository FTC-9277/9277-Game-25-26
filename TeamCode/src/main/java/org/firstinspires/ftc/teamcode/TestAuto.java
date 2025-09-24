package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    DcMotor fleft, fright, bright, bleft;
    Servo servo;

    final int WHEEL_DIAMETER = 96;
    final double WHEEL_ENCODER_RESOLUTION = 384.5;


    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        servo = hardwareMap.get(Servo.class, "serv");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset ALLLL of the encoders
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        // steps!

        // Step 1: Drive forward at a certain speed
        // Step 2: Wait UNTIL encoders reach a specified value
        // Step 3: Stop
//        drivingForward(500, .2);

        drivingForwardMM(1000,0.2);
    }

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }
    public void drivingForward (int target, double power) {



        target += averageTicks();
        while (Math.abs(target-averageTicks()) > 5) {
            int sign = (target-averageTicks())/Math.abs(target-averageTicks());
            fleft.setPower(power*sign);
            fright.setPower(power*sign);
            bright.setPower(power*sign);
            bleft.setPower(power*sign);

            telemetry.addData("Encoder",averageTicks());
            telemetry.addData("Power",power*sign);
            telemetry.update();

        }

        fleft.setPower(0);
        fright.setPower(0);
        bright.setPower(0);
        bleft.setPower(0);

    }

    public int averageTicks (){
        return (fleft.getCurrentPosition()+ fright.getCurrentPosition()+ bright.getCurrentPosition()+ bleft.getCurrentPosition())/4;
    }

    public void drivingForwardMM(int targetMillimeters, double power) {
        double circumference = WHEEL_DIAMETER*Math.PI;
        int targetTicks = (int)(targetMillimeters*(WHEEL_ENCODER_RESOLUTION/circumference));

        drivingForward(targetTicks, power);
    }
    public void driveUntilMilli (int milli, double power) {
        fleft.setPower(power);
        fright.setPower(power);
        bright.setPower(power);
        bleft.setPower(power);

        waitTime(milli);

        fleft.setPower(0);
        fright.setPower(0);
        bleft.setPower(0);
        bright.setPower(0);
    }





}
