package org.firstinspires.ftc.teamcode;
//  if (code not work); work
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestAuto")
public class KevinsAuto extends LinearOpMode {

    KevinRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KevinRobot(hardwareMap, this);
        waitForStart();

        //This is the actual auto
        robot.drivingForwardMM(267, .5);
        waitTime(670);
        robot.streftAndStrightMM(670,.5);
        waitTime(67);
        robot.turnBy(28.67,(670/2.0));

    }

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }

}
