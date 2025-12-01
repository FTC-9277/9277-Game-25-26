package org.firstinspires.ftc.teamcode;
//  if (code not work) work;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestBlueBackAuto")
public class KevinAuto2 extends LinearOpMode {

    KevinRobot robot;
    //Benjamin Benjamin the god of CAD
//He's so locked in the rest of us feel bad
//His robot designs are done so well
//However, the building team is hell
//His CAD so good he leaves us in awe
//But the build team is the worst I ever saw
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KevinRobot(hardwareMap, this);
        waitForStart();

        //This is the actual auto
        robot.drivingForwardMM(1067,.5);
        robot.turnBy(164.67, 1000);
        robot.streftAndStrightMM(55, 0.3);
    } // fixed *a

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }

}
