package org.firstinspires.ftc.teamcode;
//  if (code not work) work;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestRedFrontAuto")
public class KevinsAuto3 extends LinearOpMode {

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


//        robot.drivingForwardMM(60,0.25);
//        robot.streftAndStrightMM(-400,0.5);
//        robot.drivingForwardMM(2467,0.5);
//        robot.turnBy(40,1000);

        robot.drivingForwardMM(60,.25);
        robot.autoShoot3Balls();
        robot.drivingForwardMM(467,.5);
        // auto for red back, **untested** *a
        //robot shoot

    }

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }

}
