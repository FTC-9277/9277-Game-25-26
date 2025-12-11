package org.firstinspires.ftc.teamcode;
//  if (code not work) work;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        robot.maxLaunchSpeed=850;
        waitForStart();

        //This is the actual auto
        robot.drivingForwardMM(-1367,.5);
//        robot.turnBy(164.67, 1000);
//        robot.streftAndStrightMM(55, 0.3);

//        robot.setSafePower(robot.sorter,0.075);
//        robot.shootBall();
//        robot.servoDoor.setPosition(0.18);
//        waitTime(1000);
//
//        while (robot.getSorterPosition() <= 3 && opModeIsActive()){
//            robot.setSafePower(robot.sorter,0.15);
//            waitTime(50);
//            robot.setSafePower(robot.sorter,0.075);
//            waitTime(5000);
//
//        }


        robot.shootBall();
        waitTime(670);
        //robot.servoDoor.setPosition(0.18);

        //origional for loop
        for (int i = 1; i <=4; i++){
            waitTime(1000);

            robot.turnToPosition(i); //<--- Kevin Kevin is right here
//            robot.sorter.setPower(0.3);
//            waitTime(100);
////            robot.setSafePower(robot.sorter, -.3);
////            waitTime(100);
//            robot.sorter.setPower(0);
            telemetry.addData("Count", i);
            telemetry.update();
        }
        robot.servoDoor.setPosition(0.05);
        robot.shooter.setVelocity(0);
        robot.shooter2.setVelocity(0);
    } // fixed *a

    public void waitTime (int time) {
        long targetTime = System.currentTimeMillis()+time;
        while(opModeIsActive() && System.currentTimeMillis()<targetTime) {}
    }

}
