package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.Config;

@Autonomous(name = "Auto", preselectTeleOp = "SOLO: TeleOp")
public class Auto extends LinearOpMode {
    Config robot;
    int startDelay = 0;
    enum StartingPosition{
        POSITION1,
        POSITION2,
        POSITION2CORRECT;
    }
    Config.Alliance alliance;
    StartingPosition startingPosition = StartingPosition.POSITION2CORRECT;

    @Override
    public void runOpMode() {
        robot = new Config(this);
        robot.init();
        robot.initTweetyBird();

        telemetry.addLine("Initialized");
        telemetry.addLine(" == Auto Selector Here == ");
        telemetry.update();

        alliance = robot.alliance;

        waitForStart();

        telemetry.addData("Currently in Start Delay",startDelay);
        telemetry.update();
        sleep(startDelay);

        switch (startingPosition){
            case POSITION1:
                position1Auto();
                break;
            case POSITION2:
                position2Auto();
                break;
            case POSITION2CORRECT:
                position2AutoCorrect();
                break;
        }


    }

    void position1Auto() {
        robot.tweetyBird.engage();
        robot.agitator.setPower(robot.agitatorActivePower);
        robot.tweetyBird.addWaypoint(-1,50,0);
        robot.tweetyBird.addWaypoint(-2,84,132);
        robot.tweetyBird.waitWhileBusy();
        robot.launcherThread.launch(3);
        robot.launcherThread.waitWhileBusy();
        robot.tweetyBird.addWaypoint(-1,50,132);
        robot.tweetyBird.waitWhileBusy();
    }

    void position2Auto() {
        robot.tweetyBird.engage();
        robot.agitator.setPower(robot.agitatorActivePower);
        robot.tweetyBird.addWaypoint(1,50,0);
        robot.tweetyBird.addWaypoint(2,84,-120);
        robot.tweetyBird.waitWhileBusy();
        robot.launcherThread.launch(3);
        robot.launcherThread.waitWhileBusy();
        robot.tweetyBird.addWaypoint(1,50,-120);
        robot.tweetyBird.waitWhileBusy();
    }

    void position2AutoCorrect() {
        robot.tweetyBird.engage();
        robot.agitator.setPower(robot.agitatorActivePower);
        robot.tweetyBird.addWaypoint(1,50,0);
        robot.tweetyBird.addWaypoint(2,84,-120);
        robot.tweetyBird.waitWhileBusy();
        robot.limelightThread.startGoalCorrection();
        robot.limelightThread.waitWhileCorrecting();
        robot.launcherThread.launch(3);
        robot.launcherThread.waitWhileBusy();
        robot.tweetyBird.addWaypoint(1,50,-120);
        robot.tweetyBird.waitWhileBusy();
    }
}
