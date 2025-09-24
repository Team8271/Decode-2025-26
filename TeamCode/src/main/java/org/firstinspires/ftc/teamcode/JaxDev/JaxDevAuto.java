package org.firstinspires.ftc.teamcode.JaxDev;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="JaxDevAuto", group="JaxDev")
public class JaxDevAuto extends LinearOpMode {
    JaxDevConfig robot;
    JaxDevConfig.Motif curMotif;

    @Override
    public void runOpMode(){
        robot = new JaxDevConfig(this);
        robot.init();
        robot.initTweetyBird();

        while(opModeInInit() && !isStopRequested()){
            telemetry.addData(">","Robot Ready.  Press Play.");
            robot.scanObelisk();
            telemetry.addData("Motif", robot.motif);
            telemetry.update();
        }
        telemetry.update();

        waitForStart();

        // Move to Launch

        // Launch Preloads

        // Scan Obelisk while moving towards spike marks
        //robot.scanObelisk();
        robot.tweetyBird.engage();

        // Grab artifacts from motif spike mark
        switch(robot.motif){
            case GPP:
                // Grab GPP spike mark
                robot.tweetyBird.addWaypoint(23,24,94);
                robot.tweetyBird.waitWhileBusy();
                break;
            case PGP:
                // Grab PGP spike mark
                robot.tweetyBird.addWaypoint(24,49,94);
                robot.tweetyBird.waitWhileBusy();
                break;
            case PPG:
                // Grab PPG spike mark
                robot.tweetyBird.addWaypoint(24,74,90);
                robot.tweetyBird.waitWhileBusy();
                break;
            case NULL:
                // Grab PPG spike mark
                // Closest to goal
                break;
        }

        // Move to launch

        // Launch spike artifacts


    }

}
