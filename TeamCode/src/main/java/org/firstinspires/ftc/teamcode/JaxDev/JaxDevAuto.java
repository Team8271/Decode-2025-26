package org.firstinspires.ftc.teamcode.JaxDev;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="JaxDevAuto", group="JaxDev")
public class JaxDevAuto extends LinearOpMode {
    JaxDevConfig robot;
    JaxDevConfig.Motif motif;
    JaxDevConfig.Motif curMotif;

    @Override
    public void runOpMode(){
        robot = new JaxDevConfig(this);
        robot.init();
        robot.initTweetyBird();

        telemetry.addData(">","Robot Ready.  Press Play.");

        while(opModeInInit()){
            motif = robot.scanObelisk();
            telemetry.addData("Motif", motif);
        }
        telemetry.update();
        waitForStart();

        // Move to Launch

        // Launch Preloads

        // Scan Obelisk while moving towards spike marks
        curMotif = robot.scanObelisk();
        if(curMotif != JaxDevConfig.Motif.NULL){
            motif=curMotif;
        }

        // Grab artifacts from motif spike mark
        switch(motif){
            case GPP:
                // Grab GPP spike mark
                break;
            case PGP:
                // Grab PGP spike mark
                break;
            case PPG:
                // Grab PPG spike mark
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
