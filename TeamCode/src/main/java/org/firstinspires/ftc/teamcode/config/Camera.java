package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

/**
 *
 */
public class Camera extends Thread{

    Config robot;

    public void setConfig(Config robot) {
        this.robot = robot;
    }

    private volatile boolean running = true; // When false, thread terminates

    boolean scanGoalAngle = false;
    boolean scanObelisk = false;

    @Override
    public void run() {
        while (running) {
            synchronized (this) {
                try {
                    wait(); // Sleep until notified (Save resources)
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            if (!running) break; // Check before doing work

            if (scanGoalAngle) {
                doScanGoalAngle();
            }
            if (scanObelisk) {
                doScanObelisk();
            }

        }
    }

    /// Public class that updates goalTx and goalTy
    public synchronized void scanGoalAngle() {
        scanGoalAngle = true;
        notify();
    }

    /// Public class that updates motif
    public synchronized void scanObelisk() {
        scanObelisk = true;
        notify();
    }


    public synchronized void terminate() {
        running = false;
        notify();
    }

    /// Private method to update goal angles
    private void doScanGoalAngle() {

        if (robot.alliance == Config.Alliance.RED) {
            switchPipeline(Pipeline.RED_GOAL);
        } else {
            switchPipeline(Pipeline.BLUE_GOAL);
        }

        LLResult result = getResult();

        // Update angles
        if (result != null && result.isValid()) {
            robot.goalTx = result.getTx();
            robot.goalTy = result.getTy();
            robot.goalAnglesAreFresh = true;
            log("Updated Goal Angles");
        }
        else {
            robot.goalAnglesAreFresh = false;
            log("Failed to update Goal Angles");
        }
    }

    /// Private method to update motif
    private void doScanObelisk() {

        int aprilTag = 0;
        Config.Motif tempMotif;

        switchPipeline(Pipeline.OBELISK);

        LLResult result = getResult();

        if (result != null && result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            aprilTag = fiducialResults.get(0).getFiducialId();
        }

        switch (aprilTag) {
            case (21):
                tempMotif = Config.Motif.GPP;
                break;
            case (22):
                tempMotif = Config.Motif.PGP;
                break;
            case (23):
                tempMotif = Config.Motif.PPG;
                break;
            default:
                tempMotif = Config.Motif.NULL;
                break;
        }
        if (tempMotif != Config.Motif.NULL) {
            log("Motif changed to: " + robot.motif);
            robot.motif = tempMotif;
        }
    }

    /// Private method to start limelight data polling
    private void startLimelight() {
        if(!robot.limelight.isRunning()) {
            robot.log("Limelight: Starting");
            robot.limelight.start();
            robot.limelight.setPollRateHz(11);
        }
    }

    /// Private method to stop limelight data polling
    private void stopLimelight() {
        if(robot.limelight.isRunning()) {
            robot.log("Limelight: Stopping");
            robot.limelight.stop();
        }
    }

    private enum Pipeline {
        LOCALIZATION,
        OBELISK,
        RED_GOAL,
        BLUE_GOAL;
    }

    /// Private method to change limelight pipeline
    private void switchPipeline(Pipeline pipeline) {
        startLimelight();

        switch (pipeline) {
            case LOCALIZATION:
                if (robot.limelight.getStatus().getPipelineIndex() != 0) {
                    robot.limelight.pipelineSwitch(0);
                    log("Switching Pipeline: LOCALIZATION");
                }
                break;
            case OBELISK:
                if (robot.limelight.getStatus().getPipelineIndex() != 1) {
                    robot.limelight.pipelineSwitch(1);
                    log("Switching Pipeline: OBELISK");
                }
                break;
            case RED_GOAL:
                if (robot.limelight.getStatus().getPipelineIndex() != 2) {
                    robot.limelight.pipelineSwitch(2);
                    log("Switching Pipeline: RED_GOAL");
                }
                break;
            case BLUE_GOAL:
                if (robot.limelight.getStatus().getPipelineIndex() != 3) {
                    robot.limelight.pipelineSwitch(3);
                    log("Switching Pipeline: BLUE_GOAL");
                }
                break;
        }
    }

    private LLResult getResult() {
        startLimelight();
        return robot.limelight.getLatestResult();
    }

    private void log(String message) {
        robot.log("[Camera] - " + message);
    }
}