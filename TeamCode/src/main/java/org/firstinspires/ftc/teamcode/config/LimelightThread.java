package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

/**
 *
 */
public class LimelightThread extends Thread {
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

            if (!running) break; // check before doing work

            if (scanGoalAngle) {
                doScanGoalAngle();
            }

        }
    }

    /// Limelight updates goalTx and goalTy
    public synchronized void scanGoalAngle() {
        scanGoalAngle = true;
        notify();
    }

    /// Limelight updates motif
    public synchronized void scanObelisk() {
        scanObelisk = true;
        notify();
    }

    /// Uses Limelight to detect Goal angle and update goalTx and goalTy.
    private void doScanGoalAngle() {
        int selectedPipeline;

        // Ensure polling for limelight data
        if (!robot.limelight.isRunning()) {
            robot.log("Limelight: Starting");
            robot.limelight.start();
        }

        // Set desired pipeline
        switch (robot.alliance) {
            case BLUE:
                selectedPipeline = robot.limelightBluePipeline;
                break;
            case RED:
                selectedPipeline = robot.limelightRedPipeline;
                break;
            default:
                selectedPipeline = 4;
                break;
        }

        // Set pipeline
        if (robot.limelight.getStatus().getPipelineIndex() != selectedPipeline) {
            robot.log("Limelight: Pipeline change: " + selectedPipeline);
            robot.limelight.pipelineSwitch(selectedPipeline);
        }

        // Get latest results
        LLResult result = robot.limelight.getLatestResult();

        // Update angles
        if (result != null && result.isValid()) {
            robot.goalAnglesAreValid = true;
            robot.goalTx = result.getTx();
            robot.goalTy = result.getTy();
        } else {
            robot.goalAnglesAreValid = false;
        }

    }

    public void doScanObelisk() {
        int aprilTag = 0;
        Config.Motif tempMotif;

        if (!robot.limelight.isRunning()) {
            robot.log("Limelight: Starting");
            robot.limelight.start();
        }
        if (robot.limelight.getStatus().getPipelineIndex() != robot.limelightObeliskPipeline) {
            robot.log("Limelight: Pipeline changed to OBELISK");
            robot.limelight.pipelineSwitch(robot.limelightObeliskPipeline);
        }
        LLResult result = robot.limelight.getLatestResult();

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
            robot.log("Motif changed to: " + robot.motif);
            robot.motif = tempMotif;
        }

    }

    public synchronized void terminate() {
        running = false;
        notify();
    }
}
