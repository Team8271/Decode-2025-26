package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    volatile boolean scanGoalAngle = false;
    volatile boolean scanObelisk = false;
    volatile boolean correcting = false;

    ElapsedTime runtime;
    double lastTime;

    @Override
    public void run() {
        runtime = new ElapsedTime();
        runtime.reset();
        while (running && !robot.opMode.isStopRequested()) {
            synchronized (this) {
                try {
                    wait(); // Sleep until notified (Save resources)
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            if (!running) break; // check before doing work

            if (scanGoalAngle && (runtime.time() > lastTime+0.1)) {
                lastTime = runtime.time();
                doScanGoalAngle();
                scanGoalAngle = false;
            }

            if(correcting) {
                doGoalCorrection(); // Sets correcting false on its own
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
                selectedPipeline = robot.limelightBluePipeline;
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
            robot.avgDist = result.getBotposeAvgDist();
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

    public synchronized void startGoalCorrection() {
        correcting = true;
        notify();
    }
    public synchronized void terminateGoalCorrection() {
        correcting = false;
        robot.setWheelPowers(0,0,0,0);
    }
    public synchronized boolean isGoalCorrectionDone() {
        return correcting;
    }
    public synchronized void waitWhileCorrecting() {
        robot.opMode.sleep(100);
        while (correcting && robot.opMode.opModeIsActive());
    }

    private void doGoalCorrection() {

        while(correcting && running && !robot.opMode.isStopRequested()) {

            double deadband = 2.0;      // Stop correcting when within 2 degree(s)
            double maxError = 10.0;     // Full speed when 10 degrees or more off
            double minPower = 0.1;      // Smallest power that still moves the wheels
            double targetAngle = 5.0;   // Aim 5° to the left of the target (reversed directions)

            double error = robot.goalTx - targetAngle; // Angular error
            double absError = Math.abs(error);

            // Scale power by distance to goal
            double scale = Math.min(absError / maxError, 1.0);
            double power = Math.max(0.1, robot.autoRotateSpeed * scale);

            if (absError <= deadband) {
                // Close enough: stop correcting
                correcting = false;
                robot.setWheelPowers(0, 0, 0, 0);
            }
            else if (error < 0) {
                // Rotate Left
                robot.setWheelPowers(-power, power, -power, power);
            }
            else {
                // Rotate Right
                robot.setWheelPowers(power, -power, power, -power);
            }
        }
        robot.setWheelPowers(0,0,0,0);

    }
    public synchronized void terminate() {
        running = false;
        notify();
    }

}
