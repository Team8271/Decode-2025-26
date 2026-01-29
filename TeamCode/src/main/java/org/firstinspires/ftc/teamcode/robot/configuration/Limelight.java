package org.firstinspires.ftc.teamcode.robot.configuration;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Limelight {
    Config robot;

    double timeSincePipelineChange = 0;
    ElapsedTime pipelineChangeTime = new ElapsedTime();

    public Limelight(Config robot) {this.robot = robot;}

    private enum Pipeline {
        LOCALIZATION(0),
        OBELISK(1),
        RED_GOAL(2),
        BLUE_GOAL(3);

        private final int pipeline;
        Pipeline(int pipeline) { this.pipeline = pipeline; }
        public int getValue() { return pipeline; }
    }

    /**
     * Checks if limelight is currently running.
     * If not: start limelight and polling. Already
     * running? Do nothing.
     */
    private void startLimelight() {
        // Ensure polling for limelight data
        if (!robot.limelightCamera.isRunning()) {
            robot.log("Limelight: Starting");
            robot.limelightCamera.setPollRateHz(11);
            robot.limelightCamera.start();
            pipelineChangeTime.reset();
        }
    }

    private void changePipeline(Pipeline pipeline) {
        // Set desired pipeline
        if(timeSincePipelineChange < 15) {
            try {
                Thread.sleep(15);
            } catch (InterruptedException e) {
                log("Pipeline Change Error: " + e);
            }

        }
        if (robot.limelightCamera.getStatus().getPipelineIndex() != pipeline.getValue()) {
            robot.limelightCamera.pipelineSwitch(pipeline.getValue());
        }
        timeSincePipelineChange = pipelineChangeTime.milliseconds();
    }

    double lastTx = 0;

    public double scanGoalTx() {
        startLimelight();

        if(robot.alliance == Config.Alliance.BLUE) {
            changePipeline(Pipeline.BLUE_GOAL);
        }
        else {
            changePipeline(Pipeline.RED_GOAL);
        }

        // Get latest results
        LLResult result = robot.limelightCamera.getLatestResult();

        if(result != null && result.isValid()) {
            log("ScanGoalTx = " + result.getTx());
            lastTx = result.getTx();
            return result.getTx();
        }
        log("ScanGoalTx = d " + lastTx);
        return lastTx;
    }

    private void log(String message) {
        robot.log("[Limelight] - " + message);
    }

}
