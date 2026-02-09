package org.firstinspires.ftc.teamcode.robot.configuration;

import com.pedropathing.geometry.Pose;

public class AimAssist {

    private boolean simpleMode = false;

    /**
     * @return true if active
     */
    public boolean getSimpleStatus() {
        return simpleMode;
    }

    /**
     * Enables simple mode for all AimAssist methods.
     */
    public void enableSimpleMode() {
        simpleMode = true;
    }

    /**
     * Disable simple mode for all AimAssist methods.
     */
    public void disableSimpleMode() {
        simpleMode = false;
    }

    public double getPoseDistance(Pose p1, Pose p2) {
        double dx = p1.getPose().getX() - p2.getPose().getX();
        double dy = p1.getPose().getY() - p2.getPose().getY();
        double dz = p1.getPose().getHeading() - p2.getPose().getHeading();


        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    Config robot;

    public HeadingPIDF headingPIDF;

    public AimAssist(Config robot, double kP, double kI, double kD, double kF) {
        this.robot = robot;
        headingPIDF = new HeadingPIDF(kP, kI, kD, kF);
        log("Ready.");
    }

    /**
     * Calculates desired heading to face a position on a pedro-field. <break></break>
     * <b>Simple Mode:</b> returns BLUE=135deg  RED=35deg (In Radians)
     * @param currentPose Robot current position on the field <b>(must be correct)</b>
     * @param targetToFace Pose of the desired position to be facing
     * @return The correct heading in radians to face the target Pose
     * @SimpleMode returns BLUE=135deg  RED=35deg (In Radians)
     */
    public double getHeadingForTarget(Pose currentPose, Pose targetToFace) {

        if (simpleMode) {
            return (robot.alliance == Config.Alliance.RED ? 35 : 135);
        }

        // Robot's current state
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();

        // Target coordinates
        double targetX = targetToFace.getX();
        double targetY = targetToFace.getY();

        // Calculate the target heading in the field frame
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double targetHeading = Math.atan2(deltaY, deltaX);

        if (Math.toDegrees(targetHeading) < 13) {targetHeading = 1;}
        if (Math.toDegrees(targetHeading) > 167) {targetHeading = 179;}
        return targetHeading;
    }

    /**
     * Wraps angles (Radians).
     * @param angle Radian angle.
     * @return Normalized angle between -pi and pi.
     */
    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


    public class HeadingPIDF {

        private double kP;
        private double kI;
        private double kD;
        private double kF; // feedforward gain

        private double integralSum = 0.0;
        private double lastError = 0.0;
        private long lastTime;

        private double minOutput = -1.0;
        private double maxOutput = 1.0;

        public HeadingPIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            lastTime = System.nanoTime();
        }

        public void reset() {
            integralSum = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime();
        }

        /**
         * @param error Distance in radians from target heading (AutoWrapped)
         * @return A value between -1 and 1 used for yaw motions
         */
        public double calculate(double error) {

            error = AimAssist.angleWrap(error);

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            // PID terms
            integralSum += error * deltaTime;
            double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0.0;
            lastError = error;

            double pTerm = kP * error;
            double iTerm = kI * integralSum;
            double dTerm = kD * derivative;

            // Feedforward (static friction compensation)
            double fTerm = kF * Math.signum(error);

            double output = pTerm + iTerm + dTerm + fTerm;

            // Clamp output
            if (output > maxOutput) output = maxOutput;
            if (output < minOutput) output = minOutput;

            return output;
        }
    }


    /**
     * Sets robot ideal launch velocity for current position
     * using odometry and camera positioning.
     *
     * @return the ideal launcher velocity for current position
     * @SimpleMode returns robot.overrideLauncherVel
     */
    public double runPowerCalculation(Pose currentPose, Pose targetPose) {

        if(simpleMode) {
            return robot.overrideLauncherVel;
        }

        // Distance from target represented as x
        double x = robot.aimAssist.getPoseDistance(currentPose, targetPose);
        double idealLauncherVelocity = (0.0182942*x*x)+(1.3243*x)+1130.37088; // Not using Math.pow for speed sake
        log("Launch Velocity Calculation: " + Math.round(idealLauncherVelocity));

        return idealLauncherVelocity;

    }

    private void log(String message) {
        robot.log("[AimAssist] - " + message);
    }

}
