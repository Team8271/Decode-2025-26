package org.firstinspires.ftc.teamcode.robot.configuration;

/**
 * Thread class used to launch artifacts.
 * Must be provided a Config using 'setConfig().'
 */
public class LauncherThread extends Thread {
    Config robot;
    public void setConfig(Config robot) {this.robot = robot;}

    private volatile boolean launchThree = false;
    private volatile boolean isBusy = false;
    private volatile double targetLauncherVelocity = 1300;

    private volatile boolean running = true; // When false, thread terminates

    @Override
    public void run() {
        while (running) {
            synchronized (this) {
                try {
                    log("Entering loop wait.");
                    wait(); // Sleep until notified (Save resources)
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            if (!running) break; // check before doing work

            isBusy = true;
            log("Set isBusy true in run.");

            if(launchThree) {
                try {
                    doLaunch(3);
                } catch (Exception e) {
                    log("ERROR while LaunchThree: " + e);
                }
            }
            else {
                try {
                    doLaunch(1);
                } catch (Exception e) {
                    log("ERROR while LaunchOne: " + e);
                }
            }


            isBusy = false;
            log("Reached end of run loop.");

        }
    }

    public void waitWhileBusy() {
        while(isBusy && robot.opModeIsActive);
    }

    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Waits for launcher motor to reach target velocity within tolerance.
     * Includes timeout protection to prevent robot freezing.
     * @param velocity target velocity
     * @throws InterruptedException if thread is interrupted
     */
    private void waitForLauncherVelocity(double velocity) throws InterruptedException {
        double curVelocity = robot.launcherMotor.getVelocity();
        long startTime = System.currentTimeMillis();
        long timeoutMs = 5000;
        
        while (!(curVelocity >= velocity-20 && curVelocity <= velocity+20)) {
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                log("Timeout waiting for launcher velocity. Current: " + curVelocity + ", Target: " + velocity);
                break;
            }
            
            if (!robot.opModeisActive()) {
                log("OpMode stopped during velocity wait");
                break;
            }
            
            curVelocity = robot.launcherMotor.getVelocity();
            sleep(50);
        }
        
        log("Launcher velocity reached: " + curVelocity + " (target: " + velocity + ")");
    }

    public void setLauncherVelocity(double velocity) {
        targetLauncherVelocity = velocity;
    }

    public double getLauncherVelocity() {
        return targetLauncherVelocity;
    }

    /**
     * Sets launch motor to targetLauncherVelocity and waits for velocity to match.
     * @implNote Use <I>aimAssist.setLauncherVelocity()</I> in an opMode to change this velocity.
     */
    private void updateLauncherVelocityAndWait() {
        double velocity = targetLauncherVelocity; // Prevent race condition
        robot.launcherMotor.setVelocity(velocity);
        log("Launcher now running at '" + velocity + "' vel.");
        try {
            waitForLauncherVelocity(velocity);
        } catch (InterruptedException e) {
            log("Failed to update LauncherVelocity safely: " + e);
        }
    }

    private void doLaunch(int artifactsToLaunch) {
        try {
            boolean intakeWasRunning = robot.agitator.getPower() != 0;

            robot.agitator.setPower(robot.agitatorActivePower);
            robot.intakeMotor.setVelocity(robot.intakeMotorOnVelocity);

            //robot.aimAssist.runAngleCorrection(5);
            //robot.aimAssist.runPowerCalculation();

            updateLauncherVelocityAndWait();
            robot.deactivateIntakeLimiter();
            sleep(150);
            log("Kicking with Vel of: " + robot.launcherMotor.getVelocity());
            robot.activateKicker();
            robot.waitForKicker();
            // Artifact One fully exited
            for (int i = 1; i < artifactsToLaunch; i++) {
                robot.agitator.setPower(0);
                //robot.intakeMotor.setVelocity(); // designed for power not vel
                robot.storeKicker();
                robot.waitForKicker();
                robot.agitator.setPower(robot.agitatorActivePower);
                robot.intakeMotor.setVelocity(200);
                sleep(350); // Waiting for artifact to enter kicker
                if(i==3){
                    sleep(250); // Additional wait for third artifact
                }
                updateLauncherVelocityAndWait();
                log("Kicking with Vel of: " + robot.launcherMotor.getVelocity());
                robot.activateKicker();
                robot.waitForKicker();
            }

            // Go to IDLE mode
            idleLauncher();
            if (intakeWasRunning) {
                robot.runIntakeAssembly();
            }
            else {robot.stopIntakeAssembly();}

        }
        catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
     /// Sets the Launcher to an Idle state using safe method of clearing kicker path.
    public void idleLauncher() {
        robot.storeKicker();
        robot.activateIntakeLimiter();
        robot.launcherMotor.setVelocity(robot.idleLauncherVelocity);
        log("Launcher set to Idle");
    }

    public void cancelLaunch() {
        if(robot.launcherThread.isBusy) {
            log("Cancel: Interrupting launcher");
            robot.launcherThread.interrupt();
            log("Cancel: Idling Launcher");
            idleLauncher();

        }

    }

    /**
     * @deprecated Using doLaunch for both three and two currently.
     * @throws InterruptedException
     */
    private void doLaunchThree() throws InterruptedException {

        robot.agitator.setPower(robot.agitatorActivePower);
        robot.intakeMotor.setVelocity(robot.intakeMotorOnVelocity*.4);

        // !! Unstable: If idealVelocity changes -> may never reach
        // Launch motor ramp up.
        setLauncherVelocity(robot.idealLauncherVelocity);
        waitForLauncherVelocity(robot.idealLauncherVelocity);

        // First launch
        robot.activateKicker();
        robot.deactivateIntakeLimiter();

        //robot.waitForKicker();

        log("(1/3) - Artifact Launched");

        // TODO: Reverse agitator
        // TODO: Slow reverse intake

        // Store kicker
        robot.storeKicker();
        //robot.waitForKicker();

        // TODO: Set agitator forward - runToPos?
        // TODO: Slow forward intake
        // TODO: Wait for artifact to enter chamber - runToPos?

        // Activate kicker
        robot.activateKicker();
        //robot.waitForKicker();

        // TODO: motor reRamp - not in repeat
        // TODO: repeat

        // Activate Intake Limiter
        robot.activateIntakeLimiter();
        robot.waitForLimiter();

        // TODO: turn off/on agitator && intake (option?)

        log("Finished Launch Sequence Type:3");
    }

    public synchronized void terminate() {
        running = false;
        notify();
    }

    public synchronized void launchOne() {
        launchThree = false;
        notify();
    }
    public synchronized void launchThree() {
        launchThree = true;
        notify();
    }

    private void log(String message) {
        robot.log("[LauncherThread] - " + message);
    }

}
