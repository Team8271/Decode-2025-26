package org.firstinspires.ftc.teamcode.config;

/**
 * Thread class used to launch artifacts.
 * Must be provided a Config using 'setConfig().'
 */
public class LauncherThread extends Thread {
    Config robot;

    public void setConfig(Config robot) {
        this.robot = robot;
    }

    private int artifactsToLaunch = 0;
    private volatile boolean isBusy = false;

    private volatile boolean running = true; // When false, thread terminates

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

            isBusy = true;

            doLaunch(artifactsToLaunch);

            isBusy = false;

        }
    }

    public void waitWhileBusy() throws InterruptedException {
        while (isBusy) sleep(50);
    }

    private void setLauncherPower(double power) {
        robot.leftLauncher.setPower(power);
        robot.rightLauncher.setPower(power);
    }

    private void doLaunch(int artifactsToLaunch) {
        try {
            double agitatorStartPower = robot.agitator.getPower();

            robot.agitator.setPower(robot.agitatorActivePower);

            setLauncherPower(robot.idealLauncherPower);
            robot.kickerMotor.setPower(robot.kickerOnPower);
            sleep(robot.motorRampUpTime); // Ramp up motors
            robot.kickerServo.setPosition(robot.activeKickerPosition);
            sleep(700);
            // Artifact One fully exited
            for (int i = 1; i < artifactsToLaunch; i++) {
                robot.kickerServo.setPosition(robot.storeKickerPosition);
                sleep(400); // Waiting for artifact to clear kicker
                robot.kickerServo.setPosition(robot.activeKickerPosition);
                sleep(700); // Artifact X fully exited
            }

            // Go to IDLE mode
            robot.kickerServo.setPosition(robot.storeKickerPosition);
            robot.kickerMotor.setPower(robot.kickerIdlePower);
            setLauncherPower(0);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public synchronized void terminate() {
        running = false;
        notify();
    }

    public synchronized void launch(int artifactsToLaunch) {
        this.artifactsToLaunch = artifactsToLaunch;
        notify();
    }
}
