package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import dev.narlyx.tweetybird.Drivers.Mecanum;
import dev.narlyx.tweetybird.Odometers.ThreeWheeled;
import dev.narlyx.tweetybird.TweetyBird;

/// Configuration class
public class Config {

    // Changeable Power Values
    public final double
            agitatorActivePower = 1, intakeMotorOnVelocity = 500, intakeMotorOffVelocity = 0;

    private final double
            storeLeftKickerPosition = 0.6, storeRightKickerPosition = 1 - storeLeftKickerPosition,
            activeLeftKickerPosition = 0, activeRightKickerPosition = 1 - activeLeftKickerPosition,
            intakeLimServerActivePosition = 1, intakeLimServoInactivePosition = 0.5;

    public final double indicatorLightOn = 0.722, indicatorLightOff = 0,
                        indicatorLightReverse = 0.555;

    public final int motorRampUpTime = 3000;

    public boolean devBool = false;

    // This value will be changed with Limelight sensing to get the ideal power
    public double idealLauncherVelocity = 1900; // !! WARNING, VALUE USED IN AUTO... 0-2500 effective range

    private double  desiredLeftKickerPosition = storeLeftKickerPosition,
                    desiredIntakeLimiterPosition = intakeLimServerActivePosition;

    // Reference to opMode class
    public final LinearOpMode linearOpMode;
    public final OpMode opMode;

    boolean usingLinearOpMode = false;

    boolean opModeIsActive = false;

    // Define Motors
    public DcMotorEx fl, fr, bl, br,
            agitator, launcherMotor, intakeMotor;

    // Define Servos
    private Servo leftKickerServo, rightKickerServo, intakeLimServo;
    public Servo indicatorLight;

    // Other Hardware
    public Limelight3A limelightCamera;
    //public IMU imu;

    public Motif motif;

    public LauncherThread launcherThread;
    public Limelight limelight;

    public AimAssist aimAssist;

    // enums
    public enum Motif {
        GPP,    // AprilTag 21
        PGP,    // AprilTag 22
        PPG,    // AprilTag 23
        NULL;
    }

    public enum Alliance {
        RED(new Pose(144,144)),
        BLUE(new Pose(0, 144));

        private final Pose alliance;
        Alliance(Pose alliance) { this.alliance = alliance; }
        public Pose getPose() { return alliance; }
    }

    public enum DriverAmount {
        ONE_DRIVER,
        TWO_DRIVERS,
        DEV_DRIVER
    }

    Alliance alliance;
    DriverAmount driverAmount;

    // TweetyBird Classes
    public ThreeWheeled odometer;
    public Mecanum mecanum;
    public TweetyBird tweetyBird;

    // Pass opMode to config
    public Config(LinearOpMode linearOpMode, OpMode opMode) {
        this.linearOpMode = linearOpMode;
        this.opMode = opMode;
    }

    /// Initialization Method
    public void init() {
        motif = Motif.NULL;
        setAlliance(Alliance.BLUE);
        opModeIsActive = false;

        // Shorten HardwareMap for frequent use
        HardwareMap hwMap;

        if (linearOpMode != null) {
            hwMap = linearOpMode.hardwareMap;
            linearOpMode.telemetry.setMsTransmissionInterval(11);
            usingLinearOpMode = true;
        } else {
            hwMap = opMode.hardwareMap;
            opMode.telemetry.setMsTransmissionInterval(11);
            usingLinearOpMode = false;
        }

        // IMU
        //imu = hwMap.get(IMU.class, "imu");
        //RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        //RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        //RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        //imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Front Left Drive
        fl = hwMap.get(DcMotorEx.class, "fl");
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Front Right Drive
        fr = hwMap.get(DcMotorEx.class, "fr");
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Back Left Drive
        bl = hwMap.get(DcMotorEx.class, "bl");
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Back Right Drive
        br = hwMap.get(DcMotorEx.class, "br");
        br.setDirection(DcMotorEx.Direction.FORWARD);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Motor used in the Active Agitator module
        agitator = hwMap.get(DcMotorEx.class, "agitator");
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Modified Robot Launcher Motor
        launcherMotor = hwMap.get(DcMotorEx.class, "launcher");
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Called 'org' in spirit
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Kicker Servo - Transfers artifacts from agitator to launcher
        leftKickerServo = hwMap.get(Servo.class, "lKickerServo");
        leftKickerServo.setPosition(storeLeftKickerPosition);
        rightKickerServo = hwMap.get(Servo.class, "rKickerServo");
        rightKickerServo.setPosition(storeRightKickerPosition);
        // IntakeLimiterServo
        intakeLimServo = hwMap.get(Servo.class, "intakeLimServo");
        intakeLimServo.setPosition(intakeLimServerActivePosition);

        // Light
        indicatorLight = hwMap.get(Servo.class, "bigStupidLight");
        indicatorLight.setPosition(indicatorLightOn);

        // Launcher Multithreading
        launcherThread = new LauncherThread();
        launcherThread.setConfig(this);

        limelight = new Limelight(this);

        // Starts Threads
        checkAndRestartThreads();

        // Aim Assist
        aimAssist = new AimAssist(this,1.5,0,0,0.6);

        // Limelight3A Camera
        limelightCamera = hwMap.get(Limelight3A.class, "limelight");

        // Build drivetrain for TweetyBird Use
        mecanum = new Mecanum.Builder()
                .setFrontLeftMotor(fl)
                .setFrontRightMotor(fr)
                .setBackLeftMotor(bl)
                .setBackRightMotor(br)
                .build();

        // Build odometers for TweetyBird Use
        odometer = new ThreeWheeled.Builder()
                .setLeftEncoder(bl)
                .setRightEncoder(fr)
                .setMiddleEncoder(br)

                .setEncoderTicksPerRotation(2000)
                .setEncoderWheelRadius(0.944882)

                //Change the true/false values to correct directions
                .setFlipLeftEncoder(true)
                .setFlipRightEncoder(true)
                .setFlipMiddleEncoder(true)

                .setSideEncoderDistance(10.6875)
                .setMiddleEncoderOffset(7.5625)
                .build();

        log("Robot Initialized");
    }

    public void aimAssistInit() {
        aimAssist = new AimAssist(this,1.5,0,0,0.6);
    }

    /** Initializes TweetyBird.
     * @deprecated In favor of Pedro-Pathing
     */
    public void initTweetyBird() {
        log("TweetyBird Initialized");
        tweetyBird = new TweetyBird.Builder()
                .setDistanceBuffer(1) // Inch(es)
                .setDriver(mecanum)
                .setLinearOpMode(linearOpMode)
                .setMaximumSpeed(0.5)
                .setMinimumSpeed(0.2)
                .setOdometer(odometer)
                .setRotationBuffer(4) // Degree(s)
                .setLoggingEnabled(true)
                .build();
        odometer.resetTo(0, 0, 0);
    }

    public boolean opModeisActive() {
        if (usingLinearOpMode) {
            return linearOpMode.opModeIsActive();
        } else {
            return opModeIsActive;
        }
    }

    /// Set team for launching system etc.
    public void setAlliance(Alliance alliance) {
        log("Team set to " + alliance);
        this.alliance = alliance;
    }

    public void increaseLauncherVelocity() {
        idealLauncherVelocity = Range.clip(idealLauncherVelocity + 100, 0, 2500);
    }

    public void decreaseLauncherVelocity() {
        idealLauncherVelocity = Range.clip(idealLauncherVelocity - 100, 0, 2500);
    }

    public void switchAlliance() {
        switch (alliance) {
            case RED:
                alliance = Alliance.BLUE;
                break;
            case BLUE:
                alliance = Alliance.RED;
                break;
        }
        saveAllianceToFile(alliance);
    }


    public void log(String message) {
        // Log to Android Logcat (viewable in Android Studio)
        Log.i("FTC_CONFIG", message);

        /*
        // Also log to telemetry if opMode is available
        if (opMode != null && opMode.telemetry != null) {
            opMode.telemetry.log().add(message);
        }

         */

        // Optional: Log to file
        logToFile(message);
    }

    private void logToFile(String message) {
        try {
            File logFile = new File(AppUtil.FIRST_FOLDER, "robot_log.txt");

            // Create timestamp
            SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss.SSS", Locale.US);
            String timestamp = sdf.format(new Date());

            // Format log message
            String logEntry = "[" + timestamp + " Config]: " + message + "\n";

            // Append to file
            BufferedWriter writer = new BufferedWriter(new FileWriter(logFile, true));
            writer.write(logEntry);
            writer.close();

        } catch (IOException e) {
            // If file logging fails, just log to Logcat
            Log.e("FTC_CONFIG", "Failed to write to log file: " + e.getMessage());
        }
    }

    public Alliance readAllianceFromFile() {
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "alliance.txt");

            if (!file.exists()) {
                log("Alliance file not found, defaulting to RED");
                return Alliance.RED;
            }

            String fileContents = ReadWriteFile.readFile(file).trim().toUpperCase();

            if (fileContents.equals("BLUE")) {
                log("Alliance read from file: BLUE");
                return Alliance.BLUE;
            }

            log("Alliance read from file: RED");
            return Alliance.RED;

        } catch (Exception e) {
            log("Failed to read alliance: " + e.getMessage());
            return Alliance.RED;
        }
    }

    public void saveAllianceToFile(Alliance alliance) {
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "alliance.txt");
            ReadWriteFile.writeFile(file, alliance.toString());
            log("Alliance saved: " + alliance);
        } catch (Exception e) {
            log("Failed to save alliance: " + e.getMessage());
        }
    }

    public DriverAmount readDriverAmountFromFile() {
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "driverAmount.txt");

            if (!file.exists()) {
                log("DriverAmount file not found, defaulting to TWO_DRIVERS");
                return DriverAmount.TWO_DRIVERS;
            }

            String fileContents = ReadWriteFile.readFile(file).trim().toUpperCase();

            if (fileContents.equals("TWO_DRIVERS")) {
                log("DriverAmount read from file: TWO_DRIVERS");
                return DriverAmount.TWO_DRIVERS;
            }

            log("DriverAmount read from file: ONE_DRIVER");
            return DriverAmount.ONE_DRIVER;

        } catch (Exception e) {
            log("Failed to read driver amount: " + e.getMessage());
            return DriverAmount.TWO_DRIVERS;
        }
    }

    public void saveDriverAmountToFile(DriverAmount driverAmount) {
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "driverAmount.txt");
            ReadWriteFile.writeFile(file, driverAmount.toString());
            log("DriverAmount saved: " + driverAmount);
        } catch (Exception e) {
            log("Failed to save driver amount: " + e.getMessage());
        }
    }

    /**
     * Checks if kicker and launcher threads are active.
     * If not active, starts them.
     */
    public void checkAndRestartThreads() {
        if (!launcherThread.isAlive()) {
            log("Starting launcherThread");
            launcherThread.start();
        }
    }

    public void killThreads() {
        try {
            launcherThread.terminate();
        } catch (Exception e) {
            log("Failed to stop launcherThread");
            throw new RuntimeException(e);
        }
    }

    public void setWheelPower(double flPower, double frPower, double blPower, double brPower) {

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void runIntakeAssembly() {
        intakeMotor.setVelocity(intakeMotorOnVelocity);
        agitator.setPower(agitatorActivePower);
    }

    public void stopIntakeAssembly() {
        intakeMotor.setVelocity(intakeMotorOffVelocity);
        agitator.setPower(0);
    }

    public void activateIntakeLimiter() {
        desiredIntakeLimiterPosition = intakeLimServerActivePosition;
        intakeLimServo.setPosition(intakeLimServerActivePosition);
        devBool = true;
    }

    public void waitForLimiter() throws InterruptedException {
        while(intakeLimServo.getPosition() != desiredIntakeLimiterPosition) {
            Thread.sleep(50);
        }
    }

    public void deactivateIntakeLimiter() {
        desiredIntakeLimiterPosition = intakeLimServoInactivePosition;
        intakeLimServo.setPosition(intakeLimServoInactivePosition);
        devBool = false;
    }

    public void reverseIntakeAssembly() {
        intakeMotor.setVelocity(-intakeMotorOnVelocity);
        agitator.setPower(-agitatorActivePower);
    }

    public void activateKicker() {
        desiredLeftKickerPosition = activeLeftKickerPosition;
        leftKickerServo.setPosition(activeLeftKickerPosition);
        rightKickerServo.setPosition(activeRightKickerPosition);
    }

    public void waitForKicker() throws InterruptedException {
        Thread.sleep(200);
    }

    public void storeKicker() {
        desiredLeftKickerPosition = storeLeftKickerPosition;
        leftKickerServo.setPosition(storeLeftKickerPosition);
        rightKickerServo.setPosition(storeRightKickerPosition);
    }
}

/**
 * Thread class used to launch artifacts.
 * Must be provided a Config using 'setConfig().'
 */
class LauncherThread extends Thread {
    Config robot;
    public void setConfig(Config robot) {this.robot = robot;}

    private volatile boolean launchThree = false;
    private volatile boolean isBusy = false;

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
                    interrupt();
                }
            }
            else {
                try {
                    doLaunch(1);
                } catch (Exception e) {
                    log("ERROR while LaunchOne: " + e);
                    interrupt();
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

    private void setLauncherVelocity(double velocity) {
        robot.launcherMotor.setVelocity(velocity);
    }

    private void waitForLauncherVelocity(double velocity) throws InterruptedException {
        double curVelocity = robot.launcherMotor.getVelocity();
        while (curVelocity > velocity-20 && curVelocity < velocity+20) {
            curVelocity = robot.launcherMotor.getVelocity();
            sleep(50);
        }
    }

    private void doLaunch(int artifactsToLaunch) {
        try {
            double agitatorStartPower = robot.agitator.getPower();

            robot.agitator.setPower(robot.agitatorActivePower);
            robot.intakeMotor.setVelocity(200);

            //robot.aimAssist.runAngleCorrection(5);
            //robot.aimAssist.runPowerCalculation();

            setLauncherVelocity(robot.idealLauncherVelocity);
            sleep(robot.motorRampUpTime); // Ramp up motor
            robot.deactivateIntakeLimiter();
            sleep(150);
            robot.activateKicker();
            robot.waitForKicker();
            // Artifact One fully exited
            for (int i = 1; i < artifactsToLaunch; i++) {
                robot.agitator.setPower(0);
                robot.intakeMotor.setPower(-.1);
                robot.storeKicker();
                robot.waitForKicker();
                robot.agitator.setPower(robot.agitatorActivePower);
                robot.intakeMotor.setPower(0.3);
                sleep(650); // Waiting for artifact to enter kicker
                if(i==3){
                    sleep(200); // Additional Wait
                }
                robot.activateKicker();
                robot.waitForKicker();
            }

            // Go to IDLE mode
            idleLauncher();

        }
        catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void idleLauncher(boolean force) throws InterruptedException {
        robot.agitator.setPower(-robot.agitatorActivePower);
        robot.storeKicker();
        robot.activateIntakeLimiter();
        if(!force){
            sleep(1500);
        }
        robot.agitator.setPower(0);
        robot.intakeMotor.setVelocity(0);
        robot.launcherMotor.setPower(0);
        log("Launcher set to Idle");
    }

    /**
     * Sets the Launcher to an Idle state using safe method of clearing kicker path.
     * @throws InterruptedException if interrupted while waiting for artifacts to exit kicker path.
     */
    private void idleLauncher() throws InterruptedException {
        idleLauncher(false);
    }
    /**
     * @deprecated Using doLaunch for both three and two currently.
     * @throws InterruptedException
     */
    private void doLaunchThree() throws InterruptedException {

        robot.agitator.setPower(robot.agitatorActivePower);
        robot.intakeMotor.setPower(0.2);

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


class Limelight {
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

    public GoalResults scanGoalAngle() {
        int selectedPipeline;

        startLimelight();

        if(robot.alliance == Config.Alliance.BLUE) {
            changePipeline(Pipeline.BLUE_GOAL);
        }
        else {
            changePipeline(Pipeline.RED_GOAL);
        }

        // Get latest results
        LLResult result = robot.limelightCamera.getLatestResult();

        GoalResults goalResults = new GoalResults();
        // Update angles
        if (result != null && result.isValid()) {
            goalResults.setResults(true,result.getTx(),
                    result.getTy(),result.getBotposeAvgDist());
        }
        else {
            goalResults.setGoalAnglesAreValid(false);
        }

        return goalResults;

    }

    private void log(String message) {
        robot.log("[Limelight] - " + message);
    }

}

class GoalResults {
    boolean goalAnglesAreValid = false;
    double goalTx;
    double goalTy;
    double goalAvgDist;

    public void setResults(boolean validity, double goalTx, double goalTy, double goalAvgDist) {
        goalAnglesAreValid = validity;
        this.goalTx = goalTx;
        this.goalTy = goalTy;
        this.goalAvgDist = goalAvgDist;
    }
    public void setGoalAnglesAreValid(boolean validity) {
        goalAnglesAreValid = validity;
    }
    public void setGoalTx(double goalTx) {this.goalTx = goalTx;}
    public void setGoalTy(double goalTy) {this.goalTy = goalTy;}
    public void setGoalAvgDist(double goalAvgDist) {this.goalAvgDist = goalAvgDist;}

}

class AimAssist {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private boolean active;
    private boolean cancel;
    private final double blueDesiredTx = -5;
    private final double redDesiredTx = -5;
    private final double tolerance = 3; // Get within x degrees of target
    private final double minWheelPower = 0.1; // Smallest amount of power that still moves wheels
    private double correctionPower = 0.3;
    private double scaleFactor = 0.05;

    GoalResults goalAngles = new GoalResults();

    private enum Poses {
        CLOSE_LAUNCH_1(new Pose(0,0,0)),
        CLOSE_LAUNCH_2(new Pose(0,0,0)),
        CLOSE_LAUNCH_3(new Pose(0,0,0));

        private final Pose pose;
        Poses(Pose pose) {this.pose = pose;}
        public Pose getValue() {return pose;}
    }

    public Pose getNearestPose(Pose currentPose) {

        Poses[] shootingPoses = Poses.values();

        double lowestDistance = 99999;
        Pose closestPose = currentPose;

        for (Poses launchPose : shootingPoses) {
            if(getPoseDistance(currentPose,launchPose.getValue()) < lowestDistance) {
                lowestDistance = getPoseDistance(currentPose,launchPose.getValue());
                closestPose = launchPose.getValue();
            }
        }
        return closestPose;
    }

    public double getPoseDistance(Pose p1, Pose p2) {
        double dx = p1.getPose().getX() - p2.getPose().getX();
        double dy = p1.getPose().getY() - p2.getPose().getY();
        double dz = p1.getPose().getHeading() - p2.getPose().getHeading();


        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    Config robot;

    HeadingPID headingPID;

    public AimAssist(Config robot, double kP, double kI, double kD, double maxPower) {
        this.robot = robot;
        headingPID = new HeadingPID(kP, kI, kD);
        headingPID.setOutputLimits(maxPower);
        log("Ready.");
    }

    public double getCorrectionYaw(Pose currentPose, Pose targetToFace) {
        double headingCalc = robot.aimAssist.getHeadingForTarget(currentPose,targetToFace);
        return currentPose.getHeading()-headingCalc;
    }

    /**
     * Calculates desired heading to face a position on a pedro-field.
     * @param currentPose Robot current position on the field <b>(must be correct)</b>
     * @param targetToFace Pose of the desired position to be facing
     * @return The correct heading in radians to face the target Pose
     */
    public double getHeadingForTarget(Pose currentPose, Pose targetToFace) {
        // Robot's current state
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double currentZ = currentPose.getHeading(); // Heading in radians

        // Target coordinates
        double targetX = targetToFace.getX();
        double targetY = targetToFace.getY();

        // Calculate the target heading in the field frame
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double targetHeading = Math.atan2(deltaY, deltaX);

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

    /**
     *
     * @return The correct alliance goal heading.
     */
    public double simpleGoalHeading() {
        if(robot.alliance == Config.Alliance.RED) {
            return Math.toRadians(135);
        }
        else{
            return Math.toRadians(50); // Why flipped ??? Idk
        }
    }

    class HeadingPID {

        private double kP;
        private double kI;
        private double kD;

        private double integralSum = 0.0;
        private double lastError = 0.0;
        private long lastTime;

        private double minOutput = -1.0;
        private double maxOutput = 1.0;

        public HeadingPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            lastTime = System.nanoTime();
        }

        public void setOutputLimits(double max) {
            minOutput = -max;
            maxOutput = max;
        }

        public void reset() {
            integralSum = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime();
        }

        /**
         *
         * @param error Distance in radians from target heading (AutoWrapped)
         * @apiNote Use AimAssist's getHeadingForTarget() for calculating error
         * @return A value between -1 and 1 used for yaw motions
         */
        public double calculate(double error) {

            error = AimAssist.angleWrap(error);

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            integralSum += error * deltaTime;
            double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0.0;
            lastError = error;

            double output = (kP * error) + (kI * integralSum) + (kD * derivative);

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
     */
    public double runPowerCalculation() {

        double x = goalAngles.goalTy;
        double idealLauncherVelocity = 0.0197859*Math.pow(x,2)+1.33493*x+967.92439;
        log("Launch Velocity Calculation: " + Math.round(robot.idealLauncherVelocity));

        return idealLauncherVelocity;

    }


    /**
     * Cancels runCorrection.
     */
    public void cancelCorrection() {
        if(active) {
            cancel = true;
        }
    }

    public boolean isActive() {
        return active;
    }

    private void log(String message) {
        robot.log("[AimAssist] - " + message);
    }

}

