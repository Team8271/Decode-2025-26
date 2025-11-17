package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import dev.narlyx.tweetybird.Drivers.Mecanum;
import dev.narlyx.tweetybird.Odometers.ThreeWheeled;
import dev.narlyx.tweetybird.TweetyBird;

/// Configuration class
public class Config {

    // Changeable Power Values
    public final double kickerIdlePower = 0, kickerOnPower = 1,
        storeKickerPosition = 0.5, activeKickerPosition = 1,
        agitatorActivePower = 1;

    public final int motorRampUpTime = 300;

    // This value will be changed with Limelight sensing to get the ideal power
    public double idealLauncherPower = 1;
    public double idealLauncherVelocity = 0; // 0-2500 effective range

    // Reference to opMode class
    public final LinearOpMode opMode;

    // Define Motors
    public DcMotorEx fl, fr, bl, br,
            agitator, leftLauncher, rightLauncher, launcherMotor,
            kickerMotor;

    // Define Servos
    public Servo kickerServo, leftTilt, rightTilt, intakeServo;

    // Other Hardware
    public Limelight3A limelight;
    public IMU imu;

    // Variables
    public final int   limelightLocalizationPipeline   = 0,
                        limelightObeliskPipeline        = 1,
                        limelightRedPipeline            = 2,
                        limelightBluePipeline           = 3;

    public boolean goalAnglesAreValid;
    public double goalTx;
    public double goalTy;

    public Motif motif;

    public LauncherThread launcherThread;
    public LimelightThread limelightThread;

    public AimAssist aimAssist;

    // enums
    public enum Motif {
        GPP,    // AprilTag 21
        PGP,    // AprilTag 22
        PPG,    // AprilTag 23
        NULL;
    }
    public enum Alliance {
        RED,
        BLUE;
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
        this.opMode = opMode;}

    /// Initialization Method
    public void init() {
        motif = Motif.NULL;
        setAlliance(Alliance.BLUE);

        // Shorten HardwareMap for frequent use
        HardwareMap hwMap;

        if(linearOpMode != null) {
            hwMap = linearOpMode.hardwareMap;
            linearOpMode.telemetry.setMsTransmissionInterval(11);
            usingLinearOpMode = true;
        }
        else {
            hwMap = opMode.hardwareMap;
            opMode.telemetry.setMsTransmissionInterval(11);
            usingLinearOpMode = false;
        }

        // IMU
        imu = hwMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Front Left Drive
        fl = hwMap.get(DcMotorEx.class, "fl");
        fl.setDirection(DcMotor.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Front Right Drive
        fr = hwMap.get(DcMotorEx.class, "fr");
        fr.setDirection(DcMotor.Direction.FORWARD);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Left Drive
        bl = hwMap.get(DcMotorEx.class, "bl");
        bl.setDirection(DcMotor.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Right Drive
        br = hwMap.get(DcMotorEx.class, "br");
        br.setDirection(DcMotor.Direction.FORWARD);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor used in the Active Agitator module
        agitator = hwMap.get(DcMotorEx.class, "agitator");
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Robot Facing Left Launcher Motor
        leftLauncher = hwMap.get(DcMotorEx.class, "leftLauncher");
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Robot Facing Right Launcher Motor
        rightLauncher = hwMap.get(DcMotorEx.class, "rightLauncher");
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Modified Robot Launcher Motor
        launcherMotor = hwMap.get(DcMotorEx.class, "launcher");
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Kicker Motor "spin-ny thing"
        kickerMotor = hwMap.get(DcMotorEx.class, "kickerMotor");
        kickerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        kickerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Kicker Servo - Transfers artifacts from agitator to launcher
        kickerServo = hwMap.get(Servo.class, "kickerServo");
        kickerServo.setPosition(storeKickerPosition);
        intakeServo = hwMap.get(Servo.class, "intakeServo");

        // Launcher Multithreading
        launcherThread = new LauncherThread();
        launcherThread.setConfig(this);

        limelightThread = new LimelightThread();
        limelightThread.setConfig(this);

        // Starts Threads
        checkAndRestartThreads();

        // Aim Assist
        aimAssist = new AimAssist(this);

        // Tilt Servos - Used to angle projectile
        //leftTilt = hwMap.get(Servo.class, "leftTilt");
        //rightTilt = hwMap.get(Servo.class, "rightTilt");

        // Limelight3A Camera
        limelight = hwMap.get(Limelight3A.class, "limelight");
        opMode.telemetry.setMsTransmissionInterval(11);


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

    /// Initializes TweetyBird.
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
        odometer.resetTo(0,0,0);
    }

    public boolean opModeisActive() {
        if(usingLinearOpMode) {
            return linearOpMode.opModeIsActive();
        }
        else {
            return opModeIsActive;
        }
    }

    /// Set team for launching system etc.
    public void setAlliance(Alliance alliance) {
        log("Team set to " + alliance);
        this.alliance = alliance;
    }

    public void increaseLauncherVelocity() {
        idealLauncherVelocity = Range.clip(idealLauncherVelocity + 100,0,2500);
    }

    public void decreaseLauncherVelocity() {
        idealLauncherVelocity = Range.clip(idealLauncherVelocity - 100,0,2500);
    }

    public void switchAlliance() {
        switch(alliance) {
            case RED: alliance = Alliance.BLUE; break;
            case BLUE: alliance = Alliance.RED; break;
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
        if (!limelightThread.isAlive()) {
            log("Starting limelightThread");
            limelightThread.start();
        }
    }

    public void killThreads() {
        try {
            launcherThread.terminate();
            limelightThread.terminate();
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

}

/**
 * Thread class used to launch artifacts.
 * Must be provided a Config using 'setConfig().'
 */
class LauncherThread extends Thread {
    Config robot;
    public void setConfig(Config robot) {this.robot = robot;}

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
        while(isBusy) sleep(50);
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
            for (int i = 1; i <artifactsToLaunch; i++) {
                robot.kickerServo.setPosition(robot.storeKickerPosition);
                sleep(400); // Waiting for artifact to clear kicker
                robot.kickerServo.setPosition(robot.activeKickerPosition);
                sleep(700); // Artifact X fully exited
            }

            // Go to IDLE mode
            robot.kickerServo.setPosition(robot.storeKickerPosition);
            robot.kickerMotor.setPower(robot.kickerIdlePower);
            setLauncherPower(0);
        }
        catch (InterruptedException e) {
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

/**
 *
 */
class LimelightThread extends Thread {
    Config robot;
    public void setConfig(Config robot) {this.robot = robot;}

    private volatile boolean running = true; // When false, thread terminates

    boolean scanGoalAngle = false;
    boolean scanObelisk = false;
    boolean isBusy = false;

    @Override
    public void run() {
        while (running) {
            synchronized (this) {
                try {
                    isBusy = false;
                    wait(); // Sleep until notified (Save resources)
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            if (!running) break; // check before doing work

            if(scanGoalAngle){
                isBusy = true;
                doScanGoalAngle();
                scanGoalAngle = false;
            }
            else if(scanObelisk){
                isBusy = true;
                doScanObelisk();
                scanObelisk = false;
            }

        }
    }

    /// Limelight updates goalTx and goalTy
    public synchronized void scanGoalAngle(){
        if(!isBusy) {
            scanGoalAngle = true;
            notify();
        }
    }

    /// Limelight updates motif
    public synchronized void scanObelisk(){
        if(!isBusy) {
            scanObelisk = true;
            notify();
        }
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
        if (robot.limelight.getStatus().getPipelineIndex() != selectedPipeline){
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
        }
        else {
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

    private void log(String message) {
        robot.log("[LLThread] - " + message);
    }
}

class AimAssist {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private boolean active;
    private boolean cancel;
    private final double blueDesiredTx = -5;
    private final double redDesiredTx = -5;
    private final double tolerance = 1.5; // Get within 1.5 degrees of target
    private final double minWheelPower = 0.1; // Smallest amount of power that still moves wheels

    Config robot;

    public AimAssist(Config robot) {this.robot = robot;}

    /**
     * Corrects robot heading to align with alliance goal.
     * Unless opMode stops, timeOut, or cancelCorrection.
     *
     * @apiNote For cancelCorrection to work, this must be run in a different thread,
     * as runCorrection utilizes a while loop to complete its task.
     *
     * @param timeOut Max time AimAssist can work in seconds.
     */
    public void runAngleCorrection(double timeOut) {

        active = true;
        runtime.reset();

        double desiredTx;
        if(robot.alliance == Config.Alliance.BLUE) {
            desiredTx = blueDesiredTx;
        }
        else {
            desiredTx = redDesiredTx;
        }

        log("Entering Loop.");
        while(robot.opModeisActive()) {

            if(runtime.time() > timeOut) {
                log("Exiting loop: time out.");
                break;
            }

            if(cancel) {
                log("Exiting loop: cancelled");
                break;
            }

            // Motor Power Calculations
            double distanceFromTarget = Math.abs(robot.goalTx - desiredTx);

            // Scale factor determines how aggressively power increases with distance
            double scaleFactor = 0.05;  // Expect to tune
            double correctionPower = 0.1 + distanceFromTarget * scaleFactor;

            // Clamp to max of 1.0
            correctionPower = Math.min(correctionPower, 1.0);


            // Correction
            // Tx less than desired position
            if(robot.goalTx < desiredTx - tolerance) {
                // Rotate Left
                robot.setWheelPower(-correctionPower, correctionPower, -correctionPower, correctionPower);
            }
            // Tx more than desired position
            else if(robot.goalTx > desiredTx + tolerance) {
                // Rotate Right
                robot.setWheelPower(correctionPower, -correctionPower, correctionPower, -correctionPower);
            }
            // Tx in desired position
            else {
                robot.setWheelPower(0,0,0,0);
                log("Exiting loop: target reached.");
            }

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                log("Loop interrupted during sleep phase.");
                throw new RuntimeException(e);
            }

        }
        if(!robot.opModeisActive()) {
            log("Exiting loop: opMode is no longer active.");
        }
        robot.setWheelPower(0,0,0,0);
        active = false;
        cancel = false;
        log("Exited Loop.");

    }

    /**
     * Sets robot ideal launch velocity for current position
     * using Limelight camera.
     */
    public void runPowerCalculation() {
        robot.idealLauncherVelocity = 2500; // Regression equation here !!!
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

