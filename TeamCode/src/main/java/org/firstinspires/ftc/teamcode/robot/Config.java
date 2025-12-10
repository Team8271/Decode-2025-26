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
import java.util.ArrayList;
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
            storeLeftKickerPosition = 0.45, storeRightKickerPosition = 1 - storeLeftKickerPosition,
            activeLeftKickerPosition = 0, activeRightKickerPosition = 1 - activeLeftKickerPosition,
            intakeLimServerActivePosition = 1, intakeLimServoInactivePosition = 0.5;

    public final int motorRampUpTime = 3000;

    public boolean devBool = false;

    // This value will be changed with Limelight sensing to get the ideal power
    /// @deprecated in favor of launcher velocity
    public double idealLauncherPower = 1;
    public double idealLauncherVelocity = 1900; // !! WARNING, VALUE USED IN AUTO... 0-2500 effective range

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

    // Other Hardware
    public Limelight3A limelightCamera;
    public IMU imu;

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
        imu = hwMap.get(IMU.class, "imu");
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
        agitator.setDirection(DcMotor.Direction.REVERSE);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Modified Robot Launcher Motor
        launcherMotor = hwMap.get(DcMotorEx.class, "launcher");
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Called 'org' in spirit
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Kicker Servo - Transfers artifacts from agitator to launcher
        leftKickerServo = hwMap.get(Servo.class, "lKickerServo");
        leftKickerServo.setPosition(storeLeftKickerPosition);
        rightKickerServo = hwMap.get(Servo.class, "rKickerServo");
        rightKickerServo.setPosition(storeRightKickerPosition);
        // IntakeLimiterServo
        intakeLimServo = hwMap.get(Servo.class, "intakeLimServo");
        intakeLimServo.setPosition(intakeLimServerActivePosition);

        // Launcher Multithreading
        launcherThread = new LauncherThread();
        launcherThread.setConfig(this);

        // Starts Threads
        checkAndRestartThreads();

        // Aim Assist
        aimAssist = new AimAssist(this);

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
        intakeLimServo.setPosition(intakeLimServerActivePosition);
        devBool = true;
    }

    public void deactivateIntakeLimiter() {
        intakeLimServo.setPosition(intakeLimServoInactivePosition);
        devBool = false;
    }

    public void reverseIntakeAssembly() {
        intakeMotor.setVelocity(-intakeMotorOnVelocity);
        agitator.setPower(-agitatorActivePower);
    }

    public void activateKicker() {
        deactivateIntakeLimiter();
        leftKickerServo.setPosition(activeLeftKickerPosition);
        rightKickerServo.setPosition(activeRightKickerPosition);
    }

    public void storeKicker() {
        activateIntakeLimiter();
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

    private int artifactsToLaunch = 0;
    private volatile boolean isBusy = false;
    private volatile boolean holdPower = false;

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

            doLaunch(artifactsToLaunch, holdPower);


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

    private void doLaunch(int artifactsToLaunch, boolean keepPower) {
        try {
            double agitatorStartPower = robot.agitator.getPower();

            robot.agitator.setPower(robot.agitatorActivePower);

            //robot.aimAssist.runAngleCorrection(5);
            //robot.aimAssist.runPowerCalculation();

            setLauncherVelocity(robot.idealLauncherVelocity);
            sleep(robot.motorRampUpTime); // Ramp up motor
            robot.activateKicker();
            sleep(900);
            // Artifact One fully exited
            for (int i = 1; i <artifactsToLaunch; i++) {
                robot.agitator.setPower(-robot.agitatorActivePower*.8);
                robot.storeKicker();
                sleep(450); // wait for lowering of kicker
                robot.agitator.setPower(robot.agitatorActivePower);
                sleep(600); // Waiting for artifact to enter kicker
                robot.activateKicker();
                sleep(900); // Artifact X fully exited
            }

            // Go to IDLE mode
            robot.agitator.setPower(-robot.agitatorActivePower);
            robot.storeKicker();
            sleep(200);
            robot.agitator.setPower(agitatorStartPower);
            if(!keepPower) {
                setLauncherVelocity(0);
                isBusy = false;
            }
            else {isBusy = false;}

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
    public synchronized void launch(int artifactsToLaunch, boolean holdPower) {
        this.holdPower = holdPower;
        this.artifactsToLaunch = artifactsToLaunch;
        notify();
    }

    private void log(String message) {
        robot.log("[LauncherThread] - " + message);
    }

}


class Limelight {
    Config robot;

    public Limelight(Config robot) {this.robot = robot;}

    private GoalResults goalResults = new GoalResults();


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
            robot.limelightCamera.start();
        }
    }

    private void changePipeline(Pipeline pipeline) {
        // Set desired pipeline
        robot.limelightCamera.pipelineSwitch(pipeline.getValue());
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

    public AimAssist(Config robot) {
        this.robot = robot;
        log("Ready.");
    }

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

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        if(!goalAngles.goalAnglesAreValid) {
            log("Goal Angles Invalid, Cancelling.");
            return;
        }

        double desiredTx;

        if(robot.alliance == Config.Alliance.RED) {
            desiredTx = redDesiredTx;
            log("Correcting for Red.");
        }
        else {
            desiredTx = blueDesiredTx;
            log("Correcting for Blue.");
        }

        log("Entering loop.");
        while(robot.opModeisActive() && timeOut > runtime.time()) {
            goalAngles = robot.limelight.scanGoalAngle();


            double distance = Math.abs(goalAngles.goalTx - desiredTx);
            correctionPower = Range.clip(distance*scaleFactor, 0.1, 0.2);


            if(distance < 0.5) {
                log("Within tolerance, Closing Loop...");
                break;
            }
            else if(goalAngles.goalTx > desiredTx) {
                // Rotate right
                robot.setWheelPower(correctionPower,-correctionPower,correctionPower,-correctionPower);
            }
            else if(goalAngles.goalTx < desiredTx) {
                // Rotate left
                robot.setWheelPower(-correctionPower,correctionPower,-correctionPower,correctionPower);
            }
        }
        robot.setWheelPower(0,0,0,0);
        log("Loop Closed.");

    }

    /**
     * Using robot pose, calculate the correct heading for backboard shots.
     *
     * @return PedroHeading in radians
     */
    public double runHeadingCalculation(Pose currentPose) {
        Pose blueGoal = new Pose(-58.3727, -55.6425);
        Pose redGoal = new Pose(-58.3727, 55.6425);
        Pose goal;
        boolean rotateLeft = true;
        double blueGoalHeading = 135;
        double redGoalHeading = 50;
        double goalHeading = blueGoalHeading;
        if(robot.alliance == Config.Alliance.RED) {
            goal = redGoal;
            goalHeading = redGoalHeading;

        }
        else {
            goal = blueGoal;
        }

        if(currentPose.getHeading() > goalHeading) {
            // Rotate Right
            rotateLeft = false;
            // Otherwise it needs to rotate left
        }

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double adjacentLeg = Math.sqrt(Math.pow(currentY-goal.getY(),2) + Math.pow(currentX-goal.getX(),2));
        double oppositeLeg = 18; // Distance from apriltag to backboard;

        double headingCorrection = Math.atan(oppositeLeg/adjacentLeg); // Angular Dist from correct heading

        if(rotateLeft) {
            headingCorrection += currentPose.getHeading();
        }
        else { // Rotate Right
            headingCorrection -= currentPose.getHeading();
        }

        return headingCorrection; // Return

    }

    /**
     * Sets robot ideal launch velocity for current position
     * using Limelight camera.
     */
    public void runPowerCalculation() {

        double x = goalAngles.goalTy;
        robot.idealLauncherVelocity = -2.46914*Math.pow(x,3)+74.07407*Math.pow(x,2)-751.85185*x+3880.24691;
        log("Launch Velocity Calculation: " + Math.round(robot.idealLauncherVelocity));



        /*
        double x = robot.goalAvgDist;
        robot.idealLauncherVelocity = (989.29847*x)+255.92292;
        log("Launch Velocity Calculation: " + Math.round(robot.idealLauncherVelocity));

         */

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

