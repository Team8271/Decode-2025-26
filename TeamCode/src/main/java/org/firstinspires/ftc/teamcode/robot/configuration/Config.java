package org.firstinspires.ftc.teamcode.robot.configuration;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
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
            agitatorActivePower = 1, intakeMotorOnVelocity = 750, intakeMotorOffVelocity = 0,
            overrideLauncherVel = 1225;

    private final double
            storeLeftKickerPosition = 0.6, storeRightKickerPosition = 1 - storeLeftKickerPosition,
            activeLeftKickerPosition = 0, activeRightKickerPosition = 1 - activeLeftKickerPosition,
            intakeLimServerActivePosition = 0.85, intakeLimServoInactivePosition = 0.5;

    public final double indicatorLightOn = 0.3, indicatorLightOff = 0,
                        indicatorLightReverse = 0.555, indicatorLightOffBog = 0.444;

    public final int motorRampUpTime = 3000;

    public boolean devBool = false;
    private boolean brakesActive = false;

    // This value will be changed with Limelight sensing to get the ideal power
    public double idealLauncherVelocity = 1225, idleLauncherVelocity = 900;

    private double  desiredLeftKickerPosition = storeLeftKickerPosition,
                    desiredIntakeLimiterPosition = intakeLimServerActivePosition,
                    brakeOn = 0, brakeOff = 0.5;

    // Reference to opMode class
    public final LinearOpMode linearOpMode;
    public final OpMode opMode;

    boolean usingLinearOpMode = false;

    boolean opModeIsActive = false;

    // Define Motors
    public DcMotorEx fl, fr, bl, br,
            agitator, launcherMotor, intakeMotor;

    // Define Servos
    private Servo leftKickerServo, rightKickerServo, intakeLimServo, leftBrake, rightBrake;
    public Servo indicatorLight;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;

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
        RED(new Pose(138,138)),
        BLUE(new Pose(6, 138));

        private final Pose alliance;
        Alliance(Pose alliance) { this.alliance = alliance; }
        public Pose getPose() { return alliance; }
        public Pose getLogoPose() { return alliance; } // Not set up yet
    }

    public enum DriverAmount {
        ONE_DRIVER,
        TWO_DRIVERS,
        DEV_DRIVER
    }

    public Alliance alliance;
    DriverAmount driverAmount;

    // TweetyBird Classes
    public ThreeWheeled odometer;
    public Mecanum mecanum;
    public TweetyBird tweetyBird;

    // Follower
    Follower follower;

    private Config(LinearOpMode linearOpMode, OpMode opMode, Follower follower) {
        this.linearOpMode = linearOpMode;
        this.opMode = opMode;
        this.follower = follower;
    }

    public Config(LinearOpMode linearOpMode) {
        this(linearOpMode,null,null);
    }

    public Config(LinearOpMode linearOpMode, Follower follower) {
        this(linearOpMode,null,follower);
    }

    public Config(OpMode opMode) {
        this(null,opMode,null);
    }

    public Config(OpMode opMode, Follower follower) {
        this(null,opMode,follower);
    }

    /// Initialization Method
    public void init() {
        motif = Motif.NULL;
        setAlliance(readAllianceFromFile());
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
            opModeIsActive = true;
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
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(130, 0,0,14.3));

        // Called 'org' in spirit
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Kicker Servo - Transfers artifacts from agitator to launcher
        leftKickerServo = hwMap.get(Servo.class, "lKickerServo");
        leftKickerServo.setPosition(storeLeftKickerPosition);
        rightKickerServo = hwMap.get(Servo.class, "rKickerServo");
        rightKickerServo.setPosition(storeRightKickerPosition);
        // IntakeLimiterServo
        intakeLimServo = hwMap.get(Servo.class, "intakeLimServo");
        intakeLimServo.setPosition(intakeLimServerActivePosition);
        // Brakes
        leftBrake = hwMap.get(Servo.class, "leftBrake");
        rightBrake = hwMap.get(Servo.class, "rightBrake");
        leftBrake.setPosition(brakeOff);
        rightBrake.setPosition(brakeOff);

        redLED = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

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
        aimAssist = new AimAssist(this,4,0,0.3,0.01);

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

    /*public void aimAssistInit() {
        aimAssist = new AimAssist(this,1.5,0,0,0.6);
    }*/

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

    public void setOpModeIsActive(boolean value) {
        opModeIsActive = value;
    }

    /// Set team for launching system etc.
    public void setAlliance(Alliance alliance) {
        log("Team set to " + alliance);
        this.alliance = alliance;
        saveAllianceToFile(alliance);
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

    /**
     * Reads a Pose from file.
     * Returns null if the file does not exist or is invalid.
     */
    public Pose readPoseFromFile() {
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "pose.txt");

            if (!file.exists()) {
                log("Pose file not found");
                return null;
            }

            String contents = ReadWriteFile.readFile(file).trim();
            String[] parts = contents.split(",");

            if (parts.length != 3) {
                log("Invalid pose format");
                return null;
            }

            double x = Double.parseDouble(parts[0]);
            double y = Double.parseDouble(parts[1]);
            double heading = Double.parseDouble(parts[2]);

            Pose pose = new Pose(x, y, heading);
            log("Pose read from file: " + pose);

            return pose;

        } catch (Exception e) {
            log("Failed to read pose: " + e.getMessage());
            return null;
        }
    }

    /**
     * Saves a Pose to file.
     */
    public void savePoseToFile(Pose pose) {
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "pose.txt");

            String data = pose.getX() + "," +
                    pose.getY() + "," +
                    pose.getHeading();

            ReadWriteFile.writeFile(file, data);
            log("Pose saved: " + pose);

        } catch (Exception e) {
            log("Failed to save pose: " + e.getMessage());
        }
    }

    /**
     * Clears the saved pose file.
     */
    public void invalidateSavedPose() {
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "pose.txt");

            if (file.exists()) {
                ReadWriteFile.writeFile(file, "");
                log("Saved pose invalidated");
            }

        } catch (Exception e) {
            log("Failed to invalidate pose: " + e.getMessage());
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
    public void runIntakeAssembly(double vel) {
        intakeMotor.setVelocity(vel);
        agitator.setPower(agitatorActivePower);
    }

    public void stopIntakeAssembly() {
        intakeMotor.setVelocity(intakeMotorOffVelocity);
        agitator.setPower(0);
    }

    public void activateBrakes() {
        leftBrake.setPosition(brakeOn);
        rightBrake.setPosition(brakeOn);

        brakesActive = true;
    }

    public void deactivateBrakes() {
        leftBrake.setPosition(brakeOff);
        rightBrake.setPosition(brakeOff);

        brakesActive = false;
    }

    public void toggleBrakes() {
        if (brakesActive) {
            deactivateBrakes();
            setIndicatorLightGreen();
        }
        else {
            activateBrakes();
            setIndicatorLightRed();
        }
    }

    private void setIndicatorLightGreen() {
        redLED.setState(false);
        greenLED.setState(true);
    }

    private void setIndicatorLightRed() {
        redLED.setState(true);
        greenLED.setState(false);
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


