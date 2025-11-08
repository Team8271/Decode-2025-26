package org.firstinspires.ftc.teamcode.config;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

import dev.narlyx.tweetybird.Drivers.Mecanum;
import dev.narlyx.tweetybird.Odometers.ThreeWheeled;
import dev.narlyx.tweetybird.TweetyBird;

/// Configuration class
public class Config {

    // Changeable Power Values
    public final double kickerIdlePower = 0, kickerOnPower = 1,
        storeKickerPosition = 0.5, activeKickerPosition = 1,
        agitatorActivePower = 1, autoRotateSpeed = 0.3;

    public final int motorRampUpTime = 300;

    // This value will be changed with Limelight sensing to get the ideal power
    public double idealLauncherPower = 1;
    public double idealLauncherVelocity = 2500;

    // Reference to opMode class
    public final LinearOpMode opMode;

    // Define Motors
    public DcMotorEx fl, fr, bl, br,
            agitator, leftLauncher, rightLauncher,
            kickerMotor;

    public VoltageSensor batteryVoltageSensor;

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

    public LauncherThread launcherThread;
    public LimelightThread limelightThread;

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
    public Alliance alliance;

    // Selector Values
    public int startingPosition;
    public boolean runAuto;
    public int startingDelay;
    public int numberOfDrivers;

    // TweetyBird Classes
    public ThreeWheeled odometer;
    public Mecanum mecanum;
    public TweetyBird tweetyBird;

    // Pass opMode to config
    public Config(LinearOpMode opMode) {this.opMode = opMode;}

    /// Initialization Method
    public void init() {
        // Shorten HardwareMap for frequent use
        HardwareMap hwMap = opMode.hardwareMap;

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

        // Tilt Servos - Used to angle projectile
        //leftTilt = hwMap.get(Servo.class, "leftTilt");
        //rightTilt = hwMap.get(Servo.class, "rightTilt");

        // Limelight3A Camera
        limelight = hwMap.get(Limelight3A.class, "limelight");
        opMode.telemetry.setMsTransmissionInterval(11);

        // Battery
        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();


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
                .setDistanceBuffer(0.5) // Inch(es)
                .setDriver(mecanum)
                .setLinearOpMode(opMode)
                .setMaximumSpeed(0.3)
                .setMinimumSpeed(0.2)
                .setOdometer(odometer)
                .setRotationBuffer(8) // Degree(s)
                .setLoggingEnabled(true)
                .build();
        odometer.resetTo(0,0,0);
    }

    public void increaseLauncherPower() {
        idealLauncherPower = Range.clip(idealLauncherPower + .1,0.1,1);
    }

    public void decreaseLauncherPower() {
        idealLauncherPower = Range.clip(idealLauncherPower - .1,0.1,1);
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

    public void setWheelPowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
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

        // Also log to telemetry if opMode is available
        if (opMode != null && opMode.telemetry != null) {
            opMode.telemetry.log().add(message);
        }

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

}

