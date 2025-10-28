package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

import dev.narlyx.tweetybird.Drivers.Mecanum;
import dev.narlyx.tweetybird.Odometers.ThreeWheeled;
import dev.narlyx.tweetybird.TweetyBird;

/// Configuration class
public class Config {

    private static final Logger log = LoggerFactory.getLogger(Config.class);

    // Log file writer
    protected BufferedWriter logWriter = null;

    // Changeable Power Values
    public final double kickerIdlePower = 0.3, kickerOnPower = 1,
        storeKickerPosition = 0.5, activeKickerPosition = 1,
        agitatorActivePower = 1;

    public final int motorRampUpTime = 200;

    // This value will be changed with Limelight sensing to get the ideal power
    public double idealLauncherPower = 1;

    // Reference to opMode class
    public final LinearOpMode opMode;

    // Define Motors
    public DcMotor fl, fr, bl, br,
            agitator, leftLauncher, rightLauncher,
            kickerMotor;

    // Define Servos
    public Servo kickerServo, leftTilt, rightTilt, intakeServo;

    // Other Hardware
    public Limelight3A limelight;
    public IMU imu;

    // Variables
    private final int   limelightLocalizationPipeline   = 0,
                        limelightObeliskPipeline        = 1,
                        limelightRedPipeline            = 2,
                        limelightBluePipeline           = 3;

    public boolean goalAnglesAreValid;
    public double goalTx;
    public double goalTy;
    private Motif motif;
    public LauncherThread launcherThread;

    // enums
    public enum Motif {
        GPP,    // AprilTag 21
        PGP,    // AprilTag 22
        PPG,    // AprilTag 23
        NULL;
    }
    public enum Team {
        RED,
        BLUE;
    }
    Team team;

    // TweetyBird Classes
    public ThreeWheeled odometer;
    public Mecanum mecanum;
    public TweetyBird tweetyBird;

    // Pass opMode to config
    public Config(LinearOpMode opMode) {this.opMode = opMode;}

    /// Initialization Method
    public void init() {
        motif = Motif.NULL;
        setTeam(Team.BLUE);

        // Shorten HardwareMap for frequent use
        HardwareMap hwMap = opMode.hardwareMap;

        // IMU
        imu = hwMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Front Left Drive
        fl = hwMap.get(DcMotor.class, "fl");
        fl.setDirection(DcMotor.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Front Right Drive
        fr = hwMap.get(DcMotor.class, "fr");
        fr.setDirection(DcMotor.Direction.FORWARD);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Left Drive
        bl = hwMap.get(DcMotor.class, "bl");
        bl.setDirection(DcMotor.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Right Drive
        br = hwMap.get(DcMotor.class, "br");
        br.setDirection(DcMotor.Direction.FORWARD);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor used in the Active Agitator module
        agitator = hwMap.get(DcMotor.class, "agitator");
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Robot Facing Left Launcher Motor
        leftLauncher = hwMap.get(DcMotor.class, "leftLauncher");
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Robot Facing Right Launcher Motor
        rightLauncher = hwMap.get(DcMotor.class, "rightLauncher");
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Kicker Motor "spin-ny thing"
        kickerMotor = hwMap.get(DcMotor.class, "kickerMotor");
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

        // Starts Threads
        checkAndRestartThreads();

        // Tilt Servos - Used to angle projectile
        //leftTilt = hwMap.get(Servo.class, "leftTilt");
        //rightTilt = hwMap.get(Servo.class, "rightTilt");

        // Limelight3A Camera
        // limelight = hwMap.get(Limelight3A.class, "limelight");
        // opMode.telemetry.setMsTransmissionInterval(11);


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

                .setSideEncoderDistance(10.75)
                .setMiddleEncoderOffset(9.75)
                .build();

        log("Robot Initialized");
    }

    /// Initializes TweetyBird.
    public void initTweetyBird() {
        log("TweetyBird Initialized");
        tweetyBird = new TweetyBird.Builder()
                .setDistanceBuffer(1) // Inch(es)
                .setDriver(mecanum)
                .setLinearOpMode(opMode)
                .setMaximumSpeed(0.5)
                .setMinimumSpeed(0.2)
                .setOdometer(odometer)
                .setRotationBuffer(4) // Degree(s)
                .setLoggingEnabled(true)
                .build();
        odometer.resetTo(0,0,180);
    }

    /// Set team for launching system etc.
    public void setTeam(Team team) {
        log("Team set to " + team);
        this.team = team;
    }

    /// Uses Limelight to detect Obelisk Motif pattern and updates Motif.motif.
    public void scanObelisk() {
        int aprilTag = 0;
        Motif tempMotif;

        if (!limelight.isRunning()) {
            log("Limelight: Starting");
            limelight.start();
        }
        if (limelight.getStatus().getPipelineIndex() != limelightObeliskPipeline) {
            log("Limelight: Pipeline changed to OBELISK");
            limelight.pipelineSwitch(limelightObeliskPipeline);
        }
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            aprilTag = fiducialResults.get(0).getFiducialId();
        }

        switch (aprilTag) {
            case (21):
                tempMotif = Motif.GPP;
                break;
            case (22):
                tempMotif = Motif.PGP;
                break;
            case (23):
                tempMotif = Motif.PPG;
                break;
            default:
                tempMotif = Motif.NULL;
                break;
        }
        if (tempMotif != Motif.NULL) {
            log("Motif changed to: " + motif);
            motif = tempMotif;
        }

    }

    /// @return The last recognised motif.
    public Motif getMotif() {return motif;}

    /// Uses Limelight to detect Goal angle and update goalTx and goalTy.
    public void scanGoalAngle() {
        int selectedPipeline;

        // Ensure polling for limelight data
        if (!limelight.isRunning()) {
            log("Limelight: Starting");
            limelight.start();
        }

        // Set desired pipeline
        switch (team) {
            case BLUE:
                log("Limelight: Pipeline change: BLUE");
                selectedPipeline = limelightBluePipeline;
                break;
            case RED:
                log("Limelight: Pipeline change: RED");
                selectedPipeline = limelightRedPipeline;
                break;
            default:
                log("Limelight: Pipeline change: 4");
                selectedPipeline = 4;
                break;
        }

        // Set pipeline
        if (limelight.getStatus().getPipelineIndex() != selectedPipeline){
            limelight.pipelineSwitch(selectedPipeline);
        }

        // Get latest results
        LLResult result = limelight.getLatestResult();

        // Update angles
        if (result != null && result.isValid()) {
            goalAnglesAreValid = true;
            goalTx = result.getTx();
            goalTy = result.getTy();
        }
        else {
            goalAnglesAreValid = false;
        }

    }

    public void increaseLauncherPower() {
        idealLauncherPower = Range.clip(idealLauncherPower + .1,0.1,1);
    }

    public void decreaseLauncherPower() {
        idealLauncherPower = Range.clip(idealLauncherPower - .1,0.1,1);
    }

    protected void log(String message) {
        // Getting current time
        Date now = new Date();
        SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/YYYY hh:mm:ss.SSS");
        String date = sdf.format(now);

        // Processing string
        String outputString = "["+date+" Config]: "+message;

        // Logfile
        try {
            logWriter.write(outputString);
            logWriter.newLine();
        } catch (IOException ignored) {}
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

}

/**
 * Thread class used to launch artifacts.
 * Must be provided a Config using 'setConfig().'
 */
class LauncherThread extends Thread {
    Config robot;
    public void setConfig(Config robot) {this.robot = robot;}

    private int artifactsToLaunch = 0;

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

            doLaunch(artifactsToLaunch);

        }
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
