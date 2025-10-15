package org.firstinspires.ftc.teamcode.NewBot;

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

import java.util.List;

import dev.narlyx.tweetybird.Drivers.Mecanum;
import dev.narlyx.tweetybird.Odometers.ThreeWheeled;
import dev.narlyx.tweetybird.TweetyBird;

/// Configuration class
public class Config {

    // Reference to opMode class
    private final LinearOpMode opMode;

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
    public Config(LinearOpMode opMode){this.opMode = opMode;}

    public void devInit(){
        // Shorten HardwareMap for frequent use
        HardwareMap hwMap = opMode.hardwareMap;


        // Motor used in the Active Agitator module
        agitator = hwMap.get(DcMotor.class, "agitator");
        agitator.setDirection(DcMotorSimple.Direction.FORWARD);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Robot Facing Left Launcher Motor
        leftLauncher = hwMap.get(DcMotor.class, "leftLauncher");
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Robot Facing Right Launcher Motor
        rightLauncher = hwMap.get(DcMotor.class, "rightLauncher");
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tilt Servos - Used to angle projectile
        leftTilt = hwMap.get(Servo.class, "leftTilt");
        rightTilt = hwMap.get(Servo.class, "rightTilt");
    }

    /// Initialization Method
    public void init(){
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
        intakeServo = hwMap.get(Servo.class, "intakeServo");


        // Tilt Servos - Used to angle projectile
        //leftTilt = hwMap.get(Servo.class, "leftTilt");
        //rightTilt = hwMap.get(Servo.class, "rightTilt");

        // Limelight3A Camera
        // limelight = hwMap.get(Limelight3A.class, "limelight");
        // opMode.telemetry.setMsTransmissionInterval(11);

        launcherThread = new LauncherThread();
        launcherThread.setConfig(this);
        launcherThread.start();


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
    }

    /// Initializes TweetyBird.
    public void initTweetyBird(){
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

    public void setLauncherPower(double power){
        leftLauncher.setPower(power);
        rightLauncher.setPower(power);
    }

    /// Set team for launching system etc.
    public void setTeam(Team team){this.team = team;}

    /// Uses Limelight to detect Obelisk Motif pattern and updates Motif.motif.
    public void scanObelisk(){
        int aprilTag = 0;
        Motif tempMotif;

        if(!limelight.isRunning()){
            limelight.start();
        }
        if(limelight.getStatus().getPipelineIndex() != limelightObeliskPipeline){
            limelight.pipelineSwitch(limelightObeliskPipeline);
        }
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()){
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            aprilTag = fiducialResults.get(0).getFiducialId();
        }

        switch(aprilTag){
            case(21):
                tempMotif = Motif.GPP;
                break;
            case(22):
                tempMotif = Motif.PGP;
                break;
            case(23):
                tempMotif = Motif.PPG;
                break;
            default:
                tempMotif = Motif.NULL;
                break;
        }
        if(tempMotif != Motif.NULL){
            motif = tempMotif;
        }

    }

    public Motif getMotif(){return motif;}

    /// Uses Limelight to detect Goal angle and update goalTx and goalTy.
    public void scanGoalAngle(){
        int selectedPipeline;

        // Ensure polling for limelight data
        if(!limelight.isRunning()){
            limelight.start();
        }

        // Set desired pipeline
        switch(team){
            case BLUE:
                selectedPipeline = limelightBluePipeline;
                break;
            case RED:
                selectedPipeline = limelightRedPipeline;
                break;
            default:
                selectedPipeline = 4;
        }

        // Set pipeline
        if(limelight.getStatus().getPipelineIndex() != selectedPipeline){
            limelight.pipelineSwitch(selectedPipeline);
        }

        // Get latest results
        LLResult result = limelight.getLatestResult();

        // Update angles
        if(result != null && result.isValid()){
            goalAnglesAreValid = true;
            goalTx = result.getTx();
            goalTy = result.getTy();
        }
        else{
            goalAnglesAreValid = false;
        }

    }

}

class LauncherThread extends Thread{
    boolean isRunning = false;
    boolean hasConfig = false;
    Config robot;
    public void setConfig(Config robot){this.robot = robot; hasConfig = true;}
    public void run(){
        isRunning = true;
        System.out.println("Thread is running: " + Thread.currentThread().getName());

    }
    public void launchOneArtifact(){
        if(hasConfig){
            try {
                if(!isRunning){
                    start();
                }
                robot.setLauncherPower(1);
                robot.kickerMotor.setPower(1);
                sleep(100);
                robot.kickerServo.setPosition(1);
                sleep(700);
                robot.setLauncherPower(0);
                robot.kickerMotor.setPower(0);
                robot.kickerServo.setPosition(0.5);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}