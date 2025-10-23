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
public class DevConfig {

    // Reference to opMode class
    private final LinearOpMode opMode;

    // Define Motors
    public DcMotor agitator, leftLauncher, rightLauncher;

    // Define Servos
    public Servo kicker, leftTilt, rightTilt;

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
    public DevConfig(LinearOpMode opMode){this.opMode = opMode;}

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
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
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
        kicker = hwMap.get(Servo.class, "kicker");
    }


    /// Set team color.
    public void setTeam(Team team){this.team = team;}

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
    }

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