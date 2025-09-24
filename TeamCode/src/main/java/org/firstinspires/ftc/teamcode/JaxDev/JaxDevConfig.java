package org.firstinspires.ftc.teamcode.JaxDev;

import android.bluetooth.le.ScanSettings;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.narlyx.tweetybird.Drivers.Mecanum;
import dev.narlyx.tweetybird.Odometers.ThreeWheeled;
import dev.narlyx.tweetybird.TweetyBird;

/// Jax's development configuration class
public class JaxDevConfig {

    // Reference to opMode class
    private final LinearOpMode opMode;

    // Defining Motors
    public DcMotor fl, fr, bl, br;

    // Other Hardware
    public Limelight3A limelight;

    // Variables
    private final int   limelightLocalizationPipeline   = 0,
                        limelightObeliskPipeline        = 1,
                        limelightRedPipeline            = 2,
                        limelightBluePipeline           = 3;

    public boolean goalAnglesAreValid;
    public double goalTx;
    public double goalTy;
    public Motif motif;


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
    public JaxDevConfig(LinearOpMode opMode){this.opMode = opMode;}

    /// Initialization Method
    public void init(){
        motif = Motif.NULL;
        setTeam(Team.BLUE);

        // Shorten HardwareMap for frequent use
        HardwareMap hwMap = opMode.hardwareMap;

        // Front Left Drive
        fl = hwMap.get(DcMotor.class, "FL");
        fl.setDirection(DcMotor.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Front Right Drive
        fr = hwMap.get(DcMotor.class, "FR");
        fr.setDirection(DcMotor.Direction.FORWARD);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Left Drive
        bl = hwMap.get(DcMotor.class, "BL");
        bl.setDirection(DcMotor.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Right Drive
        br = hwMap.get(DcMotor.class, "BR");
        br.setDirection(DcMotor.Direction.FORWARD);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

                .setSideEncoderDistance(12)
                .setMiddleEncoderOffset(9.75)
                .build();
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
                .setMinimumSpeed(0.4)
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

    /// Uses Limelight to detect Goal angle and update goalTx and goalTy.
    public void scanGoalAngle(){
        int desiredPipeline;

        // Ensure polling for limelight data
        if(!limelight.isRunning()){
            limelight.start();
        }

        // Set desired pipeline
        switch(team){
            case BLUE:
                desiredPipeline = limelightBluePipeline;
                break;
            case RED:
                desiredPipeline = limelightRedPipeline;
                break;
            default:
                desiredPipeline = 4;
        }

        // Set pipeline
        if(limelight.getStatus().getPipelineIndex() != desiredPipeline){
            limelight.pipelineSwitch(desiredPipeline);
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