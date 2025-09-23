package org.firstinspires.ftc.teamcode.JaxDev;

import android.bluetooth.le.ScanSettings;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final int limelightObeliskPipeline = 0;

    // enums
    public enum Motif {
        GPP,    // AprilTag 21
        PGP,    // AprilTag 22
        PPG,    // AprilTag 23
        NULL;
    }

    // TweetyBird Classes
    public ThreeWheeled odometer;
    public Mecanum mecanum;
    public TweetyBird tweetyBird;

    // Pass opMode to config
    public JaxDevConfig(LinearOpMode opMode){this.opMode = opMode;}

    /// Initialization Method
    public void init(){
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

    /// Initializes TweetyBird
    public void initTweetyBird(){
        tweetyBird = new TweetyBird.Builder()
                .setDistanceBuffer(1) // Inch(es)
                .setDriver(mecanum)
                .setLinearOpMode(opMode)
                .setMaximumSpeed(0.7)
                .setMinimumSpeed(0.4)
                .setOdometer(odometer)
                .setRotationBuffer(4) // Degree(s)
                .setLoggingEnabled(true)
                .build();
    }

    /**
     * Uses Limelight to detect Obelisk Motif pattern and returns result.
     *
     * @return Valid Motif or Motif.NULL
     */
    public Motif scanObelisk(){
        int aprilTag = 0;

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
                return Motif.GPP;
            case(22):
                return Motif.PGP;
            case(23):
                return Motif.PPG;
            default:
                return Motif.NULL;

        }

    }

}