package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "!!FlywheelTuner!!")
public class FlywheelPFTuner extends OpMode {
    public DcMotorEx flywheelMotor, agitator;

    Config robot;
    private Follower follower;

    private final Pose startPose = new Pose(88, 12, Math.toRadians(90)); // Start Pose of robot.

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double P = 130;
    double F = 14.3;

    double[] stepSizes = {50.0, 25, 10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    Limelight3A limelight;
    LLResult result;
    boolean valid;
    Pose3D blankPose = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.RADIANS,0,0,0,0));
    Pose3D lLCurPose = blankPose;
    Pose3D lLCurPoseMT2 = blankPose;
    Pose3D lLLastPose = blankPose;
    Pose3D lLLastPoseMT2 = blankPose;

    private final double
            storeLeftKickerPosition = 0.6, storeRightKickerPosition = 1 - storeLeftKickerPosition,
            activeLeftKickerPosition = 0, activeRightKickerPosition = 1 - activeLeftKickerPosition,
            intakeLimServerActivePosition = 1, intakeLimServoInactivePosition = 0.5;

    private double desiredLeftKickerPosition = storeLeftKickerPosition,
            desiredIntakeLimiterPosition = intakeLimServerActivePosition;

    private Servo leftKickerServo, rightKickerServo, intakeLimServo;

    @Override
    public void init() {

        robot = new Config(this);
        robot.aimAssistInit();
        robot.alliance = Config.Alliance.RED;

        initLL();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        // Modified Robot Launcher Motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Kicker Servo - Transfers artifacts from agitator to launcher
        leftKickerServo = hardwareMap.get(Servo.class, "lKickerServo");
        leftKickerServo.setPosition(storeLeftKickerPosition);
        rightKickerServo = hardwareMap.get(Servo.class, "rKickerServo");
        rightKickerServo.setPosition(storeRightKickerPosition);
        // IntakeLimiterServo
        intakeLimServo = hardwareMap.get(Servo.class, "intakeLimServo");
        intakeLimServo.setPosition(intakeLimServerActivePosition);

        agitator = hardwareMap.get(DcMotorEx.class, "agitator");
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeLimServo.setPosition(intakeLimServoInactivePosition);

        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.xWasPressed()) {
            highVelocity += stepSizes[stepIndex];
        }
        if (gamepad1.aWasPressed()) {
            highVelocity -= stepSizes[stepIndex];
        }

        if (gamepad1.optionsWasPressed()) {
            activateKicker();
            try {
                waitForKicker();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            storeKicker();
        }

        if (gamepad1.shareWasPressed()) {
            if (agitator.getPower() == 1) {
                agitator.setPower(0);
            } else {
                agitator.setPower(1);
            }
        }

        // Set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set Velocity
        flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        double distance = robot.aimAssist.getPoseDistance(follower.getPose(),robot.alliance.getPose());

        updateLLResults();

        telemetry.addData("Alliance", robot.alliance);
        telemetry.addData("Distance", distance);
        telemetry.addData("High Velocity", highVelocity);
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f [D-Pad U/D}", P);
        telemetry.addData("Tuning F", "%.4f [D-Pad L/R]", F);
        telemetry.addData("Step Size", "%.4f [B Button]", stepSizes[stepIndex]);
        telemetry.addLine("-----------------------------");
        try {
            telemetry.addData("Od:CurPose", "%.2f, %.2f, %.2f",
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getPose().getHeading());
        } catch (NullPointerException ignorned) {
            telemetry.addLine("Od:CurPose: Not Available");
        }
        telemetry.addData("LL:CurPose", "%.2f, %.2f, %.2f", lLCurPose.getPosition().x,
                lLCurPose.getPosition().y,
                lLCurPose.getPosition().z);
        telemetry.addData("LL:CurPoseMT2", "%.2f, %.2f, %.2f", lLCurPoseMT2.getPosition().x,
                lLCurPoseMT2.getPosition().y,
                lLCurPoseMT2.getPosition().z);
        telemetry.addData("LL:Validity", valid);
        telemetry.addData("LL:LastPose", "%.2f, %.2f, %.2f", lLLastPose.getPosition().x,
                lLLastPose.getPosition().y,
                lLLastPose.getPosition().z);
        telemetry.addData("LL:LastPoseMT2", "%.2f, %.2f, %.2f", lLLastPoseMT2.getPosition().x,
                lLLastPoseMT2.getPosition().y,
                lLLastPoseMT2.getPosition().z);
        telemetry.update();

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

    public void initLL() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(11);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void updateLLResults() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            valid = true;
            lLCurPose = result.getBotpose();
            lLCurPoseMT2 = result.getBotpose_MT2();
            lLLastPose = lLCurPose;
            lLLastPoseMT2 = lLCurPoseMT2;
        } else { valid = false; }
    }
}
