package org.firstinspires.ftc.teamcode.util.tuners;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.teamcode.robot.configuration.Config;
import org.firstinspires.ftc.teamcode.util.Poses;

@TeleOp(name = "!!FlywheelTuner!!")
public class FlywheelPFTuner extends OpMode {
    public DcMotorEx flywheelMotor, agitator;

    Config robot;
    private Follower follower;

    private final Pose startPose = Poses.Blue.farStart; // Start Pose of robot.

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double P = 130;
    double F = 14.3;

    double[] stepSizes = {50.0, 25, 10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    LLResult result;
    boolean valid;
    Pose3D blankPose = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    Pose3D lLCurPose = blankPose;
    Pose3D lLCurPoseMT2 = blankPose;
    Pose3D lLLastPose = blankPose;
    Pose3D lLLastPoseMT2 = blankPose;

    private final double
            storeLeftKickerPosition = 0.6, storeRightKickerPosition = 1 - storeLeftKickerPosition,
            activeLeftKickerPosition = 0, activeRightKickerPosition = 1 - activeLeftKickerPosition,
            intakeLimServerActivePosition = 1, intakeLimServoInactivePosition = 0.5;

    private Servo leftKickerServo, rightKickerServo, intakeLimServo;

    @Override
    public void init() {

        robot = new Config(this);
        //robot.aimAssistInit();
        robot.init();

        robot.alliance = Config.Alliance.BLUE;

        //initLL();

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

        /* Stolen Code from Examples
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            /

        // Send calculated power to wheels
        robot.fl.setPower(frontLeftPower);
        robot.fr.setPower(frontRightPower);
        robot.bl.setPower(backLeftPower);
        robot.br.setPower(backRightPower);

        // End of stolen code*/

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

        double distance = robot.aimAssist.getPoseDistance(follower.getPose(), robot.alliance.getPose());

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
        leftKickerServo.setPosition(activeLeftKickerPosition);
        rightKickerServo.setPosition(activeRightKickerPosition);
    }

    public void waitForKicker() throws InterruptedException {
        Thread.sleep(200);
    }

    public void storeKicker() {
        leftKickerServo.setPosition(storeLeftKickerPosition);
        rightKickerServo.setPosition(storeRightKickerPosition);
    }

    public void updateLLResults() {
        result = null;//limelight.getLatestResult();

        if (result != null && result.isValid()) {
            valid = true;
            lLCurPose = result.getBotpose();
            lLCurPoseMT2 = result.getBotpose_MT2();
            lLLastPose = lLCurPose;
            lLLastPoseMT2 = lLCurPoseMT2;
        } else {
            valid = false;
        }
    }
}
