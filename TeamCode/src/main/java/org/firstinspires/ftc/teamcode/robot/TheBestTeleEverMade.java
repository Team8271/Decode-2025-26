package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.configuration.*;
import org.firstinspires.ftc.teamcode.util.Poses;
//@Configurable
@TeleOp(name = "! TheBestTeleEverMade !")
public class TheBestTeleEverMade extends LinearOpMode {
    Config robot;
    private Follower follower;
    //private Pose startPose; // Start Pose of robot.
    private Pose startPose = Poses.Blue.farStart; // Blue Far if no saved pose
    ElapsedTime runtime = new ElapsedTime();
    double lastAACalcTime = 0;
    double lastLauncherCalcTime = 0;

    public static double kP = 4;
    public static double kI = 0;
    public static double kD = 0.3;
    public static double kF = 0.01;


    // TODO: Limelight pose updating
    // TODO: Limelight correction but if no data switch to odometry

    boolean intakeAssemblyIsActive = false;
    boolean intakeAssemblyIsReversed = false;

    double targetHeading;
    //boolean correctHeading = false;
    boolean aimAssist = false;

    double lastTx = 0;

    @Override
    public void runOpMode() {
        robot = new Config(this);


        follower = Constants.createFollower(hardwareMap);
        follower.update();

        robot.init();
        robot.setAlliance(robot.readAllianceFromFile());
        // dev robot.setAlliance(Config.Alliance.BLUE);

        //TODO: Not working, Seems Red/Blue flipped
        //resetPose(0, 0, robot.alliance == Config.Alliance.RED ? 0 : Math.toRadians(180));

        if(robot.readPoseFromFile() != null) {
            startPose = robot.readPoseFromFile();
        }
        resetPose(startPose.getX(), startPose.getY(), startPose.getHeading());
        robot.invalidateSavedPose();

        // !! dev
        //resetPose(56, 12, Math.toRadians(90)); // Blue far start

        telemetry.addData("Alliance", robot.alliance);
        telemetry.update();
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Alliance", robot.alliance);

            // Driver 1 Controls
            double axialControl = -gamepad1.left_stick_y;  // y axis
            double lateralControl = gamepad1.left_stick_x; // x axis
            double yawControl = gamepad1.right_stick_x;    // z axis
            double throttle = .3+(gamepad1.right_trigger*0.8); // throttle
            boolean resetFCD = gamepad1.dpad_up; // z axis reset
            boolean enableAimAssist = gamepad1.cross;
            boolean brakeToggle = gamepad1.shareWasPressed();

            double allianceOffset = (robot.alliance == Config.Alliance.RED)
                    ? 0.0
                    : Math.PI;

            double gamepadRadians = Math.atan2(lateralControl, axialControl);
            double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
            double robotRadians = getHeading() + allianceOffset;
            double targetRadians = gamepadRadians + robotRadians;
            double lateral = Math.sin(targetRadians)*gamepadHypot;
            double axial = Math.cos(targetRadians)*gamepadHypot;

            //robot.aimAssist.headingPIDF.setCoefficient(kP,kI,kD,kF);

            if (brakeToggle) {
                robot.toggleBrakes();
            }

            if (enableAimAssist) {
                aimAssist = true;
            }

            if (Math.abs(yawControl) >= 0.1) {
                aimAssist = false;
            }

            if (resetFCD) {
                double heading = robot.alliance == Config.Alliance.RED ? 0 : Math.toRadians(180);
                resetPose(follower.getPose().getX(),follower.getPose().getY(),heading);
                gamepad1.rumble(100);
                log("FCD Reset");
            }

            if (robot.launcherThread.isBusy()) {
                double delay = 20;
                if (runtime.milliseconds() > lastLauncherCalcTime + delay) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                }
                lastLauncherCalcTime = runtime.milliseconds();
            }

            if (aimAssist) {
                // Aim Assist is active

                // Run intensive calculations every delay seconds
                double delay = 20;
                if (runtime.milliseconds() > lastAACalcTime + delay) {
                    double headingCalc = robot.aimAssist.getHeadingForTarget(follower.getPose(),robot.alliance.getPose());

                    double error = follower.getHeading()-headingCalc;

                    telemetry.addData("AA ERROR", error);

                    yawControl = robot.aimAssist.headingPIDF.calculate(error);

                    // Drive
                    setTeleOpDrive(axial, lateral, yawControl, throttle, false);

                    lastAACalcTime = runtime.milliseconds();
                }


            }
            else {
                // Driver 1 Controlling
                robot.aimAssist.headingPIDF.reset();

                // Drive
                setTeleOpDrive(axial, lateral, yawControl, throttle, true);
            }



            // Driver 2 Controls
            boolean launchOneArtifact = gamepad2.aWasPressed();
            boolean launchThreeArtifacts = gamepad2.yWasPressed();
            boolean intakeAssemblyToggle = gamepad2.bWasPressed();
            boolean reverseIntakeAssemblyToggle = gamepad2.xWasPressed();
            boolean changeAlliance = gamepad2.optionsWasPressed();
            //boolean cancelLaunch = gamepad2.dpadLeftWasPressed();
            boolean aimAssistOverrideToggle = gamepad2.dpadRightWasPressed();

            if(aimAssistOverrideToggle) {
                if(robot.aimAssist.getSimpleStatus()) {
                    robot.aimAssist.disableSimpleMode();
                    gamepad2.rumbleBlips(2);
                }
                else {
                    robot.aimAssist.enableSimpleMode();
                    gamepad2.rumbleBlips(3);
                }
            }

            //if(cancelLaunch) {
            //    log("Canceling launch");
            //    robot.launcherThread.cancelLaunch();
            //}

            if (launchOneArtifact) {
                robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                robot.launcherThread.launchOne();
                gamepad1.rumble(0.8,0.8,125);
                gamepad2.rumble(0.8,0.8,125);
            }
            if (launchThreeArtifacts) {
                robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                robot.launcherThread.launchThree();
                gamepad1.rumble(0.8,0.8,125);
                gamepad2.rumble(0.8,0.8,125);
            }

            if (intakeAssemblyToggle) {
                if(!intakeAssemblyIsActive || intakeAssemblyIsReversed) {
                    robot.runIntakeAssembly();
                    intakeAssemblyIsActive = true;
                }
                else {
                    robot.stopIntakeAssembly();
                    intakeAssemblyIsActive = false;
                }
                intakeAssemblyIsReversed = false;
            }
            if (reverseIntakeAssemblyToggle) {
                robot.reverseIntakeAssembly();
                intakeAssemblyIsReversed = true;

            }

            //
            if(robot.intakeMotor.getVelocity() > 70) { // Running
                robot.indicatorLight.setPosition(robot.indicatorLightOn);
            }
            else { // Not Running
                robot.indicatorLight.setPosition(robot.indicatorLightOff);
            }



            if(changeAlliance) {
                switch(robot.alliance) {
                    case RED:
                        robot.setAlliance(Config.Alliance.BLUE);
                        break;
                    case BLUE:
                        robot.setAlliance(Config.Alliance.RED);
                        break;
                }
                gamepad2.rumble(200);
            }

            telemetry.addData("Launcher Velocity", robot.launcherThread.getLauncherVelocity());
            telemetry.addData("Intake Vel", robot.intakeMotor.getVelocity());
            telemetry.addData("Position", follower.getPose());

            // One Driver Telemetry
            telemetry.addLine(
                    "  Gamepad1:\n" +
                    "    Axial Control - Left Stick Y\n" +
                    "    Lateral Control - Left Stick X\n" +
                    "    Yaw Control - Right Stick X\n" +
                    "    Throttle - Right Trigger\n" +
                    "    FCD Reset - dPad Up\n" +
                    "  Gamepad2:\n" +
                    "    Launch One Artifact - A\n" +
                    "    Launch Three Artifacts - Y\n" +
                    "    Agitator Assembly - B\n" +
                    "    Simple Mode - DPad-Right");

            telemetry.update();
            sleep(1); // Save resources
        }
        //robot.savePoseToFile(follower.getPose());
    }

    private void runSelector() {

        // Read last alliance from file
        robot.alliance = robot.readAllianceFromFile();

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.dpadDownWasPressed()) {

                switch (robot.alliance) {
                    case BLUE:
                        robot.setAlliance(Config.Alliance.RED);
                        break;
                    case RED:
                        robot.setAlliance(Config.Alliance.BLUE);
                        break;
                }

            }

            telemetry.addData("Alliance", robot.alliance.toString());
            telemetry.addLine("\nPress START");
            telemetry.update();

            // Don't hog system
            sleep(50);
        }
        robot.saveAllianceToFile(robot.alliance);
    }

    private void setTeleOpDrive(double axial, double lateral, double yaw, double throttle, boolean throttleEffectsYaw) {
        double axialScaled = axial * throttle;
        double lateralScaled = lateral * throttle;
        double yawScaled = throttleEffectsYaw ? yaw * throttle : yaw;

        double leftFrontPower  = axialScaled + lateralScaled + yawScaled;
        double rightFrontPower = axialScaled - lateralScaled - yawScaled;
        double leftBackPower   = axialScaled - lateralScaled + yawScaled;
        double rightBackPower  = axialScaled + lateralScaled - yawScaled;

        double max = Math.max(1.0,
                    Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(rightFrontPower),
                    Math.max(Math.abs(leftBackPower),
                    Math.abs(rightBackPower)))));

        robot.fl.setPower(leftFrontPower / max);
        robot.fr.setPower(rightFrontPower / max);
        robot.bl.setPower(leftBackPower / max);
        robot.br.setPower(rightBackPower / max);
    }

    private double getX() {
        return follower.getPose().getX();
    }
    private double getY() {
        return follower.getPose().getY();
    }
    private double getHeading() {
        return follower.getPose().getHeading();
    }
    private void resetPose(double x, double y, double heading) {
        follower.setPose(new Pose(x,y,heading));
    }

    private void log(String message) {
        robot.log("[TBTEM] - " + message);
    }

}
