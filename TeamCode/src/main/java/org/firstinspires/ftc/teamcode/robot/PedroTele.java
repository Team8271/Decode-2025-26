package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Pedro Tele")
public class PedroTele extends OpMode {
    Config robot;
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    boolean debounce = false;
    boolean launcherDebounce = false;
    boolean changeAllianceDebounce = false;
    boolean intakeAssemblyIsActive = false;
    boolean intakeAssemblyIsReversed = false;

    double targetCorrectionHeading;
    boolean correctHeading = false;

    @Override
    public void init() {

        robot = new Config(null, this);
        robot.init();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine(""+
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
                "    Increase Launcher Power - dPad Up\n" +
                "    Decrease Launcher Power - dPad Down");

        telemetryM.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        // Call this once per loop
        follower.update();
        telemetryM.update();

        // Driver 1 Controls
        double axialControl = -gamepad1.left_stick_y;  // y axis
        double lateralControl = -gamepad1.left_stick_x; // x axis
        double yawControl = -gamepad1.right_stick_x;    // z axis
        double throttle = .2+(gamepad1.right_trigger*0.8); // throttle
        boolean resetFCD = gamepad1.dpad_up; // z axis reset

        axialControl *= throttle;
        lateralControl *= throttle;
        yawControl *= throttle;

        if(correctHeading && yawControl == 0) {

            double currentHeading = follower.getHeading();
            double targetHeading = targetCorrectionHeading;

            double error = AimAssist.angleWrap(targetHeading - currentHeading);

            double kP = 1;
            double turnPower = kP * error;

            turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

            follower.setTeleOpDrive(axialControl,lateralControl,turnPower,false);
            log("Correcting heading with turnpower of: " + turnPower);
        }
        else {
            follower.setTeleOpDrive(axialControl,lateralControl,yawControl,false);
        }

        if (resetFCD) {
            // 0 if RED
            // 180 if BLUE
            double resetHeading = Math.toRadians(180);
            if(robot.alliance == Config.Alliance.RED) {
                resetHeading = 0;
            }
            follower.setPose(new Pose(follower.getPose().getX(),follower.getPose().getY(), resetHeading));
            gamepad1.rumble(100);
            log("FCD Reset");
        }

        // Have launch start auto correct

        // Driver 2 Controls
        boolean launchOneArtifact = gamepad2.a;
        boolean launchThreeArtifacts = gamepad2.y;
        boolean intakeAssemblyToggle = gamepad2.b;
        boolean reverseIntakeAssemblyToggle = gamepad2.x;
        boolean changeAlliance = gamepad2.options;

        if (launchOneArtifact && !launcherDebounce) {
            robot.launcherThread.launchOne();
            gamepad1.rumble(0.8,0.8,125);
            gamepad2.rumble(0.8,0.8,125);
            launcherDebounce = true;
        }
        if (launchThreeArtifacts && !launcherDebounce) {
            robot.launcherThread.launchThree();
            gamepad1.rumble(0.8,0.8,125);
            gamepad2.rumble(0.8,0.8,125);
            launcherDebounce = true;
        }
        if (!launchOneArtifact && !launchThreeArtifacts && launcherDebounce) {
            launcherDebounce = false;
        }

        correctHeading = false;
        if(robot.launcherThread.isBusy()) {
            targetCorrectionHeading = robot.aimAssist.runHeadingCalculation(follower.getPose());
            correctHeading = true;
            log("Starting to correct heading / Still correcting heading");
        }

        if (intakeAssemblyToggle && !debounce) {
            if(!intakeAssemblyIsActive || intakeAssemblyIsReversed) {
                robot.runIntakeAssembly();
                intakeAssemblyIsActive = true;
            }
            else {
                robot.stopIntakeAssembly();
                intakeAssemblyIsActive = false;
            }

            intakeAssemblyIsReversed = false;
            debounce = true;
        }
        if (reverseIntakeAssemblyToggle && !debounce) {
            robot.reverseIntakeAssembly();
            intakeAssemblyIsReversed = true;

        }
        if (!intakeAssemblyToggle && !reverseIntakeAssemblyToggle && debounce) {
            debounce = false;
        }

        if(changeAlliance && !changeAllianceDebounce) {
            switch(robot.alliance) {
                case RED:
                    robot.setAlliance(Config.Alliance.BLUE);
                    break;
                case BLUE:
                    robot.setAlliance(Config.Alliance.RED);
                    break;
            }
            gamepad2.rumble(200);
            changeAllianceDebounce = true;
        }
        if(!changeAlliance && changeAllianceDebounce) {
            changeAllianceDebounce = false;
        }

    }
    private void log(String message) {
        robot.log("[PedroTele] - " + message);
    }
}
