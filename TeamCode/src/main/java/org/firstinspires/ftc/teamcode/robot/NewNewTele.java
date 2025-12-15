package org.firstinspires.ftc.teamcode.robot;

// This is the good one ~Cool Charles

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "NewNEW Tele")
public class NewNewTele extends OpMode {
    Config robot;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private boolean driverOneInterrupt = false;
    boolean debounce = false;
    boolean launcherDebounce = false;
    boolean changeAllianceDebounce = false;
    boolean devDebounce = false;
    boolean intakeAssemblyIsActive = false;
    boolean intakeAssemblyIsReversed = false;
    boolean aimAssisting = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        robot = new Config(null, this);
        robot.init();

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
        // Called before entering loop
    }

    @Override
    public void loop() {
        // Call this once per loop
        follower.update();
        telemetryM.update();

        // Driver 1 Controls
        double axialControl = -gamepad1.left_stick_y;  // y axis
        double lateralControl = gamepad1.left_stick_x; // x axis
        double yawControl = gamepad1.right_stick_x;    // z axis
        double throttle = .2+(gamepad1.right_trigger*0.8); // throttle
        boolean resetFCD = gamepad1.dpad_up; // z axis reset

        if (resetFCD) {
            robot.odometer.resetTo(0,0,0);
            gamepad1.rumble(100);
        }

        // Calculate drive train power for field centric
        double gamepadRadians = Math.atan2(lateralControl, axialControl);
        double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
        double robotRadians = -robot.odometer.getZ();
        double targetRadians = gamepadRadians + robotRadians;
        double lateral = Math.sin(targetRadians)*gamepadHypot;
        double axial = Math.cos(targetRadians)*gamepadHypot;

        double leftFrontPower = axial + lateral + yawControl;
        double rightFrontPower = axial - lateral - yawControl;
        double leftBackPower = axial - lateral + yawControl;
        double rightBackPower = axial + lateral - yawControl;

        // Calculate and send power to drivetrain
        robot.fl.setPower(leftFrontPower * throttle);
        robot.fr.setPower(rightFrontPower * throttle);
        robot.bl.setPower(leftBackPower * throttle);
        robot.br.setPower(rightBackPower * throttle);


        // Checking for driver 1 interrupt
        if(aimAssisting && !gamepad1.atRest()) {
            driverOneInterrupt = true;
        }


        // AimAssist
        if (gamepad1.dpadLeftWasPressed()) {
            aimAssisting = true;
        }

        // Run the looping aim assist
        if(aimAssisting && !driverOneInterrupt) {
            // Aim Assist here (remember your in a loop) and you can pass
            // position using follow.getPose()
            double desiredHeading = robot.aimAssist.runHeadingCalculation(follower.getPose());

            Pose currentPose = follower.getPose();
            Pose targetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), desiredHeading);
            follower.followPath(new Path(new BezierLine(currentPose, targetPose)));
            // !! I don't think this will work but... ah it'll work

        }

        // Driver 2 Controls
        boolean launchOneArtifact = gamepad2.a;
        boolean launchThreeArtifacts = gamepad2.y;
        boolean intakeAssemblyToggle = gamepad2.b;
        boolean reverseIntakeAssemblyToggle = gamepad2.x;
        boolean toggleLimiter = gamepad2.back;

        boolean changeAlliance = gamepad2.options;

        if(toggleLimiter && !devDebounce) {
            if(robot.devBool) {
                robot.deactivateIntakeLimiter();
            }
            else {
                robot.activateIntakeLimiter();
            }
            devDebounce = true;
        }
        if(!toggleLimiter && devDebounce) {
            devDebounce = false;
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

        if (launchOneArtifact && !launcherDebounce) {
            //robot.launcherThread.launch(1);
            gamepad1.rumble(0.8,0.8,125);
            gamepad2.rumble(0.8,0.8,125);
            launcherDebounce = true;
        }
        if (launchThreeArtifacts && !launcherDebounce) {
            //robot.launcherThread.launch(3);
            gamepad1.rumble(0.8,0.8,125);
            gamepad2.rumble(0.8,0.8,125);
            launcherDebounce = true;
        }
        if (!launchOneArtifact && !launchThreeArtifacts && launcherDebounce) {
            launcherDebounce = false;
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

        telemetryM.debug("position", follower.getPose());

        // Mixed Controls
        telemetryM.addLine(""+
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
    }
}