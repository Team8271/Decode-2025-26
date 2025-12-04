package org.firstinspires.ftc.teamcode.robot;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "New Tele")
public class NewTele extends OpMode {
    NewConfig robot;
    private Follower follower;
    public static Pose startingPose = new Pose(0,0,0); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private boolean driverOneInterrupt = false;
    boolean debounce = false;
    boolean launcherDebounce = false;
    boolean changeAllianceDebounce = false;
    boolean intakeAssemblyIsActive = false;
    boolean intakeAssemblyIsReversed = false;

    @Override
    public void init() {
        robot = new NewConfig(null, this);
        robot.init();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower.breakFollowing();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Call this once per loop
        follower.update();
        telemetryM.update();



        // Normal Driver 1 Control

        double throttle = .2+(gamepad1.right_trigger*0.8); // throttle
        boolean resetFCD = gamepad1.dpad_up;

        if(resetFCD) {
            // Heading 180 if Blue
            // Heading 0 if Red
            double heading = 0;
            if(robot.alliance == NewConfig.Alliance.BLUE) {
                heading = 180;
            }
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), heading));
            gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(1,1,100)
                    .addStep(0,0,50)
                    .addStep(1,1,100)
                    .build());
            gamepad2.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(1,1,100)
                    .addStep(0,0,50)
                    .addStep(1,1,100)
                    .build());
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * throttle,
                -gamepad1.left_stick_x * throttle,
                -gamepad1.right_stick_x * throttle,
                false // Field Centric
        );


        // Automated PathFollowing
        if (gamepad1.dpadLeftWasPressed()) {
            // Go to nearest launch position
            Pose desiredPose = robot.aimAssist.getNearestPose(follower.getPose());
            
            follower.followPath(follower.pathBuilder()
                    .addPath(new BezierCurve(follower.getPose(), desiredPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), desiredPose.getHeading())
                    .build());
            automatedDrive = true;
        }

        // Checking for driver 1 interrupt
        if(automatedDrive && !gamepad1.atRest()) {
            driverOneInterrupt = true;
        }

        // Stop automated following if the driver interrupts
        if (automatedDrive && driverOneInterrupt) {
            follower.startTeleopDrive();
            automatedDrive = false;
            driverOneInterrupt = false;

        }

        // Driver 2 Controls
        boolean launchOneArtifact = gamepad2.a;
        boolean launchThreeArtifacts = gamepad2.y;
        boolean intakeAssemblyToggle = gamepad2.b;
        boolean reverseIntakeAssemblyToggle = gamepad2.x;

        boolean changeAlliance = gamepad2.options;

        if(changeAlliance && !changeAllianceDebounce) {
            switch(robot.alliance) {
                case RED:
                    robot.setAlliance(NewConfig.Alliance.BLUE);
                    break;
                case BLUE:
                    robot.setAlliance(NewConfig.Alliance.RED);
                    break;
            }
            gamepad2.rumble(200);
            changeAllianceDebounce = true;
        }
        if(!changeAlliance && changeAllianceDebounce) {
            changeAllianceDebounce = false;
        }

            /*

            if(increaseLauncherVelocity && !launcherVelocityDebounce) {
                robot.idealLauncherVelocity += 100;
                robot.log("Tele - Trying to increase launcher vel");
                launcherVelocityDebounce = true;
            }
            if(decreaseLauncherVelocity && !launcherVelocityDebounce) {
                robot.idealLauncherVelocity -= 100;
                robot.log("Tele - Trying to decrease launcher vel");
                launcherVelocityDebounce = true;
            }
            if (!increaseLauncherVelocity && !decreaseLauncherVelocity && launcherVelocityDebounce) {
                launcherVelocityDebounce = false;
            }

             */


        if (launchOneArtifact && !launcherDebounce) {
            robot.launcherThread.launch(1);
            gamepad1.rumble(0.8,0.8,125);
            gamepad2.rumble(0.8,0.8,125);
            launcherDebounce = true;
        }
        if (launchThreeArtifacts && !launcherDebounce) {
            robot.launcherThread.launch(3);
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
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addLine("" +
                "  Axial Control - Left Stick Y\n" +
                "  Lateral Control - Left Stick X\n" +
                "  Yaw Control - Right Stick X\n" +
                "  Throttle - Right Trigger\n" +
                "  FCD Reset - dPad Up\n" +
                "  Launch One Artifact - A\n" +
                "  Launch Three Artifacts - Y\n" +
                "  Agitator Assembly - B");

        telemetry.update();
    }
}