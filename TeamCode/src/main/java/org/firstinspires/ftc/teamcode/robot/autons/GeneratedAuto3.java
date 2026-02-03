package org.firstinspires.ftc.teamcode.robot.autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;
import org.firstinspires.ftc.teamcode.util.Poses;

@Autonomous(name = "Generated Auto 3")
public class GeneratedAuto3 extends OpMode {

    Config robot;
    ElapsedTime runtime = new ElapsedTime();
    boolean waitingForLauncher = false;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private double gateEndTime;

    private final Pose farStart = Poses.Blue.farStart;

    private final Pose closeScore = Poses.Blue.closeScore;
    private final Pose lineUpSpike2 = Poses.Blue.lineUpSpike2;
    private final Pose pickUpSpike2 = Poses.Blue.pickUpSpike2;
    private final Pose exitSpike2 = Poses.Blue.exitSpike2;
    private final Pose exitGate = Poses.Blue.exitGate;
    private final Pose pickUpGate = Poses.Blue.pickUpGate;
    private final Pose lineUpSpike1 = Poses.Blue.lineUpSpike1;
    private final Pose pickUpSpike1 = Poses.Blue.pickUpSpike1;
    private final Pose lineUpSpike3 = Poses.Blue.lineUpSpike3;
    private final Pose pickUpSpike3 = Poses.Blue.pickUpSpike3;
    private final Pose exitSpike3 = Poses.Blue.exitSpike3;
    private final Pose farScore = Poses.Blue.farScore;
    private final Pose farPark = Poses.Blue.farPark;

    // Path variables
    private PathChain farStart_to_closeScore;
    private PathChain closeScore_to_lineUpSpike2;
    private PathChain lineUpSpike2_to_pickUpSpike2;
    private PathChain pickUpSpike2_to_closeScore;
    private PathChain closeScore_to_exitGate;
    private PathChain exitGate_to_pickUpGate;
    private PathChain pickUpGate_to_exitGate;
    private PathChain exitGate_to_closeScore;
    private PathChain closeScore_to_lineUpSpike1;
    private PathChain lineUpSpike1_to_pickUpSpike1;
    private PathChain pickUpSpike1_to_closeScore;
    private PathChain closeScore_to_lineUpSpike3;
    private PathChain lineUpSpike3_to_pickUpSpike3;
    private PathChain pickUpSpike3_to_exitSpike3;
    private PathChain exitSpike3_to_farScore;
    private PathChain farScore_to_farPark;

    public void buildPaths() {
        farStart_to_closeScore = follower.pathBuilder()
                .addPath(new BezierLine(farStart, closeScore))
                .setLinearHeadingInterpolation(farStart.getHeading(), closeScore.getHeading())
                .build();

        closeScore_to_lineUpSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(closeScore, lineUpSpike2))
                .setLinearHeadingInterpolation(closeScore.getHeading(), lineUpSpike2.getHeading())
                .build();

        lineUpSpike2_to_pickUpSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(lineUpSpike2, pickUpSpike2))
                .setLinearHeadingInterpolation(lineUpSpike2.getHeading(), pickUpSpike2.getHeading())
                .build();

        pickUpSpike2_to_closeScore = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpSpike2, exitSpike2, closeScore))
                .setLinearHeadingInterpolation(pickUpSpike2.getHeading(), closeScore.getHeading())
                .build();

        closeScore_to_exitGate = follower.pathBuilder()
                .addPath(new BezierLine(closeScore, exitGate))
                .setLinearHeadingInterpolation(closeScore.getHeading(), exitGate.getHeading())
                .build();

        exitGate_to_pickUpGate = follower.pathBuilder()
                .addPath(new BezierLine(exitGate, pickUpGate))
                .setLinearHeadingInterpolation(exitGate.getHeading(), pickUpGate.getHeading())
                .build();

        pickUpGate_to_exitGate = follower.pathBuilder()
                .addPath(new BezierLine(pickUpGate, exitGate))
                .setLinearHeadingInterpolation(pickUpGate.getHeading(), exitGate.getHeading())
                .build();

        exitGate_to_closeScore = follower.pathBuilder()
                .addPath(new BezierLine(exitGate, closeScore))
                .setLinearHeadingInterpolation(exitGate.getHeading(), closeScore.getHeading())
                .build();

        closeScore_to_lineUpSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(closeScore, lineUpSpike1))
                .setLinearHeadingInterpolation(closeScore.getHeading(), lineUpSpike1.getHeading())
                .build();

        lineUpSpike1_to_pickUpSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(lineUpSpike1, pickUpSpike1))
                .setLinearHeadingInterpolation(lineUpSpike1.getHeading(), pickUpSpike1.getHeading())
                .build();

        pickUpSpike1_to_closeScore = follower.pathBuilder()
                .addPath(new BezierLine(pickUpSpike1, closeScore))
                .setLinearHeadingInterpolation(pickUpSpike1.getHeading(), closeScore.getHeading())
                .build();

        closeScore_to_lineUpSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(closeScore, lineUpSpike3))
                .setLinearHeadingInterpolation(closeScore.getHeading(), lineUpSpike3.getHeading())
                .build();

        lineUpSpike3_to_pickUpSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(lineUpSpike3, pickUpSpike3))
                .setLinearHeadingInterpolation(lineUpSpike3.getHeading(), pickUpSpike3.getHeading())
                .build();

        pickUpSpike3_to_exitSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpSpike3, exitSpike3))
                .setLinearHeadingInterpolation(pickUpSpike3.getHeading(), exitSpike3.getHeading())
                .build();

        exitSpike3_to_farScore = follower.pathBuilder()
                .addPath(new BezierLine(exitSpike3, farScore))
                .setLinearHeadingInterpolation(exitSpike3.getHeading(), farScore.getHeading())
                .build();

        farScore_to_farPark = follower.pathBuilder()
                .addPath(new BezierLine(farScore, farPark))
                .setLinearHeadingInterpolation(farScore.getHeading(), farPark.getHeading())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {
        if (!robot.launcherThread.isBusy()) {
            waitingForLauncher = false;
        } else {
            return;
        }

        switch (pathState) {
            case 0: // Follow farStart_to_closeScore
                if (!follower.isBusy()) {
                    follower.followPath(farStart_to_closeScore, true);
                    setPathState(1);
                }
                break;
            case 1: // LAUNCH
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(2);
                }
                break;
            case 2: // Follow closeScore_to_lineUpSpike2
                if (!follower.isBusy()) {
                    follower.followPath(closeScore_to_lineUpSpike2, true);
                    setPathState(3);
                }
                break;
            case 3: // RUN_INTAKE
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(4);
                }
                break;
            case 4: // Follow lineUpSpike2_to_pickUpSpike2
                if (!follower.isBusy()) {
                    follower.followPath(lineUpSpike2_to_pickUpSpike2, true);
                    setPathState(5);
                }
                break;
            case 5: // STOP_INTAKE
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(6);
                }
                break;
            case 6: // Follow pickUpSpike2_to_closeScore
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSpike2_to_closeScore, true);
                    setPathState(8);
                }
                break;
            case 8: // LAUNCH
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(9);
                }
                break;
            case 9: // Follow closeScore_to_exitGate
                if (!follower.isBusy()) {
                    follower.followPath(closeScore_to_exitGate, true);
                    setPathState(10);
                }
                break;
            case 10: // RUN_INTAKE
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(11);
                }
                break;
            case 11: // Follow exitGate_to_pickUpGate
                if (!follower.isBusy()) {
                    follower.followPath(exitGate_to_pickUpGate, true);
                    setPathState(110);
                }
                break;
            case 110: // Start waiting for artifacts to enter at gate
                if (!follower.isBusy()) {
                    gateEndTime = opmodeTimer.getElapsedTimeSeconds() + 1;
                    setPathState(1100);
                }
                break;
            case 1100: // Switched statement here: waiting in this case not next
                if (opmodeTimer.getElapsedTimeSeconds() > gateEndTime) {
                    robot.stopIntakeAssembly();
                    setPathState(12);
                }
                break;
            case 12: // Follow pickUpGate_to_exitGate
                if (!follower.isBusy()) {
                    follower.followPath(pickUpGate_to_exitGate, true);
                    setPathState(13);
                }
                break;
            case 13: // Follow exitGate_to_closeScore
                if (!follower.isBusy()) {
                    follower.followPath(exitGate_to_closeScore, true);
                    setPathState(14);
                }
                break;
            case 14: // LAUNCH
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(15);
                }
                break;
            case 15: // Follow closeScore_to_lineUpSpike1
                if (!follower.isBusy()) {
                    follower.followPath(closeScore_to_lineUpSpike1, true);
                    setPathState(16);
                }
                break;
            case 16: // RUN_INTAKE
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(17);
                }
                break;
            case 17: // Follow lineUpSpike1_to_pickUpSpike1
                if (!follower.isBusy()) {
                    follower.followPath(lineUpSpike1_to_pickUpSpike1, true);
                    setPathState(18);
                }
                break;
            case 18: // STOP_INTAKE
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(19);
                }
                break;
            case 19: // Follow pickUpSpike1_to_closeScore
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSpike1_to_closeScore, true);
                    setPathState(20);
                }
                break;
            case 20: // LAUNCH
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(21);
                }
                break;
            case 21: // Follow closeScore_to_lineUpSpike3
                if (!follower.isBusy()) {
                    follower.followPath(closeScore_to_lineUpSpike3, true);
                    setPathState(22);
                }
                break;
            case 22: // RUN_INTAKE
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(23);
                }
                break;
            case 23: // Follow lineUpSpike3_to_pickUpSpike3
                if (!follower.isBusy()) {
                    follower.followPath(lineUpSpike3_to_pickUpSpike3, true);
                    setPathState(24);
                }
                break;
            case 24: // STOP_INTAKE
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(25);
                }
                break;
            case 25: // Follow pickUpSpike3_to_exitSpike3
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSpike3_to_exitSpike3, true);
                    setPathState(26);
                }
                break;
            case 26: // Follow exitSpike3_to_farScore
                if (!follower.isBusy()) {
                    follower.followPath(exitSpike3_to_farScore, true);
                    setPathState(27);
                }
                break;
            case 27: // LAUNCH
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(28);
                }
                break;
            case 28: // Follow farScore_to_farPark
                if (!follower.isBusy()) {
                    follower.followPath(farScore_to_farPark, true);
                    setPathState(29);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        robot = new Config(this, follower);
        robot.init();
        robot.setOpModeIsActive(true);
        robot.setAlliance(Config.Alliance.BLUE);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(farStart);
        robot.savePoseToFile(follower.getPose());
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
        robot.setOpModeIsActive(true);
        opmodeTimer.resetTimer();
        robot.launcherThread.idleLauncher();
        setPathState(0);
    }

    @Override
    public void stop() {
        robot.savePoseToFile(follower.getPose());
        robot.setOpModeIsActive(false);
    }
}
