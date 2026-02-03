package org.firstinspires.ftc.teamcode.robot.autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;
import org.firstinspires.ftc.teamcode.util.Poses;

@Autonomous(name = "Blue Close - GA 5")
public class BlueCloseAutoGA5 extends OpMode {

    Config robot;
    ElapsedTime runtime = new ElapsedTime();
    boolean waitingForLauncher = false;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private double gateEndTime;

    private final Pose closeStart = Poses.Blue.closeStart;

    private final Pose closeScore = Poses.Blue.closeScore;
    private final Pose lineUpSpike2 = Poses.Blue.lineUpSpike2;
    private final Pose pickUpSpike2 = Poses.Blue.pickUpSpike2;
    private final Pose openGate = Poses.Blue.openGate;
    private final Pose exitGate = Poses.Blue.exitGate;
    private final Pose lineUpSpike1 = Poses.Blue.lineUpSpike1;
    private final Pose pickUpSpike1 = Poses.Blue.pickUpSpike1;
    private final Pose lineUpSpike3 = Poses.Blue.lineUpSpike3;
    private final Pose pickUpSpike3 = Poses.Blue.pickUpSpike3;
    private final Pose exitSpike3 = Poses.Blue.exitSpike3;
    private final Pose closeScorePark = Poses.Blue.closeScorePark;

    // Path variables
    private PathChain farStart_to_closeScore;
    private PathChain closeScore_to_lineUpSpike2;
    private PathChain lineUpSpike2_to_pickUpSpike2;
    private PathChain pickUpSpike2_to_openGate;
    private PathChain openGate_to_closeScore;
    private PathChain closeScore_to_lineUpSpike1;
    private PathChain lineUpSpike1_to_pickUpSpike1;
    private PathChain pickUpSpike1_to_closeScore;
    private PathChain closeScore_to_lineUpSpike3;
    private PathChain lineUpSpike3_to_pickUpSpike3;
    private PathChain pickUpSpike3_to_closeScorePark;

    public void buildPaths() {
        farStart_to_closeScore = follower.pathBuilder()
                .addPath(new BezierLine(closeStart, closeScore))
                .setLinearHeadingInterpolation(closeStart.getHeading(), closeScore.getHeading())
                .build();

        closeScore_to_lineUpSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(closeScore, lineUpSpike2))
                .setLinearHeadingInterpolation(closeScore.getHeading(), lineUpSpike2.getHeading())
                .build();

        lineUpSpike2_to_pickUpSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(lineUpSpike2, pickUpSpike2))
                .setLinearHeadingInterpolation(lineUpSpike2.getHeading(), pickUpSpike2.getHeading())
                .build();

        pickUpSpike2_to_openGate = follower.pathBuilder()
                .addPath(new BezierLine(pickUpSpike2, openGate))
                .setLinearHeadingInterpolation(pickUpSpike2.getHeading(), openGate.getHeading())
                .build();

        openGate_to_closeScore = follower.pathBuilder()
                .addPath(new BezierCurve(openGate, exitGate, closeScore))
                .setLinearHeadingInterpolation(openGate.getHeading(), closeScore.getHeading())
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

        pickUpSpike3_to_closeScorePark = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpSpike3, exitSpike3, closeScorePark))
                .setLinearHeadingInterpolation(pickUpSpike3.getHeading(), closeScorePark.getHeading())
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
            case 6: // Follow pickUpSpike2_to_openGate
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSpike2_to_openGate, true);
                    setPathState(61);
                }
                break;
            case 61: // Wait for gate
                if (!follower.isBusy()) {
                    gateEndTime = opmodeTimer.getElapsedTimeSeconds()+1;
                    setPathState(7);
                }
                break;
            case 7: // Follow openGate_to_closeScore
                if (!follower.isBusy() && opmodeTimer.getElapsedTimeSeconds()>gateEndTime) {
                    follower.followPath(openGate_to_closeScore, true);
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
            case 9: // Follow closeScore_to_lineUpSpike1
                if (!follower.isBusy()) {
                    follower.followPath(closeScore_to_lineUpSpike1, true);
                    setPathState(10);
                }
                break;
            case 10: // RUN_INTAKE
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(11);
                }
                break;
            case 11: // Follow lineUpSpike1_to_pickUpSpike1
                if (!follower.isBusy()) {
                    follower.followPath(lineUpSpike1_to_pickUpSpike1, true);
                    setPathState(12);
                }
                break;
            case 12: // STOP_INTAKE
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(13);
                }
                break;
            case 13: // Follow pickUpSpike1_to_closeScore
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSpike1_to_closeScore, true);
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
            case 15: // Follow closeScore_to_lineUpSpike3
                if (!follower.isBusy()) {
                    follower.followPath(closeScore_to_lineUpSpike3, true);
                    setPathState(16);
                }
                break;
            case 16: // RUN_INTAKE
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(17);
                }
                break;
            case 17: // Follow lineUpSpike3_to_pickUpSpike3
                if (!follower.isBusy()) {
                    follower.followPath(lineUpSpike3_to_pickUpSpike3, true);
                    setPathState(18);
                }
                break;
            case 18: // STOP_INTAKE
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(19);
                }
                break;
            case 19: // Follow pickUpSpike3_to_closeScorePark
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSpike3_to_closeScorePark, true);
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
        follower.setStartingPose(closeStart);
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
