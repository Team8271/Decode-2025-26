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

@Autonomous(name = "Generated Auto1")
public class GeneratedAuto1 extends OpMode {

    Config robot;
    ElapsedTime runtime = new ElapsedTime();
    boolean waitingForLauncher = false;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56.0,9.0,1.5707963267948966);

    private final Pose pose1 = new Pose(42.8,92.2,2.260201381332657);
    private final Pose pose2 = new Pose(50.0,57.9,3.141592653589793);
    private final Pose pose3 = new Pose(10.6,57.9,3.141592653589793);
    private final Pose pose4 = new Pose(24.0,57.9,3.141592653589793);
    private final Pose pose5 = new Pose(42.8,92.2,2.260201381332657);
    private final Pose pose6 = new Pose(23.5,58.0,2.428800187075309);
    private final Pose pose7 = new Pose(13.34,57.85,2.4687682269459787);
    private final Pose pose8 = new Pose(23.5,58.0,2.428800187075309);
    private final Pose pose9 = new Pose(42.8,92.2,2.260201381332657);
    private final Pose pose10 = new Pose(50.0,80.7,3.141592653589793);
    private final Pose pose11 = new Pose(18.5,80.7,3.141592653589793);
    private final Pose pose12 = new Pose(42.8,92.2,2.260201381332657);
    private final Pose pose13 = new Pose(50.0,34.6,3.141592653589793);
    private final Pose pose14 = new Pose(11.7,34.6,3.141592653589793);
    private final Pose pose15 = new Pose(24.0,34.6,3.141592653589793);
    private final Pose pose16 = new Pose(57.6,17.0,1.9884536167971398);
    private final Pose pose17 = new Pose(60.3,12.1,1.5707963267948966);

    // Path variables
    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;
    private PathChain path8;
    private PathChain path9;
    private PathChain path10;
    private PathChain path11;
    private PathChain path12;
    private PathChain path13;
    private PathChain path14;
    private PathChain path15;
    private PathChain path16;
    private PathChain path17;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose6, pose7))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(pose9, pose10))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(pose10, pose11))
                .setLinearHeadingInterpolation(pose10.getHeading(), pose11.getHeading())
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(pose11, pose12))
                .setLinearHeadingInterpolation(pose11.getHeading(), pose12.getHeading())
                .build();

        path13 = follower.pathBuilder()
                .addPath(new BezierLine(pose12, pose13))
                .setLinearHeadingInterpolation(pose12.getHeading(), pose13.getHeading())
                .build();

        path14 = follower.pathBuilder()
                .addPath(new BezierLine(pose13, pose14))
                .setLinearHeadingInterpolation(pose13.getHeading(), pose14.getHeading())
                .build();

        path15 = follower.pathBuilder()
                .addPath(new BezierLine(pose14, pose15))
                .setLinearHeadingInterpolation(pose14.getHeading(), pose15.getHeading())
                .build();

        path16 = follower.pathBuilder()
                .addPath(new BezierLine(pose15, pose16))
                .setLinearHeadingInterpolation(pose15.getHeading(), pose16.getHeading())
                .build();

        path17 = follower.pathBuilder()
                .addPath(new BezierLine(pose16, pose17))
                .setLinearHeadingInterpolation(pose16.getHeading(), pose17.getHeading())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {
        if (!robot.launcherThread.isBusy()) {
            waitingForLauncher = false;
        } else {
            return;
        }

        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(path1, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(path6, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(path8, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(path9, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(path10, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(path11, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(path12, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(path13, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(2000);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    follower.followPath(path14, true);
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    setPathState(25);
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    follower.followPath(path15, true);
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(path16, true);
                    setPathState(27);
                }
                break;
            case 27:
                if (!follower.isBusy()) {
                    robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                    robot.launcherThread.launchThree();
                    setPathState(28);
                }
                break;
            case 28:
                if (!follower.isBusy()) {
                    follower.followPath(path17, true);
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
        follower.setStartingPose(startPose);
        robot.savePoseToFile(follower.getPose());
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
        robot.setOpModeIsActive(true);
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        robot.savePoseToFile(follower.getPose());
        robot.setOpModeIsActive(false);
    }
}
