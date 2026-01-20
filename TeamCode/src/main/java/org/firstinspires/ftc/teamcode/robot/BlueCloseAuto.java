package org.firstinspires.ftc.teamcode.robot; // make sure this aligns with class location

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

@Autonomous(name = "Blue Close Auto")
public class BlueCloseAuto extends OpMode {

    Config robot;

    ElapsedTime runtime = new ElapsedTime();

    boolean waitingForLauncher = false;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //double parkTime = 25;

    private final Pose startPose = new Pose(22.49, 121.03, Math.toRadians(141.10)); // Start Pose of robot.


    private int pathState;

    private Path scorePreload;
    private PathChain toPickup1, grabPickup1, scorePickup1, toPickup2, grabPickup2,
            scorePickup2, toPickup3, grabPickup3, scorePickup3;

    private double pickupSpeed = 0.8;
    private double intakeHighVel = 2000;

    public void buildPaths() {

        final Pose scorePose = new Pose(50, 100, robot.aimAssist.getHeadingForTarget(new Pose(50,100), robot.alliance.getPose())); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.
        final Pose scorePosePark = new Pose(53, 115, robot.aimAssist.getHeadingForTarget(new Pose(53,115), robot.alliance.getPose())); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.

        final Pose toPickup1Pose = new Pose(50, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
        final Pose pickup1Pose = new Pose(19, 84, Math.toRadians(180)); // !!!!!

        final Pose toPickup2Pose = new Pose(50, 61, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        final Pose pickup2Pose = new Pose(12, 61, Math.toRadians(180)); // !!!!!
        final Pose exitGrabPickup2Pose = new Pose(50, 63, Math.toRadians(180));

        final Pose toPickup3Pose = new Pose(50, 37, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        final Pose pickup3Pose = new Pose(12, 37, Math.toRadians(180)); // !!!!!
        final Pose exitGrabPickup3Pose = new Pose(50, 38, Math.toRadians(180));

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        toPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toPickup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(toPickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        toPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toPickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toPickup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(toPickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, exitGrabPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(exitGrabPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        toPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toPickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toPickup3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(toPickup3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, exitGrabPickup3Pose, scorePosePark))
                .setLinearHeadingInterpolation(exitGrabPickup3Pose.getHeading(), scorePosePark.getHeading())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {

        if (!robot.launcherThread.isBusy()) {
            waitingForLauncher = false;
        } else {
            return; // pause auton state machine, without blocking the thread
        }

        switch (pathState) {
            case 0:
                robot.launcherThread.idleLauncher();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    launch();
                    setPathState(100);
                }
                break;
            case 100: // WAITING FOR LAUNCHER TO FINISH BEFORE MOVING
                if (!robot.launcherThread.isBusy()) {
                    follower.followPath(toPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(intakeHighVel);
                    follower.followPath(grabPickup1, pickupSpeed, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    follower.followPath(scorePickup1, 0.8, true);
                    setPathState(30);
                }
                break;
            case 30:
                if (!follower.isBusy()) {
                    launch();
                    setPathState(300);
                }
                break;
            case 300: // WAITING FOR LAUNCHER TO FINISH BEFORE MOVING
                if (!robot.launcherThread.isBusy()) {
                    follower.followPath(toPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.runIntakeAssembly(intakeHighVel);
                    follower.followPath(grabPickup2, pickupSpeed, true);
                    setPathState(5); // Pre parking
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    follower.followPath(scorePickup2,true);
                    setPathState(50);
                }
                break;
            case 50:
                if(!follower.isBusy()) {
                    launch();
                    setPathState(500);
                }
                break;
            case 500:
                if(!robot.launcherThread.isBusy()) {
                    follower.followPath(toPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    robot.runIntakeAssembly(intakeHighVel);
                    follower.followPath(grabPickup3, pickupSpeed, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    robot.stopIntakeAssembly();
                    follower.followPath(scorePickup3);
                    setPathState(70);
                }
                break;
            case 70:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(700);
                }
                break;
            case 700:
                if(!follower.isBusy()) {
                    launch();
                    setPathState(701);
                }
                break;
            case 701:
                if(!robot.launcherThread.isBusy()) {
                    robot.stopIntakeAssembly();
                    robot.launcherMotor.setVelocity(0);
                    setPathState(-2);
                }
                break;


        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            log("Interrupted wait: " + e);
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {

        robot = new Config(this, follower);
        robot.init();
        setOpModeIsActive(true);

        robot.setAlliance(Config.Alliance.BLUE);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        robot.savePoseToFile(follower.getPose());

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        runtime.reset();
        setOpModeIsActive(true);
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * Runs at the end of opMode life.
     **/
    @Override
    public void stop() {
        robot.savePoseToFile(follower.getPose());
        setOpModeIsActive(false);

    }

    public void setOpModeIsActive(boolean value) {
        robot.opModeIsActive = value;
    }

    private void log(String message) {
        robot.log("[RedFarAuto] - " + message);
    }

    private void launch() {
        robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
        robot.launcherThread.launchThree();
    }


}