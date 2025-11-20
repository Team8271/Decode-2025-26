package org.firstinspires.ftc.teamcode.robot; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Close Auto")
public class RedCloseAuto extends OpMode {

    Config robot;

    ElapsedTime runtime = new ElapsedTime();

    boolean waitingForLauncher = false;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    double parkTime = 25;


    private int pathState;

    private final Pose startPose = new Pose(117, 132, Math.toRadians(36)); // Start Pose of robot.
    private final Pose scorePose = new Pose(94, 100, Math.toRadians(36)); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.
    private final Pose scorePose1 = new Pose(94,100, Math.toRadians(39));

    private final Pose toPickup1Pose = new Pose(94, 86, Math.toRadians(360)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(127, 86, Math.toRadians(360)); // !!!!!

    private final Pose toPickup2Pose = new Pose(94, 63, Math.toRadians(360)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(135, 63, Math.toRadians(360)); // !!!!!

    private final Pose exitGrabPickup2Pose = new Pose(94,63, Math.toRadians(360));

    private final Pose toPickup3Pose = new Pose(94, 37, Math.toRadians(360)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(114, 37, Math.toRadians(360)); // !!!!!

    private final Pose parkPose = new Pose(94,115, Math.toRadians(36));

    private Path scorePreload;
    private PathChain toPickup1, grabPickup1, scorePickup1, toPickup2, grabPickup2, exitGrabPickup2, scorePickup2, toPickup3, grabPickup3, scorePickup3, park;

    private double pickupSpeed = 0.3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        toPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toPickup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(toPickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup1Pose, scorePose1))
                .setLinearHeadingInterpolation(toPickup1Pose.getHeading(), scorePose1.getHeading())
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

        exitGrabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, exitGrabPickup2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),exitGrabPickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(toPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        toPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toPickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toPickup3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(toPickup3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup3Pose, scorePose))
                .setLinearHeadingInterpolation(toPickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {

        if (waitingForLauncher) {
            if (!robot.launcherThread.isBusy()) {
                waitingForLauncher = false;
            } else {
                return; // pause auton state machine, without blocking the thread
            }
        }

        switch (pathState) {
            case 0:
                robot.launcherMotor.setVelocity(robot.idealLauncherVelocity); // Start spinning wheel
                follower.followPath(scorePreload);
                robot.runIntakeAssembly();
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
                    robot.runIntakeAssembly();
                    launch();
                    setPathState(100);
                }
                break;
            case 100: // WAITING FOR LAUNCHER TO FINISH BEFORE MOVING
                if (!waitingForLauncher) {
                    follower.followPath(toPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, pickupSpeed, true);
                    setPathState(3);
                }
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
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
                if (!waitingForLauncher) {
                    follower.followPath(toPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, pickupSpeed, true);
                    setPathState(401); // Pre parking
                }
                break;
            case 401:
                if(!follower.isBusy()) {
                    follower.followPath(exitGrabPickup2,true);
                    robot.stopIntakeAssembly();
                    robot.launcherMotor.setVelocity(0);
                    setPathState(-1);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(50);
                }
                break;
            case 50:
                if (!follower.isBusy()) {
                    launch();
                    setPathState(500);
                }
                break;
            case 500: // WAITING FOR LAUNCHER TO FINISH BEFORE MOVING
                if (!waitingForLauncher) {
                    follower.followPath(toPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup3, pickupSpeed, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(70);
                }
                break;
            case 70:
                if (!follower.isBusy()) {
                    launch();
                    setPathState(700);
                }
                break;
            case 700: // WAITING FOR LAUNCHER TO FINISH BEFORE MOVING
                if (!waitingForLauncher) {
                    follower.followPath(toPickup3, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3, pickupSpeed, true);
                    setPathState(9);
                }
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(90);
                }
                break;
            case 90:
                if (!follower.isBusy()) {
                    launch();
                    setPathState(900);
                }
                break;
            case 900: // WAITING FOR LAUNCHER TO FINISH BEFORE MOVING
                if (!waitingForLauncher) {
                    follower.followPath(park, false);
                    robot.stopIntakeAssembly();
                    setOpModeIsActive(false);
                    setPathState(-1);
                }
                break;
            case 10: // Park
                if(waitingForLauncher) {
                    // CANCEL LAUNCH
                    // !! !! !! !! !! !!
                }
                follower.followPath(park);
                robot.stopIntakeAssembly();

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

        robot = new Config(null, this);
        robot.init();
        setOpModeIsActive(true);

        robot.setAlliance(Config.Alliance.BLUE);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

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
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        setOpModeIsActive(false);
    }

    public void setOpModeIsActive(boolean value) {
        robot.opModeIsActive = value;
    }

    private void log(String message) {
        robot.log("[ExampleAuto] - " + message);
    }

    private void launch() {
        //robot.aimAssist.runAngleCorrection(2);
        robot.launcherThread.launch(3,true);
        waitingForLauncher = true;

    }


}