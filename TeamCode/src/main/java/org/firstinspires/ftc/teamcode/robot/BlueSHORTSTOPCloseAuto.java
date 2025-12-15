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

@Autonomous(name = "Blue SHORTSTOP Close Auto")
public class BlueSHORTSTOPCloseAuto extends OpMode {

    Config robot;

    ElapsedTime runtime = new ElapsedTime();

    boolean waitingForLauncher = false;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    double parkTime = 25;


    private int pathState;

    private final Pose startPose = new Pose(27, 132, Math.toRadians(144)); // Start Pose of robot.
    private final Pose scorePose = new Pose(50, 100, Math.toRadians(144)); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.
    private final Pose scorePose1 = new Pose(50, 100, Math.toRadians(135)); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.

    private final Pose toPickup1Pose = new Pose(50, 86, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(18, 86, Math.toRadians(180));

    private final Pose parkPose = new Pose(50,120, Math.toRadians(180));

    private Path scorePreload;
    private PathChain toPickup1, grabPickup1, scorePickup1, park;

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

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, parkPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), parkPose.getHeading())
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
                break;
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
                    setPathState(301);
                }
                break;
            case 301: // WAITING FOR LAUNCHER TO FINISH BEFORE PARKING
                if (!waitingForLauncher) {
                    follower.followPath(park, true);
                    robot.stopIntakeAssembly();
                    robot.launcherMotor.setVelocity(0);
                    setPathState(-1);
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
        robot.log("[BlueSHORTSTOPCloseAuto] - " + message);
    }

    private void launch() {
        //robot.aimAssist.runAngleCorrection(2);
        //robot.launcherThread.launch(3,true);
        waitingForLauncher = true;

    }


}