package org.firstinspires.ftc.teamcode.robot; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue Far Auto")
public class BlueFarAutoDev extends OpMode {

    Config robot;

    ElapsedTime runtime = new ElapsedTime();

    boolean waitingForLauncher = false;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //double parkTime = 25;

    private final Pose startPose = new Pose(56, 12, Math.toRadians(90)); // Start Pose of robot.


    private int pathState;

    private PathChain scorePreload, lineUp2, pickUp2, exit2, score2, lineUpGate, grabGate,
            exitGate, scoreGate, lineUp1, pickUp1, score1, lineUp3, pickUp3, score3, park;


    private double pickupSpeed = 0.8;
    private double intakeHighVel = 2000;

    AutoState state = AutoState.SCORE_PRELOAD;

    public enum AutoState {

        // Preload
        SCORE_PRELOAD,

        // Second artifact cycle
        LINE_UP_2,
        PICK_UP_2,
        EXIT_2,
        SCORE_2,

        // Gate cycle
        LINE_UP_GATE,
        GRAB_GATE,
        EXIT_GATE,
        SCORE_GATE,

        // First artifact cycle
        LINE_UP_1,
        PICK_UP_1,
        SCORE_1,

        // Third artifact cycle
        LINE_UP_3,
        PICK_UP_3,
        SCORE_3,

        // End
        PARK,

        // Mechanisms
        LAUNCH,
        IDLE,
        RUN_INTAKE,
        STOP_INTAKE
    }


    public void buildPaths() {

        final Pose scorePose = new Pose(50, 100, robot.aimAssist.getHeadingForTarget(new Pose(50,100), robot.alliance.getPose())); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.
        final Pose scoreParkPose = new Pose(53, 115, robot.aimAssist.getHeadingForTarget(new Pose(53,115), robot.alliance.getPose())); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.

        final Pose lineUp1Pose = new Pose(50, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
        final Pose pickUp1Pose = new Pose(18, 84, Math.toRadians(180)); // !!!!!

        final Pose lineUp2Pose = new Pose(50, 61, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        final Pose pickUp2Pose = new Pose(11, 61, Math.toRadians(180)); // !!!!!
        final Pose exit2Pose = new Pose(50, 63, Math.toRadians(180));

        final Pose lineUpGatePose = new Pose();
        final Pose exitGatePose = new Pose();

        final Pose lineUp3Pose = new Pose(50, 37, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
        final Pose pickUp3Pose = new Pose(11, 37, Math.toRadians(180)); // !!!!!
        final Pose exit3Pose = new Pose(50, 38, Math.toRadians(180));

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line.
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        toPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lineUp1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(lineUp1Pose, pickUp1Pose))
                .setLinearHeadingInterpolation(lineUp1Pose.getHeading(), pickUp1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1Pose, scorePose))
                .setLinearHeadingInterpolation(pickUp1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line.
        toPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lineUp2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(lineUp2Pose, pickUp2Pose))
                .setLinearHeadingInterpolation(lineUp2Pose.getHeading(), pickUp2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUp2Pose, exit2Pose, scorePose))
                .setLinearHeadingInterpolation(exit2Pose.getHeading(), scorePose.getHeading())
                .build();

        toPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lineUp3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(lineUp3Pose, pickUp3Pose))
                .setLinearHeadingInterpolation(lineUp3Pose.getHeading(), pickUp3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUp3Pose, exit3Pose, scoreParkPose))
                .setLinearHeadingInterpolation(exit3Pose.getHeading(), scoreParkPose.getHeading())
                .build();
/
        /**
         * scorePreload
         * lineUp2
         * pickUp2
         * exit2
         * score2
         * lineUpGate
         * grabGate
         * exitGate
         * scoreGate
         * lineUp1
         * pickUp1
         * score1
         * lineUp3
         * pickUp3
         * score3
         * park
         */

        PathChain scorePreload, lineUp2, pickUp2, exit2, score2, lineUpGate, grabGate,
                exitGate, scoreGate, lineUp1, pickUp1, score1, lineUp3, pickUp3, score3, park;


        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        lineUp2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lineUp2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp2Pose.getHeading())
                .build();

        pickUp2 = follower.pathBuilder()
                .addPath(new BezierCurve(lineUp2Pose, pickUp2Pose))
                .setLinearHeadingInterpolation(lineUp2Pose.getHeading(), pickUp2Pose.getHeading())
                .build();

        exit2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUp2Pose, exit2Pose))
                .setLinearHeadingInterpolation(pickUp2Pose.getHeading(), exit2Pose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(exit2Pose, scorePose))
                .setLinearHeadingInterpolation(exit2Pose.getHeading(), scorePose.getHeading())
                .build();

        lineUpGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lineUpGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUpGatePose.getHeading())
                .build();

        grabGate = follower.pathBuilder()
                .addPath(new BezierCurve(lineUpGatePose, exitGatePose))
                .setLinearHeadingInterpolation(lineUpGatePose.getHeading(), exitGatePose.getHeading())
                .build();

        exitGate = follower.pathBuilder()
                .addPath(new BezierCurve(exitGatePose, scorePose))
                .setLinearHeadingInterpolation(exitGatePose.getHeading(), scorePose.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lineUp1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp1Pose.getHeading())
                .build();

        lineUp1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lineUp1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp1Pose.getHeading())
                .build();

        pickUp1 = follower.pathBuilder()
                .addPath(new BezierCurve(lineUp1Pose, pickUp1Pose))
                .setLinearHeadingInterpolation(lineUp1Pose.getHeading(), pickUp1Pose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUp1Pose, scorePose))
                .setLinearHeadingInterpolation(pickUp1Pose.getHeading(), scorePose.getHeading())
                .build();

        lineUp3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, lineUp3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp3Pose.getHeading())
                .build();

        pickUp3 = follower.pathBuilder()
                .addPath(new BezierCurve(lineUp3Pose, pickUp3Pose))
                .setLinearHeadingInterpolation(lineUp3Pose.getHeading(), pickUp3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUp3Pose, scoreParkPose))
                .setLinearHeadingInterpolation(pickUp3Pose.getHeading(), scoreParkPose.getHeading())
                .build();

    }




    /// AutoCommand interface
    public interface AutoCommand {}

    /// Pose Wrapper
    public static class PoseCmd implements AutoCommand {
        public final Pose pose;

        public PoseCmd(Pose pose) {
            this.pose = pose;
        }
    }

    /// Helper for clean syntax
    public static PoseCmd P(Pose pose) {
        return new PoseCmd(pose);
    }

    /// Action commands
    public enum ActionCmd implements AutoCommand {
        RUN_INTAKE,
        STOP_INTAKE,
        LAUNCH,
        IDLE;

        /// Helper for clean syntax
        public static ActionCmd A(ActionCmd actionCmd) {
            return actionCmd;
        }
    }

    /// Action executor
    private void runAction(ActionCmd action) {
        switch (action) {
            case RUN_INTAKE:
                robot.runIntakeAssembly(2000);
                break;
            case STOP_INTAKE:
                robot.stopIntakeAssembly();
                break;
            case LAUNCH:
                robot.launcherThread.setLauncherVelocity(robot.aimAssist.runPowerCalculation(follower.getPose(), robot.alliance.getPose()));
                robot.launcherThread.launchThree();
                break;
            case IDLE:
                robot.launcherThread.idleLauncher();
                break;
        }
    }

    public void build(AutoCommand... commands) {

    }







    public void autonomousPathUpdate() throws InterruptedException {

        if (follower.isBusy() || robot.launcherThread.isBusy()) {
            return;
        }

        switch (state) {
            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                setPathState();
                break;
            case LINE_UP_2:
                follower.followPath(lineUp2);
                setPathState();
                break;
            case PICK_UP_2:
                follower.followPath(pickUp2);
                setPathState();
                break;
        }









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
    public void setPathState(AutoState pState) {
        state = pState;
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

        robot = new Config(this,follower);
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