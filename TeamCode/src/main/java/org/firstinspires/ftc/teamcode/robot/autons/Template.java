package org.firstinspires.ftc.teamcode.robot.autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;
import org.firstinspires.ftc.teamcode.util.AutoMaker;
import org.firstinspires.ftc.teamcode.util.AutoMaker.Sequence;

@Disabled
@Autonomous(name = "DS Name", group = "autons", preselectTeleOp = "")
public class Template extends OpMode {

    Config robot;
    Follower follower;
    AutoMaker autoMaker;
    Sequence sequence;



    @Override
    public void init() {
        robot = new Config(this);
        robot.init();
        robot.setOpModeIsActive(true);

        robot.setAlliance(Config.Alliance.BLUE);

        follower = Constants.createFollower(hardwareMap);

        autoMaker = new AutoMaker(robot, follower);

        final Pose startPose = new Pose(56, 12, Math.toRadians(90)); // Start Pose of robot.

        final Pose scorePose = new Pose(50, 100, robot.aimAssist.getHeadingForTarget(new Pose(50,100), robot.alliance.getPose())); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.
        final Pose scorePosePark = new Pose(53, 115, robot.aimAssist.getHeadingForTarget(new Pose(53,115), robot.alliance.getPose())); // Scoring Pose of robot. It is facing the goal at a 144 degree angle.

        final Pose toPickup1Pose = new Pose(50, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
        final Pose pickup1Pose = new Pose(18, 84, Math.toRadians(180)); // !!!!!

        final Pose toPickup2Pose = new Pose(50, 61, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        final Pose pickup2Pose = new Pose(11, 61, Math.toRadians(180)); // !!!!!
        final Pose exitGrabPickup2Pose = new Pose(50, 63, Math.toRadians(180));

        final Pose toPickup3Pose = new Pose(50, 37, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        final Pose pickup3Pose = new Pose(11, 37, Math.toRadians(180)); // !!!!!
        final Pose exitGrabPickup3Pose = new Pose(50, 38, Math.toRadians(180));

        sequence = autoMaker.build(
                autoMaker.P(startPose),
                autoMaker.P(scorePose),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(toPickup2Pose),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(pickup2Pose),
                autoMaker.P(exitGrabPickup2Pose),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(scorePose),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(toPickup2Pose),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(pickup2Pose),
                autoMaker.P(exitGrabPickup2Pose),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(scorePose),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(toPickup1Pose),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(pickup1Pose),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(scorePose),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(toPickup3Pose),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(pickup3Pose),
                autoMaker.P(exitGrabPickup3Pose),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(scorePose),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH)

        );
    }

    @Override
    public void loop() {
        follower.update();
        autoMaker.updateSequence(sequence);

        // Feedback to Driver Hub for debugging
        telemetry.addData("index", autoMaker.getCommandIndex());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.setOpModeIsActive(true);
    }

    @Override
    public void stop() {
        robot.savePoseToFile(follower.getPose());
        robot.setOpModeIsActive(false);
    }
}
