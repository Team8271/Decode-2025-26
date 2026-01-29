package org.firstinspires.ftc.teamcode.robot.autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;
import org.firstinspires.ftc.teamcode.util.AutoMaker;

public class BlueDevAuto extends OpMode {

    Config robot;
    Follower follower;
    AutoMaker autoMaker;



    @Override
    public void init() {

    }

    @Override
    public void loop() {
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

        autoMaker.build(
                autoMaker.P(startPose),
                autoMaker.P(new Pose())
        );
    }

    @Override
    public void start() {
        robot.setOpModeIsActive(true);
    }

    @Override
    public void stop() {

    }
}
