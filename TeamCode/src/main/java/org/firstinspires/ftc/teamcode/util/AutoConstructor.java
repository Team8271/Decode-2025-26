package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.configuration.Config;

import org.firstinspires.ftc.teamcode.util.Poses.Blue;

/**
 * A class used to auto build pedro pathing autons. It outputs to the robot in the generated
 * autons folder.
 */
@TeleOp(name = "Construct Auto")
public class AutoConstructor extends LinearOpMode {

    public void runOpMode() {

        waitForStart();

        Config robot = new Config(this);
        robot.init();

        robot.setOpModeIsActive(true);

        robot.setAlliance(Config.Alliance.BLUE);

        Follower follower = Constants.createFollower(hardwareMap);

        AutoMaker autoMaker = new AutoMaker(robot, follower);


        /*AutoMaker.BlankSequence seq = autoMaker.buildBlankSequence(
                autoMaker.P(Poses.Blue.farStart),
                autoMaker.P(Poses.Blue.closeScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Poses.Blue.lineUpSpike2),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(Poses.Blue.pickUpSpike2),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(Poses.Blue.exitSpike2),
                autoMaker.P(Poses.Blue.closeScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Poses.Blue.lineUpGate),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(Poses.Blue.pickUpGate),
                autoMaker.P(Poses.Blue.exitGate),
                autoMaker.P(Poses.Blue.closeScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Poses.Blue.lineUpSpike1),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(Poses.Blue.pickUpSpike1),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(Poses.Blue.closeScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Poses.Blue.lineUpSpike3),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(Poses.Blue.pickUpSpike3),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(Poses.Blue.exitSpike3),
                autoMaker.P(Poses.Blue.farScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Poses.Blue.farPark)
        );*/
        AutoMaker.BlankSequence seq = autoMaker.buildBlankSequence(
                autoMaker.P(Blue.farStart),
                autoMaker.P(Blue.closeScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Blue.lineUpSpike2),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(Blue.pickUpSpike2),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(Blue.openGate),
                autoMaker.P(Blue.closeScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Blue.lineUpSpike1),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(Blue.pickUpSpike1),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(Blue.closeScore),
                autoMaker.A(AutoMaker.ActionCmd.LAUNCH),
                autoMaker.P(Blue.lineUpSpike3),
                autoMaker.A(AutoMaker.ActionCmd.RUN_INTAKE),
                autoMaker.P(Blue.pickUpSpike3),
                autoMaker.A(AutoMaker.ActionCmd.STOP_INTAKE),
                autoMaker.P(Blue.closeScorePark)
        );

        autoMaker.printCase(seq);
    }
}
