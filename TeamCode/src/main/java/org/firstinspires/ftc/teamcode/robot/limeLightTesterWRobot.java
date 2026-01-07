package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "2 limeLightTesterWRobot")
public class limeLightTesterWRobot extends LinearOpMode {
    Config robot;
    private Follower follower;


    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        robot = new Config(this, null);
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            double targetHeading = robot.aimAssist.runHeadingCalculation(follower.getPose());
            telemetry.addData("Heading", follower.getHeading());
            telemetry.addData("Target ", targetHeading);
            telemetry.update();

        }
    }

}
