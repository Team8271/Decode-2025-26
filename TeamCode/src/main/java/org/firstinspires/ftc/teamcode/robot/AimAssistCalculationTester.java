package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "3: AimAssistCalcTester")
public class AimAssistCalculationTester extends LinearOpMode {
    Config robot;
    private Follower follower;

    private final Pose startPose = new Pose(88, 12, Math.toRadians(90)); // Start Pose of robot.
    //private final Pose startPose = new Pose(12, 12, Math.toRadians(0)); // Start Pose of robot.


    @Override
    public void runOpMode() {
        robot = new Config(null, this);
        robot.init();
        robot.setAlliance(Config.Alliance.RED);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();



        //TODO: Not working, Seems Red/Blue flipped Drive issue most likely
        //resetPose(0, 0, robot.alliance == Config.Alliance.RED ? 0 : Math.toRadians(180));

        telemetry.addData("Pose",follower.getPose());
        telemetry.addData("Alliance", robot.alliance);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            double headingCalc = robot.aimAssist.getHeadingForTarget(follower.getPose(),new Pose(144,144));

            telemetry.addData("Current Head",follower.getHeading());
            telemetry.addData("Heading Calc",headingCalc);
            telemetry.addData("Difference  ",follower.getHeading()-headingCalc);

            if(headingCalc > follower.getHeading()) {
                telemetry.addLine("Rotate LEFT");
            }
            else {
                telemetry.addLine("Rotate RIGHT");
            }
            telemetry.update();
            sleep(5);
        }
    }

    private void resetPose(double x, double y, double heading) {
        follower.setPose(new Pose(x,y,heading));
    }
}


