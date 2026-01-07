package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "1 LimelightTester")
public class limelightTester extends LinearOpMode {
    Config robot;
    AimAssist aimAssist;

    double targetTx = -5; // Red Goal

    double speed = 0.2; // Speed for rotating

    @Override
    public void runOpMode() {
        robot = new Config(this, null);
        robot.init();

        robot.setAlliance(Config.Alliance.RED);

        waitForStart();

        while (opModeIsActive()) {
            sleep(15);
            telemetry.addData("Alliance", robot.alliance);
            telemetry.addLine();
            double currentTx = robot.limelight.scanGoalTx();
            telemetry.addData("Current Tx",currentTx);
            telemetry.addData("Target Tx ",targetTx);
            telemetry.addLine();

            if(currentTx > targetTx) {
                robot.setWheelPower(speed,-speed,speed,-speed); // rotate RIGHT
                telemetry.addLine("Rotating RIGHT");
            }
            else {
                robot.setWheelPower(-speed,speed,-speed,speed); // rotate LEFT
                telemetry.addLine("Rotating LEFT");
            }

            telemetry.update();
        }


    }
}
