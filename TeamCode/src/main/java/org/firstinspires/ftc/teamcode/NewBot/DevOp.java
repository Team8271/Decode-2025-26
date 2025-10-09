package org.firstinspires.ftc.teamcode.NewBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DevOp")
public class DevOp extends LinearOpMode {
    @Override
    public void runOpMode(){
        DevConfig robot = new DevConfig(this);
        robot.devInit();
        boolean debounce = false;
        double agitatorPower = 0;

        double leftTiltStartingPos = robot.leftTilt.getPosition();
        double rightTiltStartingPos = robot.rightTilt.getPosition();
        robot.leftTilt.setPosition(leftTiltStartingPos);
        robot.rightTilt.setPosition(rightTiltStartingPos);

        telemetry.addData("LeftTilt Start",leftTiltStartingPos);
        telemetry.addData("RightTilt Start",rightTiltStartingPos);

        telemetry.addData("LeftTilt Pos", robot.leftTilt.getPosition());
        telemetry.addData("RightTilt Pos", robot.rightTilt.getPosition());

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            double launcherControl = gamepad2.right_stick_y;

            double launcherPower = Range.clip(launcherControl,-1,1);
            robot.leftLauncher.setPower(launcherPower);
            robot.rightLauncher.setPower(launcherPower);


            if(gamepad1.a && !debounce){
                if(agitatorPower == 0){
                    agitatorPower = 1;
                }
                else{
                    agitatorPower = 0;
                }
                robot.agitator.setPower(agitatorPower);
                debounce = true;
            }

            if(!gamepad1.a && debounce){
                debounce = false;
            }


            robot.leftTilt.setPosition(robot.leftTilt.getPosition());
            robot.rightTilt.setPosition(robot.rightTilt.getPosition());

            telemetry.addData("Launcher Power", launcherPower);

            telemetry.addData("LeftTilt Start",leftTiltStartingPos);
            telemetry.addData("RightTilt Start",rightTiltStartingPos);

            telemetry.addData("LeftTilt Pos", robot.leftTilt.getPosition());
            telemetry.addData("RightTilt Pos", robot.rightTilt.getPosition());
            telemetry.update();
        }
    }
}
