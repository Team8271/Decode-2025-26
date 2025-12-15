package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Dev1")
public class DevTele extends LinearOpMode {

    Config robot;

    @Override
    public void runOpMode() {
        robot = new Config(this, null);

        robot.init();
        robot.agitator.setTargetPosition(-1000);
        robot.launcherMotor.setTargetPosition(0);
        robot.agitator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcherMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.agitator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.launcherMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a) {
                robot.agitator.setPower(0.5);
            }
            if(gamepad1.b) {
                robot.agitator.setPower(0);
            }

            telemetry.addData("Agitator Position",robot.agitator.getCurrentPosition());
            telemetry.addData("Agitator DesPos  ",robot.agitator.getTargetPosition());
            telemetry.addData("Launcher Position",robot.launcherMotor.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
    }
}