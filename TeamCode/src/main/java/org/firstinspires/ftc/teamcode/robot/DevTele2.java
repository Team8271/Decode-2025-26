package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Dev2")
public class DevTele2 extends LinearOpMode {


    @Override
    public void runOpMode() {

        DcMotorEx agitator;

        agitator = hardwareMap.get(DcMotorEx.class, "agitator");
        agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator.setTargetPosition(500);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a) {
                agitator.setPower(0.5);
            }
            if(gamepad1.b) {
                agitator.setPower(0);
            }

            telemetry.addData("Agitator Position",agitator.getCurrentPosition());
            telemetry.addData("Agitator DesPos  ",agitator.getTargetPosition());
            telemetry.update();
            sleep(50);
        }
    }
}