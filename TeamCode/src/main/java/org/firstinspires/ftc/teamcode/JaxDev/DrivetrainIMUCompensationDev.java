package org.firstinspires.ftc.teamcode.JaxDev;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import dev.narlyx.tweetybird.Odometers.ThreeWheeled;

@TeleOp(name="DtIMUCompDev")
public class DrivetrainIMUCompensationDev extends LinearOpMode {
    IMU imu;

    @Override
    public void runOpMode(){
        DcMotor fl, fr, bl, br;
        ThreeWheeled odometer;
        double headingToHold = 0;
        boolean holdingHeading = false;

        // IMU
        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Front Left Drive
        fl = hardwareMap.get(DcMotor.class, "FL");
        fl.setDirection(DcMotor.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Front Right Drive
        fr = hardwareMap.get(DcMotor.class, "FR");
        fr.setDirection(DcMotor.Direction.FORWARD);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Left Drive
        bl = hardwareMap.get(DcMotor.class, "BL");
        bl.setDirection(DcMotor.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Back Right Drive
        br = hardwareMap.get(DcMotor.class, "BR");
        br.setDirection(DcMotor.Direction.FORWARD);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Build odometers for TweetyBird Use
        odometer = new ThreeWheeled.Builder()
                .setLeftEncoder(bl)
                .setRightEncoder(fr)
                .setMiddleEncoder(br)

                .setEncoderTicksPerRotation(2000)
                .setEncoderWheelRadius(0.944882)

                //Change the true/false values to correct directions
                .setFlipLeftEncoder(true)
                .setFlipRightEncoder(true)
                .setFlipMiddleEncoder(true)

                .setSideEncoderDistance(12)
                .setMiddleEncoderOffset(9.75)
                .build();

        waitForStart();

        while(opModeIsActive()){
            // Driver One Controls
            double axialControl = -gamepad1.left_stick_y;  // y axis
            double lateralControl = gamepad1.left_stick_x; // x axis
            double yawControl = gamepad1.right_stick_x;    // z axis
            double mainThrottle = .2+(gamepad1.right_trigger*0.8); // throttle
            boolean resetFCD = gamepad1.dpad_up; // z axis reset

            // FCD reset
            if(resetFCD){
                odometer.resetTo(0,0,0);
            }

            // Calculate drive train power for field centric
            double gamepadRadians = Math.atan2(lateralControl, axialControl);
            double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
            double robotRadians = -odometer.getZ();
            double targetRadians = gamepadRadians + robotRadians;
            double lateral = Math.sin(targetRadians)*gamepadHypot;
            double axial = Math.cos(targetRadians)*gamepadHypot;
            double yawCorrection = 0;

            // Get imuYaw with values 0-360
            double imuYaw = imu.getRobotYawPitchRollAngles().getYaw() + 180;


            // Lock targetRadians when not moving yaw
            if(yawControl == 0 && !holdingHeading){
                headingToHold = robotRadians;
                holdingHeading = true;

            }
            // Hold Heading
            else if(yawControl == 0 && holdingHeading){
                if(robotRadians > headingToHold - 1){
                    yawCorrection = 0.05; // Need to scale with distance from
                }
                if(robotRadians < headingToHold + 1){
                    yawCorrection = -0.05;
                }

            }
            // Don't interfere with driver
            else{
                holdingHeading = false;
                yawCorrection = 0;
            }




            double leftFrontPower = axial + lateral + yawControl + yawCorrection;
            double rightFrontPower = axial - lateral - yawControl - yawCorrection;
            double leftBackPower = axial - lateral + yawControl + yawCorrection;
            double rightBackPower = axial + lateral - yawControl - yawCorrection;

            fl.setPower(leftFrontPower);
            fr.setPower(rightFrontPower);
            bl.setPower(leftBackPower);
            br.setPower(rightBackPower);

            telemetry.addData("yawControl",yawControl);

            telemetry.addData("gamepadRadians",gamepadRadians);
            telemetry.addData("gamepadHypot",gamepadHypot);
            telemetry.addData("robotRadians",robotRadians);
            telemetry.addData("targetRadians",targetRadians);
            telemetry.addData("imuYaw",imuYaw);
            telemetry.update();
        }

    }

}
