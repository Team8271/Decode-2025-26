package org.firstinspires.ftc.teamcode.NewBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DevOp")
public class DevOp extends LinearOpMode {
    @Override
    public void runOpMode(){
        Config robot = new Config(this);
        robot.init();
        boolean debounce = false;
        double agitatorPower = 0;

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            // Driver 1 Controls
            double axialControl = -gamepad1.left_stick_y;  // y axis
            double lateralControl = gamepad1.left_stick_x; // x axis
            double yawControl = gamepad1.right_stick_x;    // z axis
            double mainThrottle = .2+(gamepad1.right_trigger*0.8); // throttle
            boolean resetFCD = gamepad1.dpad_up; // z axis reset

            // Driver 2 Controls
            boolean launchArtifactControl = gamepad2.a;
            boolean activateAgitator = gamepad2.b;


            if(launchArtifactControl){
                robot.launcherThread.launchOneArtifact();
            }


            if(activateAgitator && !debounce){
                if(agitatorPower == 1){
                    agitatorPower = 0;
                }
                else {
                    agitatorPower = 1;
                }
                debounce = true;
            }

            if(robot.intakeServo.getPosition() != 1){
                robot.intakeServo.setPosition(1);
            }


            if(!activateAgitator && debounce){
                debounce = false;
            }

            robot.agitator.setPower(agitatorPower);


            // FCD reset
            if(resetFCD){
                robot.odometer.resetTo(0,0,180);
            }

            // Calculate drive train power for field centric
            double gamepadRadians = Math.atan2(lateralControl, axialControl);
            double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
            double robotRadians = -robot.odometer.getZ();
            double targetRadians = gamepadRadians + robotRadians;
            double lateral = Math.sin(targetRadians)*gamepadHypot;
            double axial = Math.cos(targetRadians)*gamepadHypot;

            double leftFrontPower = axial + lateral + yawControl;
            double rightFrontPower = axial - lateral - yawControl;
            double leftBackPower = axial - lateral + yawControl;
            double rightBackPower = axial + lateral - yawControl;

            // Calculate and send power to drivetrain
            robot.fl.setPower(leftFrontPower * mainThrottle);
            robot.fr.setPower(rightFrontPower * mainThrottle);
            robot.bl.setPower(leftBackPower * mainThrottle);
            robot.br.setPower(rightBackPower * mainThrottle);

            telemetry.addData("KickerServo Position", robot.kickerServo.getPosition());

            telemetry.update();
        }
    }
}
